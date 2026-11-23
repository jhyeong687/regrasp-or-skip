"""SO-ARM101 R7 contact simulation and local RGB-D/tactile dashboard."""
import argparse
import base64
from collections import deque
import io
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import mujoco as mj
import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
SETUP = json.loads((HERE / "setup.json").read_text())
POINTS = json.loads((HERE / "pxsr_stdcp03b_points.json").read_text())["points"]
TAXELS = np.array([[point[f"{axis}_mm"] for axis in "xyz"] for point in POINTS]) / 1000
COMPRESSION_POSITIVE = np.array([1., 1., -1.])
PATCH_ALLOWANCE_M = .0004
CONTACT_NORMAL_N = .15
ACTIVE_POINT_N = .02
HISTORY_S = .10
SIDES = ("left", "right")
SCENARIOS = {
    "normal": (0.05, 0.8),
    "slippery": (0.05, 0.018),
    "heavy": (0.6, 0.30),
    "heavy_matched": (0.6, 0.8),
    "round": (0.05, 0.8),
}
ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
KEYS = [(t, np.radians(q), slide, label) for t, q, slide, label in SETUP["keyframes"]]
DURATION = KEYS[-1][0]


def trajectory(t):
    for (t0, q0, d0, p0), (t1, q1, d1, p1) in zip(KEYS, KEYS[1:]):
        if t <= t1:
            u = np.clip((t-t0)/(t1-t0), 0., 1.)
            blend = u**3 * (10 + u * (-15 + 6*u))
            return q0 + blend * (q1-q0), d0 + blend * (d1-d0), p1
    _, q, d, phase = KEYS[-1]
    return q, d, phase


def jpeg(image):
    buf = io.BytesIO()
    Image.fromarray(image).save(buf, "JPEG", quality=82)
    return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


class Sim:
    """Model coordinates: radians, metres, seconds, newtons and N·m."""

    def __init__(self, scenario="normal"):
        self.renderers = {}
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.round_object = None
        self.reset(scenario)

    def _load_model(self, round_object):
        self.close()
        assets = {}
        if round_object:
            object_xml = (HERE / "object.xml").read_text()
            object_xml = object_xml.replace('type="box" size="0.016 0.017 0.04023"',
                                            'type="cylinder" size="0.017 0.04023"')
            object_xml = object_xml.replace('rgba=".95 .51 .14 1"', 'rgba=".22 .68 .63 1"')
            assets["object.xml"] = object_xml.encode()
        self.model = mj.MjModel.from_xml_path(str(HERE / "scene.xml"), assets)
        self.round_object = round_object
        self.data = mj.MjData(self.model)
        self.sites = [self.model.site(f"{side}_sensor_frame").id for side in SIDES]
        self.skin_ids = {self.model.geom(side+"_skin").id: i for i, side in enumerate(SIDES)}
        self.object_id = self.model.body("test_object").id
        self.object_geom = self.model.geom("object_surface").id
        self.slide_q = [int(self.model.joint(f"jaw_{s}").qposadr[0]) for s in SIDES]
        self.arm_q = [int(self.model.joint(name).qposadr[0]) for name in ARM]
        self.pairs = [self.model.pair(f"{side}_tactile_pair").id for side in SIDES]
        self.initial_mass = float(self.model.body_mass[self.object_id])
        self.initial_inertia = self.model.body_inertia[self.object_id].copy()

    def reset(self, scenario="normal"):
        if self.round_object != (scenario == "round"):
            self._load_model(scenario == "round")
        mass, friction = SCENARIOS[scenario]
        self.scenario = scenario
        self.model.body_mass[self.object_id] = mass
        self.model.body_inertia[self.object_id] = self.initial_inertia * mass / self.initial_mass
        self.model.pair_friction[self.pairs, :2] = friction
        mj.mj_setConst(self.model, self.data)
        mj.mj_resetDataKeyframe(self.model, self.data, 0)
        mj.mj_forward(self.model, self.data)
        self.start_z = float(self.data.xpos[self.object_id, 2])
        self.phase = "Inspect"
        self.max_lift = 0.
        self.bilateral_ticks = 0
        self.motor_peak = np.zeros(5)
        self.taxels = np.zeros((2, len(POINTS), 3))
        self.patch_taxels = np.zeros_like(self.taxels)
        self.pad_torque = np.zeros((2, 3))
        self.last_wrench = self.taxels.copy()
        self.sample_window_s = 0.
        self.tactile_history = deque()

    def set_targets(self, joint_radians, gap_mm):
        """Set five simulated arm targets and desired distance between sensor crowns."""
        targets = np.asarray(joint_radians, dtype=float)
        if targets.shape != (5,) or not np.all(np.isfinite(targets)) or not 10.4 <= gap_mm <= 82.414:
            raise ValueError("Expected five finite joint angles and 10.4–82.414 mm jaw gap.")
        limits = self.model.actuator_ctrlrange[:5]
        if np.any(targets < limits[:,0]) or np.any(targets > limits[:,1]):
            raise ValueError("The joint targets exceed the simulated arm limits.")
        slide = min(.036, max(0., (82.4137-gap_mm)*.0005))
        self.data.ctrl[:] = [*targets,slide,slide]

    def read_tactile(self):
        """Return forces on the two pads in their local frames, shape (2, 77, 3)."""
        return self._sample_tactile()[0]

    def _surface_distance(self, points):
        """Distance from pad points to the scripted object's box or cylinder."""
        center = self.data.geom_xpos[self.object_geom]
        basis = self.data.geom_xmat[self.object_geom].reshape(3, 3)
        local = (points - center) @ basis
        size = self.model.geom_size[self.object_geom]
        if self.round_object:
            distances = np.column_stack((np.linalg.norm(local[:, :2], axis=1) - size[0],
                                         np.abs(local[:, 2]) - size[1]))
        else:
            distances = np.abs(local) - size
        return np.linalg.norm(np.maximum(distances, 0), axis=1) + np.minimum(distances.max(axis=1), 0)

    def _object_contact(self, geom1, geom2):
        return self.object_geom in (geom1, geom2)

    def _sample_tactile(self):
        readings = np.zeros((2, len(POINTS), 3))
        object_readings = np.zeros_like(readings)
        torque = np.zeros((2, 3))
        wrench = np.zeros(6)
        for i, contact in enumerate(self.data.contact):
            g1, g2 = int(contact.geom1), int(contact.geom2)
            if g1 in self.skin_ids:
                slot, sign = self.skin_ids[g1], -1
            elif g2 in self.skin_ids:
                slot, sign = self.skin_ids[g2], 1
            else:
                continue
            site = self.sites[slot]
            basis = self.data.site_xmat[site].reshape(3, 3)
            local_point = basis.T @ (contact.pos-self.data.site_xpos[site])
            mj.mj_contactForce(self.model, self.data, i, wrench)
            contact_to_pad = sign * basis.T @ contact.frame.reshape(3,3).T
            local_force = contact_to_pad @ wrench[:3]
            torque[slot] += contact_to_pad @ wrench[3:] + np.cross(local_point, local_force)
            # Distribute each contact over the three nearest measured positions.
            # The weights sum to one, preserving the resultant in the sensor axes.
            r2 = np.sum((TAXELS[:, :2]-local_point[:2])**2, axis=1)
            closest = np.argpartition(r2, 2)[:3]
            weights = 1. / (r2[closest]+.00065**2)
            distributed = (weights/weights.sum())[:,None] * local_force
            readings[slot,closest] += distributed
            if self._object_contact(g1, g2):
                object_readings[slot,closest] += distributed

        patch = readings.copy()
        for slot, site in enumerate(self.sites):
            resultant = object_readings[slot].sum(axis=0)
            if -resultant[2] <= 1e-8:
                continue
            basis = self.data.site_xmat[site].reshape(3, 3)
            points = TAXELS @ basis.T + self.data.site_xpos[site]
            weights = np.maximum(PATCH_ALLOWANCE_M - self._surface_distance(points), 0)
            if weights.sum() > 0:
                patch[slot] += weights[:, None] * (resultant / weights.sum()) - object_readings[slot]
        return readings, patch, torque

    @staticmethod
    def _center_xy_mm(vectors):
        pressure = np.maximum(vectors[:, 2], 0)
        total = pressure.sum()
        if total < CONTACT_NORMAL_N:
            return None
        return np.average(TAXELS[:, :2], axis=0, weights=pressure) * 1000

    def _tactile_features(self, vectors):
        features = []
        for slot, values in enumerate(vectors):
            resultant = values.sum(axis=0)
            center = self._center_xy_mm(values)
            shear = float(np.linalg.norm(resultant[:2]))
            normal = float(max(resultant[2], 0))
            contact = normal >= CONTACT_NORMAL_N
            old = self.tactile_history[0] if self.tactile_history else None
            dt = float(self.data.time - old[0]) if old is not None else 0.
            prior = old[1][slot] if dt >= HISTORY_S - .003 else None
            prior_center = self._center_xy_mm(prior) if prior is not None else None
            latest = self.last_wrench[slot] * COMPRESSION_POSITIVE
            latest_center = self._center_xy_mm(latest)
            features.append({
                "contact": contact,
                "active_points": int((values[:, 2] >= ACTIVE_POINT_N).sum()),
                "peak_normal_N": round(float(max(values[:, 2].max(), 0)), 5),
                "shear_N": round(shear, 5),
                "shear_normal_ratio": round(shear / normal, 5) if contact else None,
                "cop_xy_mm": center.round(3).tolist() if center is not None else None,
                "force_change_N": (latest.sum(axis=0) - prior.sum(axis=0)).round(5).tolist()
                if prior is not None else None,
                "cop_change_mm": round(float(np.linalg.norm(latest_center - prior_center)), 4)
                if latest_center is not None and prior_center is not None else None,
                "change_window_s": round(dt, 3) if prior is not None else None,
            })
        return features

    def step(self, count=1, scripted=False):
        for _ in range(count):
            if scripted:
                angles, slide, self.phase = trajectory(self.data.time)
                self.data.ctrl[:] = [*angles, slide, slide]
            mj.mj_step(self.model, self.data)
            self.motor_peak = np.maximum(self.motor_peak, np.abs(self.data.actuator_force[:5]))
            self.max_lift = max(self.max_lift, self.data.xpos[self.object_id,2]-self.start_z)
            readings, patch, torque = self._sample_tactile()
            if np.all(-readings.sum(axis=1)[:,2] > CONTACT_NORMAL_N):
                self.bilateral_ticks += 1
            self.last_wrench = readings
            self.taxels += readings
            self.patch_taxels += patch
            self.pad_torque += torque
            self.tactile_history.append((self.data.time, readings * COMPRESSION_POSITIVE))
            while len(self.tactile_history) > 1 and self.tactile_history[1][0] <= self.data.time - HISTORY_S:
                self.tactile_history.popleft()

    def advance(self, seconds, *, scripted=True):
        count = max(0, round(min(seconds, DURATION-self.data.time) / self.model.opt.timestep))
        self.taxels.fill(0)
        self.patch_taxels.fill(0)
        self.pad_torque.fill(0)
        self.step(count, scripted=scripted)
        self.sample_window_s = count * self.model.opt.timestep
        if count:
            self.taxels /= count
            self.patch_taxels /= count
            self.pad_torque /= count
        else:
            self.taxels, self.patch_taxels, self.pad_torque = self._sample_tactile()
            self.last_wrench = self.taxels.copy()

    def seek(self, seconds):
        """Replay the scripted motion from the initial pose to a timestamp."""
        self.reset(self.scenario)
        while self.data.time < seconds - .001:
            self.advance(min(.08, seconds - self.data.time))

    def _render(self, name, shape, mode="rgb"):
        width, height = shape
        renderer = self.renderers.get((width,height))
        if renderer is None:
            renderer = mj.Renderer(self.model, width=width, height=height)
            self.renderers[width,height] = renderer
        renderer.disable_depth_rendering()
        renderer.disable_segmentation_rendering()
        if mode == "depth":
            renderer.enable_depth_rendering()
        elif mode == "segmentation":
            renderer.enable_segmentation_rendering()
        renderer.update_scene(self.data, camera=name, scene_option=self.option)
        return renderer.render()

    def read_rgbd(self):
        """Return separate-camera RGB (uint8) and masked optical-axis Z (float32 metres)."""
        rgb = self._render("wrist_rgb", SETUP["rgb_resolution"])
        depth = self._render("wrist_depth", SETUP["depth_resolution"], "depth")
        valid = (depth >= SETUP["depth_min_m"]) & (depth <= SETUP["depth_max_m"])
        return rgb, np.where(valid, depth, np.nan)

    def side_rgb(self):
        """Render the left and right 26 cm RGB mounts."""
        return tuple(self._render(f"rgb_scene_{side}", SETUP["rgb_resolution"])
                     for side in SIDES)

    def observations(self):
        camera = self.model.camera("wrist_depth").id
        direction = -self.data.cam_xmat[camera].reshape(3,3)[:,2]
        target_depth = float((self.data.xpos[self.object_id]-self.data.cam_xpos[camera]) @ direction)
        # Pad-local tangential axes, compression positive as in the device readout.
        pad_vectors = self.taxels * COMPRESSION_POSITIVE
        patch_vectors = self.patch_taxels * COMPRESSION_POSITIVE
        tactile_features = self._tactile_features(pad_vectors)
        pad_normal = np.maximum(pad_vectors.sum(axis=1)[:, 2], 0)
        normal_sum = pad_normal.sum()
        return {
            "time_s": round(self.data.time, 3), "phase": self.phase, "scenario": self.scenario,
            "mass_g": SCENARIOS[self.scenario][0]*1000, "pad_friction": SCENARIOS[self.scenario][1],
            "gap_mm": round(float(82.4137-1000*self.data.qpos[self.slide_q].sum()), 2),
            "force_N": self.taxels.round(6).tolist(), "force_sum_N": self.taxels.sum(axis=1).round(4).tolist(),
            "paxini_force_N": pad_vectors.round(6).tolist(),
            "paxini_resultant_N": pad_vectors.sum(axis=1).round(6).tolist(),
            "paxini_torque_Nm": self.pad_torque.round(8).tolist(),
            "tactile_average_window_s": round(self.sample_window_s, 3),
            "estimated_contact_patch_N": patch_vectors.round(6).tolist(),
            "tactile_features": {
                "pads": tactile_features,
                "bilateral_contact": all(pad["contact"] for pad in tactile_features),
                "normal_balance": round(float((pad_normal[0] - pad_normal[1]) / normal_sum), 5)
                if normal_sum >= CONTACT_NORMAL_N else None,
            },
            "pad_axes_world": [self.data.site_xmat[site].reshape(3, 3).round(6).tolist() for site in self.sites],
            "depth_target_m": round(target_depth, 4),
            "lift_mm": round(max(0.,float(self.data.xpos[self.object_id,2]-self.start_z))*1000,2),
            "peak_lift_mm": round(self.max_lift*1000, 2),
            "motor_Nm": self.data.actuator_force[:5].round(4).tolist(),
            "joint_rad": self.data.qpos[self.arm_q].round(5).tolist()
        }

    def agent_tactile(self):
        state = self.observations()
        fields = ("time_s", "paxini_force_N", "paxini_resultant_N", "paxini_torque_Nm",
                  "tactile_average_window_s", "tactile_features")
        return {**{key: state[key] for key in fields},
                "source": "mujoco", "sides": list(SIDES), "axis_order": ["Fx", "Fy", "Fz"],
                "normal_positive": "compression", "taxel_indices": [point["index"] for point in POINTS]}

    def frames(self, include_cameras=True):
        state = self.observations()
        state["side_rgb_offset_mm"] = 260
        state["images"] = {"overview": jpeg(self._render("overview", (640,360))),
                           "grasp": jpeg(self._render("grasp_detail", (640,360)))}
        if not include_cameras:
            return state
        rgb, depth = self.read_rgbd()
        valid = np.isfinite(depth)
        fraction = np.clip((np.nan_to_num(depth, nan=.8)-.195)/(.62-.195), 0.,1.)
        anchors = np.array([[250,191,101],[225,91,122],[118,83,179],[38,102,158],[46,154,168]])
        colored = np.stack([np.interp(fraction, np.linspace(0,1,len(anchors)), anchors[:,i])
                            for i in range(3)],axis=-1).astype(np.uint8)
        colored[~valid] = [21, 25, 36]
        state["valid_depth_pct"] = round(float(valid.mean())*100,1)
        left_rgb, right_rgb = self.side_rgb()
        state["images"].update(rgb=jpeg(rgb), depth=jpeg(colored),
                               scene_left=jpeg(left_rgb), scene_right=jpeg(right_rgb))
        return state

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()


def verify(sim, render=True):
    reports = {}
    for scenario in SCENARIOS:
        sim.reset(scenario)
        max_force=np.zeros(2)
        milestone={}
        for t in (2.7, 6.2, 7.5, 8.9, 10.5, 11.7, DURATION):
            while sim.data.time < t-.001:
                sim.advance(min(.10,t-sim.data.time))
                max_force=np.maximum(max_force,-sim.taxels.sum(axis=1)[:,2])
            data=sim.observations()
            summary={k:data[k] for k in ("time_s","lift_mm","gap_mm","depth_target_m","force_sum_N")}
            if render and scenario=="normal" and t in (2.7,6.2,8.9):
                rgb,depth=sim.read_rgbd()
                seg=sim._render("wrist_depth",SETUP["depth_resolution"],"segmentation")
                mask=(seg[:,:,0]==sim.object_geom)&(seg[:,:,1]==mj.mjtObj.mjOBJ_GEOM)
                summary.update(rgb_shape=list(rgb.shape), depth_shape=list(depth.shape),
                               target_pixels=int(mask.sum()), valid_target_pixels=int((mask & np.isfinite(depth)).sum()))
                for side in SIDES:
                    side_seg=sim._render("rgb_scene_"+side,SETUP["rgb_resolution"],"segmentation")
                    summary[side+"_target_pixels"]=int(((side_seg[:,:,0]==sim.object_geom)&
                        (side_seg[:,:,1]==mj.mjtObj.mjOBJ_GEOM)).sum())
            milestone[str(t)]=summary
        reports[scenario]={"max_pad_normal_N":max_force.round(4).tolist(),
                           "bilateral_contact_s":round(sim.bilateral_ticks*sim.model.opt.timestep,3),
                           "peak_lift_mm":round(sim.max_lift*1000,2),
                           "motor_peak_Nm":sim.motor_peak.round(4).tolist(),"stages":milestone}
    assert min(reports["normal"]["max_pad_normal_N"]) > .1, reports
    assert reports["normal"]["peak_lift_mm"] >= 8.0, reports
    assert all(reports[s]["peak_lift_mm"] < 3.0 for s in ("slippery", "heavy", "heavy_matched")), reports
    if render:
        assert all(reports["normal"]["stages"]["2.7"][s+"_target_pixels"] > 100 for s in SIDES), reports
        assert reports["normal"]["stages"]["2.7"]["valid_target_pixels"] > 100, reports
        assert reports["normal"]["stages"]["6.2"]["valid_target_pixels"] == 0, reports
    return reports


def serve(sim, port):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            request = urlsplit(self.path)
            query = parse_qs(request.query)
            if request.path == "/":
                body, mime = (HERE/"index.html").read_bytes(), "text/html; charset=utf-8"
            elif request.path == "/taxels":
                body, mime = json.dumps(POINTS).encode(), "application/json"
            elif request.path == "/agent_tactile":
                body, mime = json.dumps(sim.agent_tactile()).encode(), "application/json"
            elif request.path == "/frame":
                try:
                    reset = query.get("reset",[None])[0]
                    seconds = float(query.get("dt",["0"])[0])
                    seek = query.get("at", [None])[0]
                    at = float(seek) if seek is not None else None
                    if ((reset is not None and reset not in SCENARIOS)
                            or (at is not None and not 0 <= at <= DURATION)
                            or not 0 <= seconds <= .25):
                        raise ValueError()
                    if reset is not None:
                        sim.reset(reset)
                    if at is None:
                        sim.advance(seconds)
                    else:
                        sim.seek(at)
                    result = sim.frames(include_cameras=query.get("cameras", ["1"])[0] != "0")
                    body, mime = json.dumps(result).encode(), "application/json"
                except (ValueError, KeyError):
                    self.send_error(400)
                    return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type",mime)
            self.send_header("Content-Length",str(len(body)))
            self.send_header("Cache-Control","no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    with HTTPServer(("127.0.0.1",port),Handler) as server:
        print(f"http://127.0.0.1:{server.server_port}/", flush=True)
        server.serve_forever()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",type=int,default=8768)
    parser.add_argument("--verify",action="store_true")
    parser.add_argument("--physics-only",action="store_true",help="Skip OpenGL in verification")
    args=parser.parse_args()
    sim=Sim()
    try:
        if args.verify:
            print(json.dumps(verify(sim,render=not args.physics_only),indent=2))
        else:
            serve(sim,args.port)
    finally:
        sim.close()


if __name__=="__main__":
    main()
