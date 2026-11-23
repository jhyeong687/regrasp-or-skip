"""A two-station workbench for an R7 robot and a hidden-ballast top layer."""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from latch.run import SEED, camera, chamfer_box, load_model
from sim import COMPRESSION_POSITIVE, POINTS, SETUP, Sim

PICK = (222, 36)
TOWER = (204, -101)
TOP_DIMS = np.array([.034, .018, .016])
LEDGE_TOP_MM = 38.4
HOME = {"xyz_mm": [246, 36, 90], "yaw_deg": 0, "pitch_deg": -27, "gap_mm": 76, "time_s": .5}
BALLAST_MM = (-13, 13)


def make_scene(robot_scene, ballast_mm):
    def customize(xml):
        world, asset = xml.find("worldbody"), xml.find("asset")
        xml.set("model", "R7 hidden-ballast stacking")
        for name, dims in (("base_layer", (.052, .038, .008, .007)),
                           ("middle_layer", (.035, .026, .013, .004)),
                           ("top_shell", (*TOP_DIMS, .0035))):
            chamfer_box(asset, name, *dims)
        part = ET.SubElement(world, "body", name="top_layer", pos=".222 .036 .00887")
        ET.SubElement(part, "freejoint", name="top_free")
        ET.SubElement(part, "inertial", pos=f"{ballast_mm/1000} 0 0", mass=".085",
                      diaginertia=".000017 .000045 .000049")
        ET.SubElement(part, "geom", name="object_surface", type="mesh", mesh="top_shell",
                      material="handle_finish", contype="2", conaffinity="21", condim="4",
                      friction=".83 .005 .0001")
        ET.SubElement(part, "geom", type="box", pos="0 0 .01612", size=".020 .006 .00012",
                      material="vise", contype="0", conaffinity="0", mass="0")
        ET.SubElement(part, "geom", type="box", pos="0 0 .01630", size=".0006 .005 .00010",
                      material="handle_mark", contype="0", conaffinity="0", mass="0")
        camera(world, "stack_full", (.627, -.641, .409), (.122, -.014, .086), 34)
        camera(world, "stack_detail", (.375, -.315, .197), (.208, -.074, .036), 43)
        camera(world, "stack_top", (.208, -.115, .30), (.212, -.033, .01), 40)
    return load_model(robot_scene, HERE / "bench.xml", customize)


class StackBench(Sim):
    def __init__(self, scene, ballast_mm):
        self.model = make_scene(scene, ballast_mm)
        self.data, self.fit = mj.MjData(self.model), mj.MjData(self.model)
        self.sites = [self.model.site(f"{side}_sensor_frame").id for side in ("left", "right")]
        self.skin_ids = {self.model.geom(f"{side}_skin").id: i for i, side in enumerate(("left", "right"))}
        self.renderers = {}
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.object_geom = self.model.geom("object_surface").id
        self.ledge_geom = self.model.geom("ledge").id
        self.table = self.model.geom("table").id
        self.object_id = self.model.body("top_layer").id
        self.object_dof = int(self.model.joint("top_free").dofadr[0])
        self.reset()

    def reset(self):
        mj.mj_resetData(self.model, self.data)
        home = self.pose(HOME, SEED)
        self.data.qpos[:5] = home
        self.command = np.r_[home, [self.slide(HOME["gap_mm"])] * 2]
        self.data.qpos[5:7] = self.command[5:7]
        self.data.ctrl[:] = self.command
        self.trace = []
        self.frame_sink = None
        mj.mj_forward(self.model, self.data)

    def hand(self, angles):
        self.fit.qpos[:5] = angles
        mj.mj_kinematics(self.model, self.fit)
        xyz = self.fit.site_xpos[self.sites].mean(axis=0)
        # The wrist pitch and jaw heading are the two orientation coordinates.
        left = self.fit.site_xmat[self.sites[0]].reshape(3, 3)
        up, across = left[:, 1], left[:, 2]
        return np.r_[xyz, np.arctan2(up[0], up[2]), np.arctan2(-across[0], across[1])]

    def pose(self, waypoint, seed):
        target = np.r_[np.asarray(waypoint["xyz_mm"]) / 1000,
                       np.radians([waypoint["pitch_deg"], waypoint["yaw_deg"]])]
        weights = np.array([12., 12., 12., 1., 1.])
        limits = self.model.actuator_ctrlrange[:5]
        angles = seed.copy()
        delta = np.eye(5) * 1e-5
        for _ in range(24):
            current = self.hand(angles)
            error = (target - current) * weights
            if np.linalg.norm(error) < .0008:
                break
            jac = (np.array([self.hand(angles + d) for d in delta]) - current).T / 1e-5
            jac = jac * weights[:, None]
            step = np.linalg.solve(jac.T @ jac + .0002 * np.eye(5), jac.T @ error)
            angles = np.clip(angles + np.clip(step, -.16, .16), limits[:, 0], limits[:, 1])
        actual = self.hand(angles)
        if np.linalg.norm(actual[:3] - target[:3]) > .0015 or np.linalg.norm(actual[3:] - target[3:]) > .065:
            raise ValueError(f"Unreachable {waypoint['xyz_mm']} at pitch/yaw {waypoint['pitch_deg']}/{waypoint['yaw_deg']}; "
                             f"closest pose {self.describe(angles)}")
        return angles

    def describe(self, joints):
        hand = self.hand(joints)
        return {"xyz_mm": (hand[:3]*1000).round(2).tolist(),
                "pitch_deg": round(float(np.degrees(hand[3])), 2),
                "yaw_deg": round(float(np.degrees(hand[4])), 2)}

    @staticmethod
    def slide(gap_mm):
        return float(np.clip((82.4137 - gap_mm)/2000, 0, .036))

    def preview(self, waypoints):
        if not 1 <= len(waypoints) <= 10:
            raise ValueError("Supply from one to ten waypoints")
        seed = self.command[:5]
        path = []
        for point in waypoints:
            xyz = np.asarray(point["xyz_mm"])
            if (xyz.shape != (3,) or not np.all(np.isfinite(xyz))
                    or np.any(xyz < [150, -127, 5]) or np.any(xyz > [273, 68, 116])
                    or not -53 <= point["pitch_deg"] <= 8 or not -33 <= point["yaw_deg"] <= 33
                    or not 14 <= point["gap_mm"] <= 82 or not .28 <= point["time_s"] <= 2.0):
                raise ValueError(f"Out-of-envelope waypoint {point}")
            angles = self.pose(point, seed)
            if max(abs(angles-seed)) / point["time_s"] > 2.8:
                raise ValueError("Waypoint would move a joint too fast")
            path.append((point, np.r_[angles, [self.slide(point["gap_mm"])] * 2]))
            seed = angles
        return path

    def run(self, waypoints, phase):
        for point, target in self.preview(waypoints):
            start = self.command.copy()
            duration = point["time_s"]
            steps = round(duration / self.model.opt.timestep)
            for i in range(steps):
                u = (i+1)/steps
                self.data.ctrl[:] = start+(target-start)*u**3*(10-15*u+6*u*u)
                self.data.ctrl[:5] += self.data.qfrc_bias[:5]/170
                mj.mj_step(self.model, self.data)
                tick = round(self.data.time * 1000)
                if tick % 40 == 0:
                    self.trace.append(self.sample(phase))
                if self.frame_sink is not None and tick % 80 == 0:
                    self.frame_sink(self.frame(), self.sample(phase))
            self.command = target

    def _surface_distance(self, points):
        local = (points - self.data.xpos[self.object_id]) @ self.data.xmat[self.object_id].reshape(3, 3)
        distance = np.abs(local) - TOP_DIMS
        return np.linalg.norm(np.maximum(distance, 0), axis=1) + np.minimum(distance.max(axis=1), 0)

    def on(self, geom):
        pair = {self.object_geom, geom}
        return any({c.geom1, c.geom2} == pair and c.efc_address >= 0 for c in self.data.contact)

    def sample(self, phase):
        raw, patch, torque = self._sample_tactile()
        net = raw.sum(axis=1)
        axes = self.data.site_xmat[self.sites].reshape(2, 3, 3)
        reference = self.data.site_xpos[self.sites].mean(axis=0)
        world_f = np.einsum("pij,pj->pi", axes, net)
        world_moments = np.einsum("pij,pj->pi", axes, torque)
        world_moments += np.cross(self.data.site_xpos[self.sites] - reference, world_f)
        return {
            "t": round(float(self.data.time), 3), "phase": phase,
            "tcp": self.describe(self.data.qpos[:5]),
            "gap_mm": round(float(82.4137 - 1000*self.data.qpos[5:7].sum()), 3),
            "arm_torque_Nm": self.data.actuator_force[:5].round(5).tolist(),
            "pad_force_N": (raw*COMPRESSION_POSITIVE).round(4).tolist(),
            "pad_patch_N": (patch*COMPRESSION_POSITIVE).round(4).tolist(),
            "pad_resultant_N": (net*COMPRESSION_POSITIVE).round(4).tolist(),
            "pad_torque_Nm": torque.round(6).tolist(),
            "pad_load_world_N": world_f.sum(axis=0).round(5).tolist(),
            "pad_moment_about_tcp_mNm": (world_moments.sum(axis=0)*1000).round(4).tolist(),
            "top_xyz_mm_eval": (self.data.xpos[self.object_id]*1000).round(4).tolist(),
            "table_eval": self.on(self.table),
        }

    def verdict(self):
        rotation = self.data.xmat[self.object_id].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1))))
        velocity = float(np.linalg.norm(self.data.qvel[self.object_dof:self.object_dof+3]))
        xyz = self.data.xpos[self.object_id]*1000
        pad_force = self.read_tactile().sum(axis=1)
        released = bool(np.all(np.abs(pad_force[:, 2]) < .05))
        return {"stable": bool(self.on(self.ledge_geom) and tilt < 5 and velocity < .03 and released),
                "ledge_contact": self.on(self.ledge_geom), "released": released,
                "tilt_deg": round(tilt, 3), "speed_m_s": round(velocity, 5), "xyz_mm": xyz.round(3).tolist()}

    def settle(self, seconds=2.0):
        for _ in range(round(seconds / self.model.opt.timestep)):
            self.data.ctrl[:5] = self.command[:5] + self.data.qfrc_bias[:5]/170
            mj.mj_step(self.model, self.data)
            tick = round(self.data.time*1000)
            if tick % 40 == 0:
                sample = self.sample("settle")
                self.trace.append(sample)
                if self.frame_sink is not None and tick % 80 == 0:
                    self.frame_sink(self.frame(), sample)

    def cameras(self):
        feeds = [self._render(name, (320, 180)) for name in ("rgb_scene_left", "rgb_scene_right", "wrist_rgb")]
        depth = self._render("wrist_depth", (320, 180), "depth")
        valid = (depth >= SETUP["depth_min_m"]) & (depth <= SETUP["depth_max_m"])
        shade = np.clip(np.where(valid, depth - .195, .45)/.45, 0, 1)[..., None]
        color = (np.array([244,194,121])*(1-shade) + np.array([38,57,102])*shade).astype("uint8")
        color[~valid] = [19,24,32]
        return np.column_stack((*feeds, color))

    def frame(self):
        return np.column_stack((self._render("stack_full", (850, 480)),
                                self._render("stack_detail", (430, 480)))), self.cameras()


def waypoint(x, y, z, pitch, gap, duration=.64, yaw=0):
    return {"xyz_mm": [x, y, z], "pitch_deg": pitch, "yaw_deg": yaw, "gap_mm": gap, "time_s": duration}
