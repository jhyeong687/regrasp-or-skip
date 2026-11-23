"""Run the SO-ARM101 latch bench and optionally render synchronized replays."""

import argparse
import csv
import gzip
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
SIMULATION = HERE.parent
sys.path.insert(0, str(SIMULATION))
from sim import COMPRESSION_POSITIVE, KEYS, POINTS, SETUP, Sim

CASE_ORDER = [(state, motion) for motion in ("wiggle", "press", "wait")
              for state in ("loose", "engaged")]
SEED = KEYS[3][1].copy()
SEED[3] += np.radians(3)


def camera(world, name, position, target, fovy):
    back = np.asarray(position) - target
    back /= np.linalg.norm(back)
    right = np.cross([0, 0, 1], back)
    right /= np.linalg.norm(right)
    up = np.cross(back, right)
    ET.SubElement(world, "camera", name=name, pos=" ".join(map(str, position)),
                  xyaxes=" ".join(map(str, np.r_[right, up])), fovy=str(fovy))


def save_wood(path):
    rng = np.random.default_rng(101)
    height, width = 384, 768
    y, x = np.mgrid[0:height, 0:width]
    warp = y + 2 * np.sin(x / 97 + y / 45) + .8 * np.sin(x / 18 + y / 13)
    grain = 3.5 * np.sin(warp * 1.1) + 4.5 * np.sin(warp * .17)
    grain += rng.normal(0, 1, (height, width))
    for seam in (85, 178, 276):
        grain += np.where(y > seam, 2, 0) - 7 * np.exp(-((y - seam) / .95)**2)
    rgb = np.array([171, 132, 98]) + grain[:, :, None] * [.95, .8, .6]
    Image.fromarray(rgb.clip(0, 255).astype("uint8")).save(path)


def chamfer_box(asset, name, x, y, z, bevel):
    """Create decorative flat-faced panels with their corners trimmed in XY."""
    outline = ((x - bevel, y), (x, y - bevel), (x, -y + bevel), (x - bevel, -y),
               (-x + bevel, -y), (-x, -y + bevel), (-x, y - bevel), (-x + bevel, y))
    vertices = " ".join(f"{px} {py} {height}" for height in (-z, z) for px, py in outline)
    ET.SubElement(asset, "mesh", name=name, vertex=vertices)


def load_model(robot_scene):
    xml = ET.parse(robot_scene).getroot()
    xml.set("model", "SO-ARM101 latch workbench")
    xml.find("compiler").set("meshdir", str(robot_scene.parent / "meshes"))
    xml.find("option").set("timestep", ".001")
    xml.find("option").set("iterations", "65")
    xml.remove(xml.find("keyframe"))
    world = xml.find("worldbody")
    world.remove(world.find("include"))
    world.find("geom[@name='table']").set("rgba", "0 0 0 0")
    for item in ET.parse(HERE / "workbench.xml").getroot():
        world.append(item)
    for item in ET.parse(HERE / "fixture.xml").getroot():
        world.append(item)
    default = ET.SubElement(ET.SubElement(xml, "default"), "default", {"class": "mechanism"})
    ET.SubElement(default, "geom", contype="16", conaffinity="16", condim="3",
                  friction=".045 .001 .0001", solref=".004 1", solimp=".95 .99 .0002")
    visual = xml.find("visual")
    visual.find("global").attrib.update(offwidth="1280", offheight="720")
    ET.SubElement(visual, "quality", shadowsize="4096", offsamples="4")
    visual.find("headlight").attrib.update(ambient=".34 .34 .36", diffuse=".43 .43 .44",
                                          specular=".12 .12 .14")
    key = world.find("light[@name='key']")
    key.attrib.update(directional="false", cutoff="78", diffuse=".66 .61 .54",
                      specular=".24 .23 .21", attenuation="1 .4 .04")
    world.find("light[@name='fill']").set("diffuse", ".42 .47 .59")
    camera(world, "bench_view", (.66, -.62, .395), (.087, -.010, .091), 33)
    camera(world, "cutaway", (.255, .090, .073), (.202, .015, .037), 49)
    asset = xml.find("asset")
    colors = {
        "housing": (".21 .33 .36 1", ".35"), "vise": (".11 .16 .19 1", ".40"),
        "silver": (".52 .58 .64 1", ".55"), "mat": (".21 .25 .26 1", ".06"),
        "wall": (".70 .73 .71 1", ".03"), "pegboard": (".56 .58 .55 1", ".06"),
        "paper": (".77 .76 .67 1", ".02"), "notebook": (".24 .31 .38 1", ".13"),
        "tray_finish": (".27 .31 .32 1", ".27"), "cup": (".73 .76 .74 1", ".33"),
        "coffee": (".17 .094 .058 1", ".22"), "cover_finish": (".24 .46 .48 1", ".32"),
        "handle_finish": (".90 .48 .16 1", ".30"), "handle_mark": (".96 .76 .38 1", ".06"),
        "brass": (".69 .40 .17 1", ".37"), "bolt_finish": (".27 .66 .70 1", ".40"),
    }
    for name, (rgba, specular) in colors.items():
        ET.SubElement(asset, "material", name=name, rgba=rgba, specular=specular, shininess=".27")
    for name, dimensions in {
        "fixture_base": (.067, .046, .005, .006),
        "fixture_body": (.046, .034, .012, .004),
        "lid_finish": (.043, .030, .0023, .003),
        "grip_finish": (.013, .016, .017, .0018),
    }.items():
        chamfer_box(asset, name, *dimensions)
    # Brass wedge rises into the spring bolt. Its flat upper face retains the bolt.
    wedge = " ".join(f"{x} {y} {z}" for x in (-.010, .010)
                     for y, z in ((.009, -.009), (.011, -.009), (.021, -.003),
                                  (.021, -.0016), (.009, -.0016)))
    ET.SubElement(asset, "mesh", name="latch_ramp", vertex=wedge)
    for side, sign in (("left", -1), ("right", 1)):
        funnel = " ".join(f"{sign*x} {y} {z}" for y in (-.021, -.013)
                          for x, z in ((.005, -.011), (.009, -.011), (.009, -.023),
                                       (.00145, -.023), (.00145, -.0165)))
        ET.SubElement(asset, "mesh", name=f"guide_{side}", vertex=funnel)
    ET.SubElement(asset, "texture", name="wood", type="2d", file="grain.png")
    ET.SubElement(asset, "material", name="oak", texture="wood", specular=".16", shininess=".22")
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "grain.png"
        save_wood(path)
        return mj.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"), {"grain.png": path.read_bytes()})


class Bench(Sim):
    """Use the shared pad contact sampler and robot renderer with a separate fixture scene."""

    def __init__(self, robot_scene):
        self.model = load_model(robot_scene)
        self.data = mj.MjData(self.model)
        self.fit = mj.MjData(self.model)
        self.renderers = {}
        self.option = mj.MjvOption()
        self.option.geomgroup[3:5] = 0
        self.option.sitegroup[:] = 0
        self.sites = [self.model.site(f"{side}_sensor_frame").id for side in ("left", "right")]
        self.skin_ids = {self.model.geom(f"{side}_skin").id: i for i, side in enumerate(("left", "right"))}
        self.object_geom = self.model.geom("object_surface").id
        self.round_object = False
        self.fixture_q = [int(self.model.joint(name).qposadr[0])
                          for name in ("cover_side", "cover_down", "bolt_travel")]
        self.bolt_dof = int(self.model.joint("bolt_travel").dofadr[0])
        self.retention_geoms = {self.model.geom(name).id for name in ("bolt_tip", "ramp")}
        self.fixture = self.model.body("fixture").pos.copy()
        self.home = self.pose(0, .017)
        self.grip = self.pose(0, 0)

    def hand(self, angles):
        self.fit.qpos[:5] = angles
        mj.mj_kinematics(self.model, self.fit)
        x, z = self.fit.site_xpos[self.sites].mean(axis=0)[[0, 2]]
        up = self.fit.site_xmat[self.sites[0]].reshape(3, 3)[:, 1]
        return np.array([x, z, np.arctan2(up[0], up[2])])

    def pose(self, dx, dz):
        target = [self.fixture[0] + dx, self.fixture[2] + .026 + dz, 0.]
        angles = SEED.copy()
        delta = np.eye(5)[1:4] * 1e-5
        for _ in range(8):
            current = self.hand(angles)
            jacobian = (np.array([self.hand(angles + step) for step in delta]) - current).T / 1e-5
            angles[1:4] += np.linalg.solve(jacobian, target - current)
        if np.linalg.norm(self.hand(angles) - target) > 1e-6:
            raise RuntimeError("Fixture pose lies outside the tested IK branch")
        return angles

    def engaged(self):
        _, height, bolt = self.data.qpos[self.fixture_q]
        return bool(height < -.0052 and bolt < .0035)

    def run_trial(self, state, motion, offset=0., stiffness_scale=1., friction=.8,
                  frames=None, bolt_retracted=False, include_vectors=False, frame_ms=50):
        data, model = self.data, self.model
        mj.mj_resetData(model, data)
        model.jnt_stiffness[model.joint("bolt_travel").id] = 60 * stiffness_scale
        for side in ("left", "right"):
            model.pair_friction[model.pair(f"{side}_tactile_pair").id, :2] = friction
        data.qpos[:5] = self.home
        if state == "engaged":
            data.qpos[self.fixture_q[1]] = -.0076
        else:
            data.qpos[self.fixture_q[0]] = offset
        if bolt_retracted:
            data.qpos[self.fixture_q[2]] = .015
            data.qfrc_applied[self.bolt_dof] = 1.3
        mj.mj_forward(model, data)
        close = .0305
        hold = self.pose(offset, -.0015)
        plans = {
            "wiggle": [(.55, self.pose(offset + .0015, -.0015), close, "wiggle_out"),
                       (.65, self.pose(offset - .0015, -.0015), close, "wiggle_back"),
                       (.40, hold, close, "center")],
            "press": [(.85, self.pose(offset + .003, -.012), close, "press"),
                      (.75, hold, close, "unload")],
            "wait": [(1.60, self.home, 0., "wait")],
        }
        setup = [] if motion == "wait" else [(.65, hold, 0., "approach"), (.65, hold, close, "grip")]
        finish = [(1.15, self.home, 0., "hold_clear")] if motion == "wait" else [
            (.45, hold, 0., "release"), (.70, self.home, 0., "retract")]
        schedule = [(.40, self.home, 0., "settle"), *setup, *plans[motion], *finish, (.55, self.home, 0., "hold_clear")]
        samples, transitions = [], []
        previous = np.r_[self.home, 0., 0.]
        initial = last = None
        armed = False
        for duration, angle, slide, phase in schedule:
            if phase != "settle" and not armed:
                initial = last = self.engaged()
                if initial != (state == "engaged"):
                    raise RuntimeError(f"Fixture failed to reset to {state}: {data.qpos[self.fixture_q]}")
                armed = True
            target = np.r_[angle, slide, slide]
            steps = round(duration / model.opt.timestep)
            for index in range(steps):
                progress = (index + 1) / steps
                blend = progress**3 * (10 + progress * (-15 + 6 * progress))
                data.ctrl[:] = previous + blend * (target - previous)
                data.ctrl[:5] += data.qfrc_bias[:5] / 170
                mj.mj_step(model, data)
                current = self.engaged()
                if armed and current != last:
                    transitions.append({"s": round(float(data.time), 3), "phase": phase, "engaged": current})
                    last = current
                if round(data.time * 1000) % 20 == 0:
                    samples.append(self.sample(phase, include_vectors))
                if frames is not None and round(data.time * 1000) % frame_ms == 0:
                    frames(self.frame(), phase)
            previous = target
        # Compare the same forward half of the wiggle in every run; no joint truth in classifier.
        window = [sample for sample in samples if sample["phase"] == "wiggle_out"]
        metric = float(max((abs(sample["shear_x_N"]) for sample in window), default=0.))
        return {"state": state, "motion": motion, "initial_engaged": initial,
                "final_engaged": self.engaged(), "transitions": transitions, "peak_wiggle_x_N": metric,
                "final_cover_z_mm": samples[-1]["cover_z_mm"],
                "final_bolt_mm": samples[-1]["bolt_mm"],
                "final_retaining_contact": self.engaged() and any(
                    {int(contact.geom1), int(contact.geom2)} == self.retention_geoms
                    and contact.efc_address >= 0 for contact in data.contact),
                "peak_pinch_N": max(sample["pinch_N"] for sample in samples),
                "peak_press_z_N": max(abs(sample["shear_z_N"]) for sample in samples)}, samples

    def sample(self, phase, include_vectors):
        mj.mj_forward(self.model, self.data)
        vectors, patch, torque = self._sample_tactile()
        vectors *= COMPRESSION_POSITIVE
        resultants = vectors.sum(axis=1)
        force_world = sum(self.data.site_xmat[s].reshape(3, 3) @ (f * COMPRESSION_POSITIVE)
                          for s, f in zip(self.sites, resultants))
        x, z, bolt = self.data.qpos[self.fixture_q] * 1000
        frame = {"t": round(float(self.data.time), 3), "phase": phase,
                "engaged": self.engaged(), "cover_x_mm": round(float(x), 4),
                "cover_z_mm": round(float(z), 4), "bolt_mm": round(float(bolt), 4),
                "pinch_N": round(float(resultants[:, 2].mean()), 4),
                "gap_mm": round(float(82.4137 - 1000 * self.data.qpos[5:7].sum()), 3),
                "elbow_Nm": round(float(abs(self.data.actuator_force[2])), 4),
                "shear_x_N": round(float(force_world[0]), 4),
                "shear_z_N": round(float(force_world[2]), 4)}
        if include_vectors:
            frame["pad_force_N"] = vectors.round(4).tolist()
            frame["pad_patch_N"] = (patch * COMPRESSION_POSITIVE).round(4).tolist()
            frame["pad_resultant_N"] = resultants.round(4).tolist()
            frame["pad_torque_Nm"] = torque.round(6).tolist()
        return frame

    def frame(self):
        self.option.geomgroup[2] = self.option.geomgroup[5] = 1
        wide = self._render("bench_view", (860, 500))
        self.option.geomgroup[2] = self.option.geomgroup[5] = 0
        detail = self._render("cutaway", (420, 500))
        self.option.geomgroup[2] = self.option.geomgroup[5] = 1
        feeds = [self._render(name, (320, 180))
                 for name in ("rgb_scene_left", "rgb_scene_right", "wrist_rgb")]
        depth = self._render("wrist_depth", (320, 180), "depth")
        valid = (depth >= SETUP["depth_min_m"]) & (depth <= SETUP["depth_max_m"])
        shade = np.clip(np.where(valid, depth - .195, .45) / .45, 0., 1.)[..., None]
        depth_rgb = (np.array([244, 194, 121]) * (1 - shade)
                     + np.array([38, 57, 102]) * shade).astype(np.uint8)
        depth_rgb[~valid] = [19, 24, 32]
        return np.column_stack((wide, detail)), np.column_stack((*feeds, depth_rgb))


def render_trial(bench, state, motion, output):
    prefix = output / f"{state}-{motion}"
    command = ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
               "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "1280x500",
               "-framerate", "20", "-i", "-", "-c:v", "libx264", "-crf", "19",
               "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(prefix.with_suffix(".mp4"))]
    camera_command = command.copy()
    camera_command[camera_command.index("1280x500")] = "1280x180"
    camera_command[-1] = str(prefix.with_name(prefix.name + "-cameras").with_suffix(".mp4"))
    with (subprocess.Popen(command, stdin=subprocess.PIPE) as encoder,
          subprocess.Popen(camera_command, stdin=subprocess.PIPE) as cameras):
        def write_frames(frames, _phase):
            for pipe, frame in zip((encoder.stdin, cameras.stdin), frames):
                pipe.write(frame.tobytes())

        result = bench.run_trial(state, motion, frames=write_frames, include_vectors=True)
        for process in (encoder, cameras):
            process.stdin.close()
            if process.wait() != 0:
                raise RuntimeError("Could not encode the rendered trial")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-scene", type=Path, default=SIMULATION / "scene.xml",
                        help="Use an existing R7 scene.xml with its adjacent meshes directory")
    parser.add_argument("--render", action="store_true", help="Generate four MuJoCo camera replays using FFmpeg")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    bench = Bench(args.robot_scene.resolve())
    summary = {"mujoco_version": mj.__version__, "nominal": {}, "validation": []}
    replays = {}
    try:
        for state, motion in CASE_ORDER:
            key = f"{state}-{motion}"
            if args.render and motion != "wait":
                result, samples = render_trial(bench, state, motion, args.output)
            else:
                result, samples = bench.run_trial(state, motion, include_vectors=True)
            print(key, result, flush=True)
            summary["nominal"][key] = result
            replays[key] = samples
        loose = summary["nominal"]["loose-wiggle"]["peak_wiggle_x_N"]
        engaged = summary["nominal"]["engaged-wiggle"]["peak_wiggle_x_N"]
        threshold = (loose + engaged) / 2
        summary["nominal_midpoint_threshold_N"] = threshold
        for offset, bolt_k, grip in ((-.00025, .90, .70), (-.00013, 1.10, .88),
                                     (0., .96, .76), (.00012, 1.06, .85), (.00024, 1.14, .73)):
            for state, motion in CASE_ORDER:
                result, _ = bench.run_trial(state, motion, offset, bolt_k, grip)
                result.update(pose_offset_mm=offset * 1000, bolt_stiffness_scale=bolt_k, grip_friction=grip)
                if motion == "wiggle":
                    result["predicted_engaged"] = result["peak_wiggle_x_N"] > threshold
                summary["validation"].append(result)
        control, traces = bench.run_trial("loose", "press", bolt_retracted=True, include_vectors=True)
        summary["bolt_retracted_control"] = control
        replays["loose-press-bolt-retracted"] = traces
        (args.output / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
        replay_json = json.dumps({"points": POINTS, "trials": replays}, separators=(",", ":"))
        if args.render:
            (args.output / "replays.json").write_text(replay_json + "\n")
        (args.output / "replays.json.gz").write_bytes(gzip.compress(replay_json.encode(), mtime=0))
        if args.render:
            (args.output / "index.html").write_bytes((HERE / "index.html").read_bytes())
        with (args.output / "nominal-traces.csv").open("w", newline="") as handle:
            columns = [key for key in next(iter(replays.values()))[0] if not key.startswith("pad_")]
            writer = csv.DictWriter(handle, fieldnames=["trial", *columns],
                                    extrasaction="ignore", lineterminator="\n")
            writer.writeheader()
            for key, samples in replays.items():
                writer.writerows({"trial": key, **sample} for sample in samples)
    finally:
        bench.close()


if __name__ == "__main__":
    main()
