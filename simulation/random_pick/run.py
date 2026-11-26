"""Run the randomized SO-ARM101 RGB-D pick-and-place pilot with live reporting."""

import argparse
from dataclasses import asdict, dataclass
import csv
import hashlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE.parent / "stacking")]
from agent_depth import depth_calibration
from astra_runtime import EFFORT, MODEL, infer
from bench import StackBench, waypoint
from latch.run import camera, load_model
from sim import KEYS, SETUP, Sim

SEED = 261126
TRAY = (204, -101)
TRAY_HALF_MM = np.array([45, 40])
FLOOR_MM = -2
SHAPES = {
    "brick": ("box", (.020, .015, .017)),
    "cylinder": ("cylinder", (.017, .017)),
    "bar": ("box", (.030, .013, .017)),
}
SCHEMA = {
    "type": "object",
    "properties": {
        "grasp": {"type": "string", "enum": ["A", "B", "C", "skip"]},
        "evidence": {"type": "string"},
    },
    "required": ["grasp", "evidence"],
    "additionalProperties": False,
}
PROMPT = """Pick one grasp from the supplied candidates for an SO-ARM101 parallel jaw gripper.
Use the three RGB views and calibrated wrist-depth heatmap together. The pick target is the amber object.
A/B/C mark alternative locations or jaw headings for the same object. The pads close along the
jaw heading (yaw 0 means the world Y axis). Prefer opposing surfaces and contact near the center;
for a round part prefer a centered grasp. Coordinates are in mm and yaw is in degrees.
The three candidates are IK-feasible; feasibility alone does not imply good contact.
The depth panel is from a separate optical camera, so do not match its pixels directly to RGB.
Return one ID and a brief concrete observation that justifies it. Use skip only if none is justified.
The accepted grasp is executed immediately, followed by a lift and a prevalidated route to the tray."""


@dataclass(frozen=True)
class Case:
    trial: int
    shape: str
    x_mm: float
    y_mm: float
    yaw_deg: float


def cases(seed):
    rng = np.random.default_rng(seed)
    order = rng.permutation(np.repeat(list(SHAPES), 4))
    return [Case(i, str(shape), *np.round([rng.uniform(192, 232), rng.uniform(-18, 34),
                                         rng.uniform(-22, 22)], 2).tolist())
            for i, shape in enumerate(order, 1)]


def build_scene(scene, shape, visual_fixture=None, object_fixture=None):
    kind, size = SHAPES[shape]

    def customize(xml):
        world = xml.find("worldbody")
        obj = ET.SubElement(world, "body", name="pick_object", pos=".212 0 .01")
        ET.SubElement(obj, "freejoint", name="pick_free")
        surface = ET.SubElement(obj, "geom", name="object_surface", type=kind,
                      size=" ".join(map(str, size)), rgba=".98 .44 .07 1", mass=".05",
                      contype="2", conaffinity="21", condim="4", friction=".83 .005 .0001")
        if object_fixture is not None:
            object_fixture(xml.find("asset"), surface)
        x, y = np.array(TRAY) / 1000
        half_x, half_y = TRAY_HALF_MM / 1000
        ET.SubElement(world, "geom", name="target_tray", type="box", pos=f"{x} {y} -.0045",
                      size=f"{half_x} {half_y} .0025", rgba=".16 .45 .43 1",
                      contype="16", conaffinity="2", friction=".85 .005 .0001")
        for dx, dy, sx, sy in ((half_x, 0, .002, half_y), (-half_x, 0, .002, half_y),
                               (0, half_y, half_x, .002), (0, -half_y, half_x, .002)):
            ET.SubElement(world, "geom", type="box", pos=f"{x + dx} {y + dy} .001",
                          size=f"{sx} {sy} .003", rgba=".22 .62 .57 1",
                          contype="16", conaffinity="2")
        camera(world, "pick_live", (.47, -.49, .33), (.16, -.032, .075), 41)
        if visual_fixture is not None:
            visual_fixture(world)

    return load_model(scene, HERE / "empty.xml", customize)


class PickBench(StackBench):
    def __init__(self, scene, shape, *, visual_fixture=None, object_fixture=None):
        self.model = build_scene(scene, shape, visual_fixture, object_fixture)
        self.data, self.fit = mj.MjData(self.model), mj.MjData(self.model)
        self.sites = [self.model.site(f"{side}_sensor_frame").id for side in ("left", "right")]
        self.skin_ids = {self.model.geom(f"{side}_skin").id: i for i, side in enumerate(("left", "right"))}
        self.renderers = {}
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.object_geom = self.model.geom("object_surface").id
        self.object_id = self.model.body("pick_object").id
        self.tray_geom = self.model.geom("target_tray").id
        self.table = self.model.geom("table").id
        joint = self.model.joint("pick_free")
        self.object_qpos, self.object_dof = int(joint.qposadr[0]), int(joint.dofadr[0])
        self.round_object = SHAPES[shape][0] == "cylinder"

    _surface_distance = Sim._surface_distance

    def reset(self, case):
        mj.mj_resetData(self.model, self.data)
        self.command = np.r_[KEYS[2][1], 0., 0.]
        self.data.qpos[:7] = self.command
        self.data.ctrl[:] = self.command
        angle = np.radians(case.yaw_deg / 2)
        self.data.qpos[self.object_qpos:self.object_qpos + 7] = [
            case.x_mm / 1000, case.y_mm / 1000, .010, np.cos(angle), 0, 0, np.sin(angle)]
        self.trace = []
        self.frame_sink = None
        mj.mj_forward(self.model, self.data)
        self.settle(.25)

    def frame(self):
        return self._render("pick_live", (720, 406))

    def sample(self, phase):
        return {
            "sim_s": round(float(self.data.time), 3),
            "phase": phase,
            "tcp_mm": self.describe(self.data.qpos[:5])["xyz_mm"],
            "pad_normal_n": (-self.read_tactile().sum(axis=1)[:, 2]).round(4).tolist(),
            "object_xyz_mm_eval": (self.data.xpos[self.object_id] * 1000).round(4).tolist(),
            "tray_contact_eval": self.on(self.tray_geom),
        }

    def score(self):
        xyz = self.data.xpos[self.object_id] * 1000
        velocity = float(np.linalg.norm(self.data.qvel[self.object_dof:self.object_dof + 3]))
        center_inside = bool(np.all(np.abs(xyz[:2] - TRAY) < TRAY_HALF_MM - 2))
        tray_contact = self.on(self.tray_geom)
        fingers_open = bool(self.data.qpos[5:7].sum() < .012)
        fingers_clear = not any(self.on(geom) for geom in self.skin_ids)
        return {
            "success": bool(center_inside and tray_contact and fingers_open and fingers_clear and velocity < .02),
            "object_xyz_mm": xyz.round(3).tolist(), "speed_m_s": round(velocity, 5),
            "tray_contact": tray_contact, "center_inside": center_inside,
            "fingers_open": fingers_open, "fingers_clear": fingers_clear,
        }


def rgbd(bench):
    wrist_size = tuple(SETUP["depth_resolution"])
    depth = bench._render("wrist_depth", wrist_size, "depth")
    reference = bench._render("wrist_depth", wrist_size).astype(np.int16)
    red, green, blue = reference.transpose(2, 0, 1)
    # A rendered intensity image in the depth camera's optics supplies a
    # registered color mask. True geom IDs and true object pose stay in scoring.
    mask = (red > green + 23) & (green > blue + 20) & (red > 102)
    valid = np.isfinite(depth) & (depth >= SETUP["depth_min_m"]) & (depth <= SETUP["depth_max_m"])
    rows, cols = np.nonzero(mask & valid)
    intrinsics, transform = depth_calibration(bench)
    z = depth[rows, cols]
    focal = intrinsics.diagonal()[:2]
    pixels = np.column_stack((cols, rows)) + .5
    points = np.column_stack(((pixels - intrinsics[:2, 2]) * z[:, None] / focal, z))
    world_mm = (points @ transform[:3, :3].T + transform[:3, 3]) * 1000
    within = (world_mm[:, 2] > 2) & (world_mm[:, 2] < 40)
    within &= ((world_mm[:, 0] > 145) & (world_mm[:, 0] < 276)
               & (world_mm[:, 1] > -64) & (world_mm[:, 1] < 80))
    points = world_mm[within]
    if len(points) < 24:
        raise ValueError(f"Only {len(points)} calibrated target pixels in the pick area")
    top = points[points[:, 2] > np.percentile(points[:, 2], 92) - 2.2]
    low, high = np.percentile(top[:, :2], [2, 98], axis=0)
    center = (low + high) / 2
    values, vectors = np.linalg.eigh(np.cov(top[:, :2].T))
    axis = vectors[:, -1]
    heading = float((np.degrees(np.arctan2(axis[1], axis[0])) + 90) % 180 - 90)
    elongation = float(values[1] / max(values[0], 1))
    height = float(np.median(top[:, 2]))
    observation = {
        "depth_target_pixels": len(points), "top_target_pixels": len(top),
        "top_center_world_mm": np.round([*center, height], 2).tolist(),
        "top_span_world_xy_mm": (high - low).round(2).tolist(),
        "top_major_axis_yaw_deg": round(heading, 2), "top_axis_variance_ratio": round(elongation, 2),
        "depth_p10_p50_p90_mm": np.percentile(z[within] * 1000, [10, 50, 90]).round(2).tolist(),
        "sensor_range_mm": [1000 * SETUP["depth_min_m"], 1000 * SETUP["depth_max_m"]],
    }
    target_mask = np.zeros_like(mask)
    target_mask[rows[within], cols[within]] = True
    return observation, depth, valid, target_mask, intrinsics, transform


def pick_path(x, y, z, yaw):
    return [waypoint(x, y, 59, -13, 76, 1.08, yaw),
            waypoint(x, y, z, -1, 76, .82, yaw),
            waypoint(x, y, z, -1, 20, .65, yaw),
            waypoint(x, y, 71, -15, 20, 1.00, yaw),
            waypoint(x, y, 71, -15, 20, .34, yaw)]


def placement_path(z):
    x, y = TRAY
    return [waypoint(x, y, 76, -17, 20, 1.38),
            waypoint(x, y, z, -1, 20, .80),
            waypoint(x, y, z, -1, 76, .72),
            waypoint(x, y, 82, -18, 76, .92)]


def proposals(bench, observation, order=(0, 1, 2)):
    x, y, height = observation["top_center_world_mm"]
    z = float(np.clip(round(height * .68, 1), 12, 22))
    yaw = observation["top_major_axis_yaw_deg"]
    if abs(yaw) > 40 or observation["top_axis_variance_ratio"] < 1.16:
        yaw = 0
    yaw = float(np.clip(yaw, -24, 24))
    bias = round(float(observation["top_span_world_xy_mm"][0]) * .36, 2)
    offsets = [(0, yaw), (-bias, 0 if abs(yaw) > 6 else -14),
               (bias, yaw if abs(yaw) > 6 else 14)]
    rows = {}
    for name, index in zip("ABC", order):
        dx, heading = offsets[index]
        grasp = [round(x + dx, 2), y, z]
        try:
            bench.preview(pick_path(*grasp, heading))
        except ValueError:
            # The wider edge candidate may exceed this arm's reachable workspace.
            # Move it toward the observed center while keeping its jaw heading.
            grasp[0] = round(x + .5 * dx, 2)
            bench.preview(pick_path(*grasp, heading))
        rows[name] = {"tcp_world_mm": grasp, "yaw_deg": heading}
    return rows


def save_observation(bench, path, sensed, candidates):
    observation, depth, valid, mask, k, transform = sensed
    width, height = 424, 240
    camera_names = ("rgb_scene_left", "rgb_scene_right", "wrist_rgb")
    images = [Image.fromarray(bench._render(name, (width, height))) for name in camera_names]
    blend = np.clip((depth - .195) / .26, 0, 1)[..., None]
    color = (np.array([234, 194, 112]) * (1 - blend) + np.array([18, 66, 117]) * blend).astype(np.uint8)
    color[~valid] = [15, 21, 31]
    color[mask] = [.3 * 40 + .7 * 64, 174, 190]
    panel = Image.fromarray(color).resize((width, height), Image.Resampling.LANCZOS)
    marker = ImageDraw.Draw(panel)
    for name, candidate in candidates.items():
        xyz = np.array(candidate["tcp_world_mm"], dtype=float) / 1000
        # Project a point on the observed top face to label the physical proposal.
        xyz[2] = observation["top_center_world_mm"][2] / 1000
        point = (xyz - transform[:3, 3]) @ transform[:3, :3]
        u, v, w = k @ point
        px, py = u / w * width / depth.shape[1], v / w * height / depth.shape[0]
        marker.ellipse((px - 2, py - 2, px + 2, py + 2), fill="white")
        offset = {"A": -19, "B": -33, "C": 15}[name]
        marker.line((px, py, px, py + offset), fill="white")
        marker.text((px + 3, py + offset - 6), name, fill="white", stroke_width=1, stroke_fill="#172332")
    canvas = Image.new("RGB", (2 * width, 2 * (height + 22)), "#142032")
    drawer = ImageDraw.Draw(canvas)
    for i, (label, image) in enumerate(zip(
            ("LEFT RGB", "RIGHT RGB", "WRIST RGB", "WRIST DEPTH | 195 mm ascending | cyan: target"),
            (*images, panel))):
        x, y = width * (i % 2), (height + 22) * (i // 2)
        drawer.text((x + 8, y + 3), label, fill="white")
        canvas.paste(image, (x, y + 22))
    canvas.save(path, quality=90)


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=SEED)
    args = parser.parse_args()
    run_batch(args.scene, args.out, args.seed)


def run_batch(scene, out, seed):
    out.mkdir(parents=True, exist_ok=True)
    if (out / "results.json").exists():
        raise FileExistsError(f"Choose an empty output directory instead of overwriting {out}")
    results = {"model": MODEL, "effort": EFFORT, "seed": seed, "scene_sha256":
               hashlib.sha256(scene.read_bytes()).hexdigest(), "mujoco": mj.__version__,
               "tray_xy_mm": list(TRAY), "total": 12, "runs": []}
    live = {"run_started_unix_s": time.time(), "active": None, "done": False}
    start = time.monotonic()

    def publish():
        write_json(out / "live.json", {**results, **live})

    publish()
    benches = {}
    labels_rng = np.random.default_rng(seed + 1)
    try:
        for case in cases(seed):
            trial_start = time.monotonic()
            row = {"trial": case.trial, "shape": case.shape, "case_eval": asdict(case),
                   "inference_s": None, "sim_s": 0, "selection": None, "success": False}
            prefix = f"trial-{case.trial:02d}"
            live["active"] = {"trial": case.trial, "shape": case.shape, "phase": "reset",
                              "started_unix_s": time.time()}
            publish()

            def phase(name, **extra):
                live["active"].update(phase=name, **extra)
                publish()

            def snapshot(image, _sample=None, force=False):
                now = time.monotonic()
                if force or now - snapshot.last >= .27:
                    temporary = out / "frame.tmp"
                    Image.fromarray(image).save(temporary, format="JPEG", quality=85)
                    temporary.replace(out / "frame.jpg")
                    snapshot.last = now
            snapshot.last = -100

            bench = None
            replay = []

            def record(image, sample):
                replay.append(Image.fromarray(image).resize((540, 304), Image.Resampling.BILINEAR))
                snapshot(image, sample)

            try:
                if case.shape not in benches:
                    benches[case.shape] = PickBench(scene, case.shape)
                bench = benches[case.shape]
                bench.reset(case)
                snapshot(bench.frame(), force=True)
                sensed = rgbd(bench)
                candidates = proposals(bench, sensed[0], labels_rng.permutation(3))
                observation_image = out / f"{prefix}-rgbd.jpg"
                save_observation(bench, observation_image, sensed, candidates)
                row.update(observation=sensed[0], candidates=candidates, rgbd=observation_image.name)
                phase("inference", rgbd=observation_image.name, candidates=candidates)
                prompt = PROMPT + "\n" + json.dumps({"observed_rgbd": sensed[0], "candidates": candidates})
                receipt = infer(prompt, observation_image, SCHEMA)
                row["inference_s"] = receipt["wall_s"]
                row["runtime"] = receipt.get("runtime")
                row["turn_id"] = receipt.get("turn_id")
                row["receipt"] = f"{prefix}-astra.json"
                receipt.update(prompt=prompt, image=observation_image.name,
                               image_sha256=hashlib.sha256(observation_image.read_bytes()).hexdigest())
                write_json(out / row["receipt"], receipt)
                if "error" in receipt:
                    row["outcome"] = "inference_error"
                    raise ValueError(receipt["error"])
                selected = receipt["response"]["grasp"]
                row.update(selection=selected, evidence=receipt["response"]["evidence"])
                if selected == "skip":
                    row["outcome"] = "agent_skip"
                else:
                    chosen = candidates[selected]
                    x, y, z = chosen["tcp_world_mm"]
                    plan = pick_path(x, y, z, chosen["yaw_deg"])
                    bench.frame_sink = record
                    for name, point in zip(("approach", "descend", "close", "lift", "lift"), plan):
                        phase(name, selection=selected)
                        bench.run([point], name)
                        snapshot(bench.frame(), force=True)
                    approach_height = bench.trace[0]["object_xyz_mm_eval"][2]
                    lift_height = bench.trace[-1]["object_xyz_mm_eval"][2]
                    row["lift_mm_eval"] = round(lift_height - approach_height, 2)
                    # Direct execution of the chosen grasp through the fixed
                    # waypoint controller. The evaluator never gates transfer.
                    for name, point in zip(("transfer", "lower", "release", "retreat"),
                                           placement_path(z + FLOOR_MM + 7.63 + 2)):
                        phase(name)
                        bench.run([point], name)
                        snapshot(bench.frame(), force=True)
                    phase("settle")
                    bench.settle(.60)
                    row["score"] = bench.score()
                    row["success"] = row["score"]["success"]
                    row["outcome"] = "placed" if row["success"] else "placement_failed"
            except (ValueError, RuntimeError, KeyError) as error:
                row.setdefault("outcome", "execution_error")
                row["error"] = str(error)
                print(f"{prefix}: {row['outcome']}: {error}", flush=True)
            if bench is not None:
                bench.frame_sink = None
                row["sim_s"] = round(float(bench.data.time), 3)
                row["trace"] = f"{prefix}-trace.json"
                write_json(out / row["trace"], bench.trace)
                snapshot(bench.frame(), force=True)
                row["end_frame"] = f"{prefix}-end.jpg"
                Image.fromarray(bench.frame()).save(out / row["end_frame"], quality=88)
            row["total_s"] = round(time.monotonic() - trial_start, 3)
            if replay:
                replay[0].save(out / f"{prefix}-motion.webp", save_all=True, append_images=replay[1:],
                               duration=80, loop=0, quality=69, method=1)
                row["replay"] = f"{prefix}-motion.webp"
            results["runs"].append(row)
            live["active"] = None
            publish()
            print(f"{prefix}: {row['outcome']}, {row['total_s']:.3f} s total,"
                  f" {row['inference_s']} s inference", flush=True)
    finally:
        live.update(done=True, aborted=len(results["runs"]) != results["total"])
        results["batch_s"] = round(time.monotonic() - start, 3)
        write_json(out / "results.json", results)
        publish()
        fields = ("trial", "shape", "selection", "success", "outcome", "total_s", "inference_s",
                  "sim_s", "lift_mm_eval", "turn_id", "error")
        with (out / "runs.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results["runs"])
        for bench in benches.values():
            bench.close()


if __name__ == "__main__":
    main()
