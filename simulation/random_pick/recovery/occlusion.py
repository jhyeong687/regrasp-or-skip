"""Paired occlusion benchmark with two new workpieces and four held-out layouts."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from recovery import run as recovery
from recovery import vision_alignment as visual

VISIBILITIES = ("clear", "marker_occlusion", "object_occlusion")
OPTICAL_SCREENS = 5
DESIGN_SEED = 26071104
PROFILES = {
    "hex_prism": {"sides": 6, "half_xy": (22, 18), "gap_mm": 27, "low_friction": .125},
    "oval_prism": {"sides": 24, "half_xy": (23, 16), "gap_mm": 28, "low_friction": .125},
}
# The same four layouts appear under each visibility level. The object poses
# have never been used by the previous 12-layout experiments.
LAYOUTS = (
    ("hex_prism", "off_center", 201.41, -26.85, -9.7, 1),
    ("hex_prism", "low_friction", 235.32, 41.25, 12.4, -1),
    ("oval_prism", "off_center", 219.61, 37.82, -12.8, -1),
    ("oval_prism", "low_friction", 236.39, -26.64, 9.8, 1),
)


GRADED_SEED = 26112630
GRADED_VISIBILITIES = ("clear", "marker_occlusion", "one_side", "slit", "full_occlusion")
GRADED_PROFILES = {
    "beveled_block": {
        "outline_mm": [(21, 11), (16, 16), (-16, 16), (-21, 11),
                       (-21, -11), (-16, -16), (16, -16), (21, -11)],
        "half_xy": (21, 16), "gap_mm": 28, "half_height_mm": 16,
    },
    "rounded_rectangle": {
        "sides": 32, "exponent": 4, "half_xy": (24, 16), "gap_mm": 28, "half_height_mm": 17,
    },
    "tapered_block": {
        "outline_mm": [(23, 11), (17, 15.5), (-20, 16), (-23, 13),
                       (-23, -13), (-20, -16), (17, -15.5), (23, -11)],
        "half_xy": (23, 16), "gap_mm": 27, "half_height_mm": 18,
    },
}
GRADED_LAYOUTS = (
    ("beveled_block", "off_center", 207.82, -34.73, -9.9, 1),
    ("beveled_block", "low_friction", 235.65, 25.43, 9.2, -1),
    ("rounded_rectangle", "off_center", 224.41, 42.65, 11.7, -1),
    ("rounded_rectangle", "low_friction", 203.68, -9.12, -11.7, 1),
    ("tapered_block", "off_center", 218.05, 36.17, 5.7, 1),
    ("tapered_block", "low_friction", 241.27, -17.81, -6.4, -1),
)
CALIBRATION_POSES = ((206, 19, -3), (227, -3, 7))
FRICTION_GRID = (.075, .090, .105, .115, .125, .135)


def design(layouts=LAYOUTS, visibilities=VISIBILITIES, seed=DESIGN_SEED):

    rng = np.random.default_rng(seed)
    order = rng.permutation([(i, j) for i in range(len(layouts)) for j in range(len(visibilities))])
    counters = Counter()
    for trial, (base_idx, visibility_idx) in enumerate(order, 1):
        shape, fault, x, y, yaw, sign = layouts[base_idx]
        visibility = visibilities[visibility_idx]
        first = counters[visibility, fault]
        if len(visibilities) > 3:
            first += visibility_idx + int(fault == "low_friction")
        modes = recovery.MODES if first % 2 == 0 else recovery.MODES[::-1]
        counters[visibility, fault] += 1
        yield recovery.Case(trial, shape, x, y, yaw), fault, sign, visibility, int(base_idx + 1), modes


def object_mesh(profile):
    def add_mesh(asset, geom):
        if "outline_mm" in profile:
            outline = np.asarray(profile["outline_mm"]) / 1000
        else:
            theta = np.arange(profile["sides"]) * 2 * np.pi / profile["sides"]
            unit = np.column_stack((np.cos(theta), np.sin(theta)))
            exponent = 2 / profile.get("exponent", 2)
            outline = np.sign(unit) * np.abs(unit)**exponent * (np.asarray(profile["half_xy"]) / 1000)
        hz = profile.get("half_height_mm", 17) / 1000
        vertices = [(x, y, z) for x, y in outline for z in (-hz, hz)]
        ET.SubElement(asset, "mesh", name="held_out_prism",
                      vertex=" ".join(f"{v:.7f}" for xyz in vertices for v in xyz))
        geom.set("type", "mesh")
        geom.set("mesh", "held_out_prism")
        geom.attrib.pop("size", None)
    return add_mesh


def camera_screen(world, name, suffix, half_width, half_height, vertical_offset=0, *, half_thickness=.001):
    camera = world.find(f".//camera[@name='{name}']")
    origin = np.fromstring(camera.attrib["pos"], sep=" ")
    right, up = np.fromstring(camera.attrib["xyaxes"], sep=" ").reshape(2, 3)
    back = np.cross(right, up)
    back /= np.linalg.norm(back)
    center = origin - .058 * back + vertical_offset * up
    ET.SubElement(world, "geom", name=f"{name}_{suffix}", type="box",
                  pos=" ".join(map(str, center)), xyaxes=camera.attrib["xyaxes"],
                  size=f"{half_width} {half_height} {half_thickness}", rgba=".26 .29 .33 1",
                  group=str(OPTICAL_SCREENS), contype="0", conaffinity="0", mass="0")


def view_setup(visibility, graded=False):
    def add_view(world):
        visual.add_alignment_view(world)
        if visibility == "clear":
            return
        camera_screen(world, visual.CAMERA, "screen", .028, .021)
        if graded:
            if visibility in ("one_side", "slit", "full_occlusion"):
                camera_screen(world, "rgb_scene_right", "screen", .051, .032)
            if visibility == "full_occlusion":
                camera_screen(world, "rgb_scene_left", "screen", .051, .032)
            if visibility == "slit":
                # Leave a fixed horizontal opening at normalized rows 0.66–0.73.
                # The shutter positions come from static camera field of view.
                height_at_screen = .058 * 360 / 457.0073622
                for index, (first, last) in enumerate(((-.2, .66), (.73, 1.2))):
                    camera_screen(world, "rgb_scene_left", f"slit_{index}", .051,
                                  (last - first) * height_at_screen / 2,
                                  (.5 - (first + last) / 2) * height_at_screen, half_thickness=.0005)
        elif visibility == "object_occlusion":
            for name in ("rgb_scene_left", "rgb_scene_right"):
                camera_screen(world, name, "screen", .036, .019)
    return add_view


class OcclusionBench(recovery.RecoveryBench):
    archive_raw = True

    def __init__(self, scene, shape, visibility, *, profiles=PROFILES, graded=False):
        self.profile = profiles[shape]
        super().__init__(scene, "brick", visual_fixture=view_setup(visibility, graded),
                         object_fixture=object_mesh(self.profile))

    def plan_grasp(self, selected, condition, sign, shape):
        return recovery.grasp_parameters(self, selected, condition, sign, shape,
                                         geometry=(self.profile["gap_mm"], self.profile["half_xy"][0]))

    def _render(self, name, shape, mode="rgb"):
        old = self.option.geomgroup[OPTICAL_SCREENS]
        # Fixed panels enter the optical paths after the shared pre-pick scan.
        # They are visual geometry with no contact response.
        self.option.geomgroup[OPTICAL_SCREENS] = int(self.data.time >= .5)
        try:
            return super()._render(name, shape, mode)
        finally:
            self.option.geomgroup[OPTICAL_SCREENS] = old


def visibility_summary(records, visibilities=VISIBILITIES):
    summary = {}
    for v in visibilities:
        visible = [r for r in records if r["visibility"] == v]
        summary[v] = recovery.summarize_results(visible)
    return summary


def run(args):
    args.out.mkdir(parents=True, exist_ok=True)
    if any((args.out / name).exists() for name in ("results.json", "live.json")):
        raise FileExistsError("Choose a fresh output directory")
    graded = args.suite == "graded"
    layouts, visibilities, seed = (GRADED_LAYOUTS, GRADED_VISIBILITIES, GRADED_SEED) if graded else (
        LAYOUTS, VISIBILITIES, DESIGN_SEED)
    profiles = PROFILES
    calibration = None
    if graded:
        calibration = json.loads(args.calibration.read_text())
        if calibration["scene_sha256"] != recovery.digest(args.scene):
            raise ValueError("Use the scene used for calibration")
        profiles = {name: {**profile, "low_friction": calibration["chosen"][name]}
                    for name, profile in GRADED_PROFILES.items()}
    cases = list(design(layouts, visibilities, seed))
    started = time.monotonic()
    result = {
        "model": recovery.MODEL, "effort": recovery.EFFORT,
        "mujoco": recovery.mj.__version__, "scene_sha256": recovery.digest(args.scene),
        "visual_alignment": True, "recovery_controller": "clearance_recovery",
        "occlusion_benchmark": True, "suite": args.suite, "planned": 2 * len(cases), "seed": seed,
        "visibility_order": list(visibilities), "profiles": profiles, "layouts": layouts, "batch_started_unix_s": time.time(), "records": [],
    }
    if calibration is not None:
        recovery.write_json(args.out / "calibration.json", calibration)
    benches = {}
    paired_inputs = {}
    current = None

    def publish(done=False):
        result["summary"] = recovery.summarize_results(result["records"])
        result["visibility_summary"] = visibility_summary(result["records"], visibilities)
        recovery.write_json(args.out / "live.json", {**result, "active": current, "done": done})

    publish()
    try:
        for case, fault, sign, visibility, layout, modes in cases:
            key = case.shape, visibility
            if key not in benches:
                benches[key] = OcclusionBench(args.scene, *key, profiles=profiles, graded=graded)
            bench = benches[key]
            friction = recovery.NORMAL_FRICTION if fault == "off_center" else profiles[case.shape]["low_friction"]
            for mode in modes:
                current = {"trial": case.trial, "shape": case.shape, "mode": mode, "fault": fault,
                           "visibility": visibility, "layout": layout, "phase": "reset",
                           "trial_started_unix_s": time.time()}
                publish()

                def stage(name, **update):
                    current.update(phase=name, **update)
                    publish()

                row = recovery.run_trial(bench, case, fault, sign, mode, None, friction, args.out, paired_inputs,
                                         stage, use_contact_history=True, clearance_recovery=True, vision_alignment=True,
                                         inference_retries=1)
                row.update(visibility=visibility, layout=layout)
                result["records"].append(row)
                current = None
                publish()
                print(f"{case.trial:02d} {visibility} {mode}: {row['action']} {row['outcome']} "
                      f"{row['total_s']:.2f} s total, {row['inference_s']} s inference", flush=True)
    finally:
        result["batch_s"] = round(time.monotonic() - started, 3)
        result["aborted"] = len(result["records"]) != result["planned"]
        current = None
        fields = ("trial", "layout", "shape", "visibility", "condition", "mode", "action",
                  "success", "outcome", "total_s", "inference_s", "sim_s", "turn_id", "error")
        with (args.out / "runs.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fields, extrasaction="ignore")
            writer.writeheader()
            for row in result["records"]:
                writer.writerow({**row, "condition": row["fault_eval"]["condition"]})
        publish(done=True)
        recovery.write_json(args.out / "results.json", result)
        for bench in benches.values():
            bench.close()

def calibrate(scene, output):
    """Choose friction on separate positions with slip and successful reinforcement."""
    outcome = {"scene_sha256": recovery.digest(scene), "poses": CALIBRATION_POSES, "grid": FRICTION_GRID,
               "chosen": {}, "measurements": {}}
    for shape in GRADED_PROFILES:
        bench = OcclusionBench(scene, shape, "clear", profiles=GRADED_PROFILES, graded=True)
        bench.archive_raw = False
        try:
            measurements = []
            for mu in FRICTION_GRID:
                scores = []
                for index, (x, y, yaw) in enumerate(CALIBRATION_POSES, 101):
                    trial = recovery.Case(index, shape, x, y, yaw)
                    bench.reset_trial(trial, mu)
                    z = float(bench.data.xpos[bench.object_id, 2] * 1000)
                    observed = recovery.rgbd(bench)[0]
                    selected = recovery.proposals(bench, observed)["A"]
                    nominal, first, gap = bench.plan_grasp(selected, "low_friction", 1, shape)
                    recovery.probe(bench, first, gap, lambda _: ())
                    arm, _ = recovery.summarize(bench.trace)
                    test = recovery.evaluate_probe(bench, z)
                    recovery.retry(first, gap, recovery.ACTIONS[1], bench.run, nominal,
                                   clear_before_descent=abs(arm["hold"]["jaw_gap_mm"] - gap) <= 1)
                    bench.settle(.60)
                    scores.append([test["lifted"], bench.score()["success"]])
                measurements.append({"coefficient": mu, "probe_lifted_and_recovered": scores})
                print(shape, mu, scores, flush=True)
            viable = [v["coefficient"] for v in measurements
                      if all(not lifted and success for lifted, success in v["probe_lifted_and_recovered"])]
            if not viable:
                raise ValueError(f"No calibration coefficient recovered the low-friction {shape}")
            outcome["chosen"][shape] = max(viable)
            outcome["measurements"][shape] = measurements
            recovery.write_json(output, outcome)
        finally:
            bench.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--suite", choices=("pilot", "graded"), default="pilot")
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--calibrate-only", action="store_true")
    args = parser.parse_args()
    if args.calibrate_only:
        args.out.mkdir(parents=True, exist_ok=True)
        calibrate(args.scene, args.out / "calibration.json")
    else:
        if args.suite == "graded" and args.calibration is None:
            parser.error("--suite graded requires --calibration")
        run(args)
