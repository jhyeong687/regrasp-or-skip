"""Paired occlusion benchmark with two new workpieces and four held-out layouts."""

import argparse
from collections import Counter
import csv
import json
from pathlib import Path
from statistics import median
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


def design():
    rng = np.random.default_rng(DESIGN_SEED)
    order = rng.permutation([(i, j) for i in range(len(LAYOUTS)) for j in range(len(VISIBILITIES))])
    counters = Counter()
    for trial, (base_idx, visibility_idx) in enumerate(order, 1):
        shape, fault, x, y, yaw, sign = LAYOUTS[base_idx]
        visibility = VISIBILITIES[visibility_idx]
        modes = recovery.MODES if counters[visibility, fault] % 2 == 0 else recovery.MODES[::-1]
        counters[visibility, fault] += 1
        yield recovery.Case(trial, shape, x, y, yaw), fault, sign, visibility, int(base_idx + 1), modes


def object_mesh(shape):
    profile = PROFILES[shape]

    def add_mesh(asset, geom):
        sides = profile["sides"]
        half_x, half_y = np.asarray(profile["half_xy"]) / 1000
        theta = np.arange(sides) * 2 * np.pi / sides
        outline = zip(np.cos(theta) * half_x, np.sin(theta) * half_y)
        vertices = [(x, y, z) for x, y in outline for z in (-.017, .017)]
        ET.SubElement(asset, "mesh", name="held_out_prism",
                      vertex=" ".join(f"{v:.7f}" for xyz in vertices for v in xyz))
        geom.set("type", "mesh")
        geom.set("mesh", "held_out_prism")
        geom.attrib.pop("size", None)
    return add_mesh


def view_setup(visibility):
    def add_view(world):
        visual.add_alignment_view(world)
        cameras = {visual.CAMERA: (.028, .021)}
        if visibility == "object_occlusion":
            cameras.update(rgb_scene_left=(.036, .019), rgb_scene_right=(.036, .019))
        if visibility == "clear":
            return
        for name, (half_x, half_y) in cameras.items():
            camera = world.find(f".//camera[@name='{name}']")
            origin = np.fromstring(camera.attrib["pos"], sep=" ")
            right, up = np.fromstring(camera.attrib["xyaxes"], sep=" ").reshape(2, 3)
            back = np.cross(right, up)
            back /= np.linalg.norm(back)
            center = origin - .058 * back
            ET.SubElement(world, "geom", name=f"{name}_screen", type="box",
                          pos=" ".join(map(str, center)), xyaxes=camera.attrib["xyaxes"],
                          size=f"{half_x} {half_y} .001", rgba=".26 .29 .33 1",
                          group=str(OPTICAL_SCREENS), contype="0", conaffinity="0", mass="0")
    return add_view


class OcclusionBench(recovery.RecoveryBench):
    archive_raw = True

    def __init__(self, scene, shape, visibility):
        self.profile = PROFILES[shape]
        super().__init__(scene, "brick", visual_fixture=view_setup(visibility), object_fixture=object_mesh(shape))

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


def visibility_summary(records):
    summary = {}
    for v in VISIBILITIES:
        visible = [r for r in records if r["visibility"] == v]
        summary[v] = recovery.summarize_results(visible)
    return summary


def run(args):
    args.out.mkdir(parents=True, exist_ok=True)
    if any((args.out / name).exists() for name in ("results.json", "live.json")):
        raise FileExistsError("Choose a fresh output directory")
    cases = list(design())
    started = time.monotonic()
    result = {
        "model": recovery.MODEL, "effort": recovery.EFFORT,
        "mujoco": recovery.mj.__version__, "scene_sha256": recovery.digest(args.scene),
        "visual_alignment": True, "recovery_controller": "clearance_recovery",
        "occlusion_benchmark": True, "planned": 2 * len(cases), "seed": DESIGN_SEED,
        "profiles": PROFILES, "layouts": LAYOUTS, "batch_started_unix_s": time.time(), "records": [],
    }
    benches = {}
    paired_inputs = {}
    current = None

    def publish(done=False):
        result["summary"] = recovery.summarize_results(result["records"])
        result["visibility_summary"] = visibility_summary(result["records"])
        recovery.write_json(args.out / "live.json", {**result, "active": current, "done": done})

    publish()
    try:
        for case, fault, sign, visibility, layout, modes in cases:
            key = case.shape, visibility
            if key not in benches:
                benches[key] = OcclusionBench(args.scene, *key)
            bench = benches[key]
            friction = recovery.NORMAL_FRICTION if fault == "off_center" else PROFILES[case.shape]["low_friction"]
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    run(parser.parse_args())
