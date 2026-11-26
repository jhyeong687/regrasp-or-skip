"""Compare two scripted regrasp actions on the same shell in MuJoCo."""

import argparse
import csv
from dataclasses import asdict
import gzip
import hashlib
import json
from pathlib import Path
from statistics import median

import mujoco as mj
import numpy as np

from run_recovery_pilot import (
    Case, FIRST_SLIDE_M, HOLD_SAMPLES, MAX_TILT_DEG, MIN_LIFT_MM,
    RETRY_SLIDE_M, STEP_S, ProbeRunner, summarize, vector,
)


MASS_G = 50.0
OFFSET_MM = (-31.5, -30, -28.5, -27, -25.5, -24, -22.5, -21,
             19.5, 21, 22.5, 24, 25.5, 27, 28.5, 30)
LOW_FRICTION = (.018, .033, .048, .063)
CENTER_JITTER_MM = (-2.5, -.75, .75, 2.5)
CONDITIONS = ("off_center", "low_friction")
ACTIONS = ("position_correction", "grip_reinforcement")
PHASES = ("first", *ACTIONS)


def cases():
    layouts = {
        "off_center": [(offset, .8) for offset in OFFSET_MM],
        "low_friction": [(offset, friction) for friction in LOW_FRICTION
                         for offset in CENTER_JITTER_MM],
    }
    return [Case(f"{condition}-{i:02d}", condition, "comparison", offset, MASS_G, friction)
            for condition, layout in layouts.items()
            for i, (offset, friction) in enumerate(layout, start=1)]


class ComparisonRunner(ProbeRunner):
    def read_sample(self, phase):
        sample = super().read_sample(phase)
        sample["scene_qpos"] = vector(self.sim.data.qpos, 7)
        return sample

    def compare(self, case):
        self.reset(case)
        first_trace, first, _ = self.attempt(self.grasp, self.lift, FIRST_SLIDE_M)
        set_down = self.move("lower", .88, self.grasp, FIRST_SLIDE_M)
        set_down += self.move("release", .64, self.grasp, 0)
        set_down += self.move("settle", .24, self.grasp, 0)
        checkpoint = self.snapshot()

        # Isolate the recovery action from position estimation. Align to the
        # shell's simulated X displacement from the nominal scene reference.
        shift_m = float(self.sim.data.qpos[self.object_q] - self.model.qpos0[self.object_q])
        retries = {}
        traces = {"first": first_trace, "set_down": set_down}
        for name, dx, slide in (
            ("position_correction", shift_m, FIRST_SLIDE_M),
            ("grip_reinforcement", 0.0, RETRY_SLIDE_M),
        ):
            self.restore(checkpoint)
            grasp, lift = self.shift(self.grasp, dx), self.shift(self.lift, dx)
            reposition = self.move("reposition", .72, grasp, 0)
            attempt, result, _ = self.attempt(grasp, lift, slide)
            retries[name] = result
            traces[name] = reposition + attempt

        observations = summarize(first_trace, None, None, None)
        pinch_normal = observations["tactile"]["pinch"]["normal_N_lr"]
        return {
            "id": case.id,
            "case_truth": asdict(case),
            "correction_mm": round(1000 * shift_m, 3),
            "first_pinch_normal_N_lr": pinch_normal,
            "first_hold_normal_N_lr": observations["tactile"]["end_hold"]["normal_N_lr"],
            "first": first,
            **retries,
        }, traces


def summarize_records(records):
    by_condition = {}
    for condition in CONDITIONS:
        trials = [record for record in records if record["case_truth"]["scene"] == condition]
        failures = [record for record in trials if not record["first"]["success"]]
        by_condition[condition] = {
            "trials": len(trials),
            "first_successes": len(trials) - len(failures),
            "actions": {
                action: {
                    "successes": sum(record[action]["success"] for record in trials),
                    "recoveries_after_first_failure": sum(record[action]["success"]
                                                          for record in failures),
                    "median_hold_min_rise_mm": median(
                        record[action]["hold_min_rise_mm"] for record in trials
                    ) if trials else None,
                }
                for action in ACTIONS
            },
        }
    return by_condition


def write_results(output, records, scene_path):
    fields = ["id", "condition", "mass_g", "offset_mm", "friction", "correction_mm",
              "first_pinch_normal_left_N", "first_pinch_normal_right_N",
              "first_hold_normal_left_N", "first_hold_normal_right_N"]
    outcome_fields = tuple(records[0]["first"])
    fields += [f"{phase}_{key}" for phase in PHASES for key in outcome_fields]
    with (output / "trials.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for record in records:
            case = record["case_truth"]
            writer.writerow([record["id"], case["scene"], case["mass_g"], case["offset_mm"],
                             case["friction"], record["correction_mm"],
                             *record["first_pinch_normal_N_lr"], *record["first_hold_normal_N_lr"],
                             *(record[phase][key] for phase in PHASES for key in outcome_fields)])

    summary = {
        "protocol": {
            "mujoco_version": mj.__version__,
            "numpy_version": np.__version__,
            "scene_sha256": hashlib.sha256(scene_path.read_bytes()).hexdigest(),
            "shell_mass_g": MASS_G,
            "sample_interval_s": STEP_S,
            "hold_scored_s": HOLD_SAMPLES * STEP_S,
            "minimum_hold_rise_mm": MIN_LIFT_MM,
            "maximum_hold_tilt_deg": MAX_TILT_DEG,
            "first_and_position_correction_slide_m_per_jaw": FIRST_SLIDE_M,
            "grip_reinforcement_slide_m_per_jaw": RETRY_SLIDE_M,
            "position_hint": "MuJoCo shell X after release, relative to the scene reference",
            "paired_checkpoint": "After the shared first attempt, set-down, release and settle",
        },
        "conditions": summarize_records(records),
        "trials": records,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")


def run(output, limit, scene_path):
    output.mkdir(parents=True, exist_ok=False)
    runner = ComparisonRunner(scene_path)
    records = []
    try:
        with gzip.open(output / "traces.jsonl.gz", "wt", compresslevel=5) as raw:
            for case in cases()[:limit]:
                record, traces = runner.compare(case)
                records.append(record)
                raw.write(json.dumps({"id": case.id, "traces": traces},
                                     separators=(",", ":"), allow_nan=False) + "\n")
                print(case.id, *(f"{phase}={record[phase]['hold_min_rise_mm']:.2f}mm"
                                 for phase in PHASES), flush=True)
    finally:
        runner.sim.close()
    write_results(output, records, runner.sim.scene_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="New directory for run results")
    parser.add_argument("--scene", type=Path, help="Load a local R7 scene and its adjacent assets")
    parser.add_argument("--limit", type=int, help="Run only the first N configurations")
    args = parser.parse_args()
    if args.limit is not None and args.limit < 1:
        parser.error("--limit must be positive")
    run(args.output, args.limit, args.scene)
