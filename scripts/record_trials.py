"""Record deterministic scripted trials without opening an OpenGL renderer."""
from pathlib import Path
import argparse
import csv
import json
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from sim import Sim, DURATION, POINTS

CASES = ("normal", "round", "slippery", "heavy_matched")
CHECKPOINTS = (2.7, 6.2, 7.5, 7.8, 8.1, 8.5, 8.9, 10.5, 12.9, DURATION)
STEP_S = .04


def flat(state):
    result = {
        "scenario": state["scenario"], "time_s": state["time_s"], "phase": state["phase"],
        "mass_g_truth": state["mass_g"], "friction_truth": state["pad_friction"],
        "lift_mm_truth": state["lift_mm"], "jaw_gap_mm": state["gap_mm"],
        "elbow_torque_Nm": state["motor_Nm"][2], "target_depth_m_truth": state["depth_target_m"],
        "tactile_average_window_s": state["tactile_average_window_s"],
    }
    for i, side in enumerate(("left", "right")):
        features = state["tactile_features"]["pads"][i]
        for prefix, axes, values in [
            ("F", "xyz", state["paxini_resultant_N"][i]),
            ("T", "xyz", state["paxini_torque_Nm"][i]),
            ("cop_", "xy", features["cop_xy_mm"] or [None, None]),
        ]:
            result.update({f"{side}_{prefix}{axis}": value for axis, value in zip(axes, values)})
        for key in ("active_points", "peak_normal_N", "shear_N", "shear_normal_ratio", "cop_change_mm"):
            result[f"{side}_{key}"] = features[key]
    return result


def record(output):
    output.mkdir(parents=True, exist_ok=True)
    rows, snapshots, summary = [], {}, {}
    sim = Sim()
    try:
        for name in CASES:
            sim.reset(name)
            rows.append(flat(sim.observations()))
            while sim.data.time < DURATION - .001:
                sim.advance(STEP_S)
                rows.append(flat(sim.observations()))

            snapshots[name] = []
            for checkpoint in CHECKPOINTS:
                sim.seek(checkpoint)
                state = sim.observations()
                state.pop("force_N")
                state.pop("force_sum_N")
                snapshots[name].append(state)
                if checkpoint == 8.9:
                    force = np.asarray(state["paxini_resultant_N"])
                    summary[name] = {
                        "time_s": state["time_s"], "lift_mm": state["lift_mm"],
                        "mean_Fz_N": round(float(force[:, 2].mean()), 3),
                        "mean_abs_Fy_N": round(float(np.abs(force[:, 1]).mean()), 3),
                    }
    finally:
        sim.close()

    with (output / "trial-traces.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys(), lineterminator=chr(10))
        writer.writeheader()
        writer.writerows(rows)
    (output / "trial-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    record = {
        "source": "MuJoCo simulated contact", "model": "PX6AX-GEN3-CP-M3025-Core",
        "axis_order": ["Fx", "Fy", "Fz"], "normal_positive": "compression",
        "sides": ["left", "right"], "taxel_indices": [point["index"] for point in POINTS],
        "scenarios": snapshots,
    }
    (output / "tactile-pilot.json").write_text(json.dumps(record, separators=(",", ":")) + "\n")
    print(f"Recorded {len(rows)} time samples and {sum(map(len, snapshots.values()))} tactile snapshots")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "data")
    record(parser.parse_args().output)
