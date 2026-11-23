"""Replay cross-matched release targets and measure small position perturbations."""

import argparse
from pathlib import Path
import json
from statistics import mean, pstdev

from bench import HERE, TOWER, StackBench
from evaluate import atomic
from feedback_ablation import replay

JITTER_MM = (-1., -.5, -.25, 0., .25, .5, 1.)


def retarget(path, x):
    """Translate every waypoint above the destination along world X."""
    return [{**point, "xyz_mm": [x, *point["xyz_mm"][1:]]} if point["xyz_mm"][1] == TOWER[1]
            else point for point in path]


def measure(scene, lift, row, x):
    path = retarget(row["place_waypoints"], x)
    verdict, trace = replay(scene, row["hidden_cg_mm_eval"], lift, path)
    up = max((sample["pad_load_world_N"][2] for sample in trace if sample["phase"] == "place"),
             default=0.)
    return {"release_tcp_x_mm": x, "outcome": verdict, "peak_upward_pad_force_N": up}


def hover_trial(scene, lift, row, remeasure):
    """Isolate the new measurement with the same hover and motion time in the control."""
    path = row["place_waypoints"]
    stop = next(i for i, point in enumerate(path)
                if point["xyz_mm"][1] == TOWER[1] and point["xyz_mm"][2] >= 78)
    above = path[stop]
    bench = StackBench(scene, row["hidden_cg_mm_eval"])
    try:
        bench.run(lift, "lift")
        bench.run(path[:stop+1], "transport")
        bench.run([{**above, "time_s": .44}], "hover")
        samples = bench.trace[-8:]
        load = [-sample["pad_load_world_N"][2] for sample in samples]
        moment = mean(sample["pad_moment_about_tcp_mNm"][1] for sample in samples)
        valid = .40 <= mean(load) <= 1.25 and pstdev(load) < .08
        x = row["release_tcp_x_mm"]
        if remeasure and valid:
            correction = TOWER[0] - moment / mean(load)
            x = max(TOWER[0]-22, min(TOWER[0]+22, correction))
        adjustment = retarget([{**above, "time_s": .40}, *path[stop+1:]], x)
        bench.run(adjustment, "place")
        bench.settle()
        return {"offset_mm_eval": row["hidden_cg_mm_eval"], "used_hover_measurement": remeasure,
                "hover_load_N": mean(load), "hover_load_sd_N": pstdev(load),
                "hover_moment_mNm": moment, "valid_measurement": valid,
                "first_release_x_mm": row["release_tcp_x_mm"], "final_release_x_mm": x,
                "outcome": bench.verdict()}
    finally:
        bench.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    results = json.loads(args.results.read_text())
    lift = results["lift"]
    scene = args.scene.resolve()
    selected = {row["policy"]: row for row in results["trials"] if row["hidden_cg_mm_eval"] == -16}
    paths = ("astra_aligned", "moment_rule")
    cross = []
    for route in paths:
        for target in paths:
            sample = measure(scene, lift, selected[route], selected[target]["release_tcp_x_mm"])
            cross.append({"route": route, "target_from": target, **sample})
    tolerance = []
    for row in results["trials"]:
        if row["policy"] != "moment_rule":
            continue
        samples = []
        for delta in JITTER_MM:
            sample = measure(scene, lift, row, row["release_tcp_x_mm"]+delta)
            samples.append({"perturbation_mm": delta, **sample})
        tolerance.append({"offset_mm_eval": row["hidden_cg_mm_eval"], "samples": samples})
    hover = [hover_trial(scene, lift, row, remeasure)
             for row in results["trials"] if row["policy"] == "astra_aligned"
             for remeasure in (False, True)]
    atomic(args.output, {"left16_crossed_routes": cross, "moment_rule_release_jitter": tolerance,
                         "matched_hover_feedback": hover})


if __name__ == "__main__":
    main()
