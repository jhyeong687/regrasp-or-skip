"""Test Astra's response to paired, physically recorded tactile observations."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import gzip
import json
from pathlib import Path

import numpy as np

from bench import HERE, POINTS, TOWER, StackBench, waypoint
from evaluate import EFFORT, MODEL, atomic, call_checked, observation_image, record_measurement, summarize

# Both magnitudes are held out from the original +/-13 mm waypoint pilot.
OFFSETS_MM = (-16, -8, 8, 16)
MODES = ("astra_aligned", "astra_swapped", "moment_rule", "center_rule")


def case_name(offset):
    return f"{'left' if offset < 0 else 'right'}-{abs(offset):02d}"


def rules(tactile):
    load = -tactile["sum_force_base_N"][2]
    if not .40 <= load <= 1.25:
        raise ValueError(f"Moment rule requires a fully supported lift: measured {load} N")
    estimate = tactile["sum_moment_about_tcp_base_mNm"][1] / load
    return float(np.clip(TOWER[0] - estimate, TOWER[0]-22, TOWER[0]+22))


def reference_path(x):
    """Run a shared arm path with only the release coordinate chosen by the rule."""
    return [waypoint(x, TOWER[1], 82, -27, 22, 1.2),
            waypoint(x, TOWER[1], 58, -16, 22, .8),
            waypoint(x, TOWER[1], 58, -16, 76, .6),
            waypoint(x, TOWER[1], 82, -27, 76, .8)]


def observe_cases(scene, output, lift):
    cases = {}
    for offset in OFFSETS_MM:
        bench = StackBench(scene, offset)
        try:
            bench.run(lift, "lift")
            measured = record_measurement(bench)
            if not measured["airborne"]:
                raise RuntimeError(f"{case_name(offset)} failed the common lift")
            image = output/(case_name(offset) + "-hold.jpg")
            cases[offset] = {"name": case_name(offset), "offset_mm_eval": offset,
                             "image": image.name, "image_sha256": observation_image(bench, image),
                             "observation": summarize(bench, tactile=True), "lift_eval": measured}
        finally:
            bench.close()
    return cases


def replay(scene, offset, lift, path):
    bench = StackBench(scene, offset)
    try:
        bench.run(lift, "lift")
        bench.run(path, "place")
        bench.settle()
        return bench.verdict(), bench.trace
    finally:
        bench.close()


def release_x(path):
    return next((point["xyz_mm"][0] for point in path
                 if point["gap_mm"] >= 60 and point["xyz_mm"][2] < 74), None)


def write_traces(output, traces):
    payload = json.dumps({"points": POINTS, "trials": traces}, separators=(",", ":")).encode()
    (output/"sensor-traces.json.gz").write_bytes(gzip.compress(payload, mtime=0))


def replay_archive(scene, results, output):
    traces = {}
    for row in results["trials"]:
        if "rejection" in row["outcome"]:
            continue
        outcome, traces[row["id"]] = replay(
            scene, row["hidden_cg_mm_eval"], results["lift"], row["place_waypoints"])
        if outcome != row["outcome"]:
            raise RuntimeError(f"{row['id']} replay differed: {outcome}")
        print(row["id"], "reproduced", outcome, flush=True)
    write_traces(output, traces)


def run_models(scene, output, lift, cases, parallel):
    jobs = {}
    for offset, case in cases.items():
        for mode, donor in (("astra_aligned", offset), ("astra_swapped", -offset)):
            observation = {**case["observation"], "paxini": cases[donor]["observation"]["paxini"]}
            key = f"{case['name']}-{mode}"
            jobs[key] = (offset, donor, case, observation)

    def solve(key, job):
        offset, donor, case, observation = job
        checkpoint = output/(key+"-call.json")
        if checkpoint.exists():
            record = json.loads(checkpoint.read_text())
            if record["input"] != observation or record["image_sha256"] != case["image_sha256"]:
                raise ValueError(f"Input changed for archived call {key}")
            return record
        bench = StackBench(scene, offset)
        try:
            bench.run(lift, "lift")
            calls = call_checked(bench, "place", observation, output/case["image"])
        finally:
            bench.close()
        record = {"input": observation, "image": case["image"], "image_sha256": case["image_sha256"],
                  "tactile_donor_offset_mm_eval": donor, "calls": calls}
        atomic(checkpoint, record)
        return record

    results = {}
    with ThreadPoolExecutor(max_workers=parallel) as pool:
        futures = {pool.submit(solve, key, job): key for key, job in jobs.items()}
        for future in as_completed(futures):
            key = futures[future]
            results[key] = future.result()
            info = results[key]["calls"][-1]
            x = release_x(info["response"]["waypoints"])
            print(key, "x_mm", x, "wall_s", info["wall_s"], "rejection", info.get("rejection"), flush=True)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--source", type=Path, help="Original stacking pilot results.json")
    inputs.add_argument("--replay-from", type=Path, help="Reproduce archived ablation results without model calls")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--parallel", type=int, default=4)
    parser.add_argument("--rules-only", action="store_true")
    args = parser.parse_args()
    scene = args.scene.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.replay_from:
        replay_archive(scene, json.loads(args.replay_from.read_text()), output)
        return
    source = json.loads(args.source.read_text())
    lift = source["lift"]["calls"][-1]["response"]["waypoints"]
    cases = observe_cases(scene, output, lift)
    atomic(output/"observations.json", list(cases.values()))
    responses = {} if args.rules_only else run_models(scene, output, lift, cases, args.parallel)
    results = {"model": MODEL, "effort": EFFORT, "lift": lift, "cases": list(cases.values()), "trials": []}
    traces = {}
    for offset, case in cases.items():
        for mode in MODES:
            key = f"{case['name']}-{mode}"
            extra = {}
            if mode.startswith("astra"):
                if args.rules_only:
                    continue
                extra = responses[key]
                final = extra["calls"][-1]
                path = final["response"]["waypoints"]
                if "rejection" in final:
                    verdict = {"stable": False, "rejection": final["rejection"]}
                else:
                    verdict, traces[key] = replay(scene, offset, lift, path)
            else:
                x = rules(case["observation"]["paxini"]) if mode == "moment_rule" else float(TOWER[0])
                path = reference_path(x)
                verdict, traces[key] = replay(scene, offset, lift, path)
            row = {"id": key, "policy": mode, "hidden_cg_mm_eval": offset,
                   "release_tcp_x_mm": release_x(path), "place_waypoints": path, "outcome": verdict, **extra}
            results["trials"].append(row)
            print(key, row["release_tcp_x_mm"], verdict, flush=True)
    atomic(output/"results.json", results)
    write_traces(output, traces)


if __name__ == "__main__":
    main()
