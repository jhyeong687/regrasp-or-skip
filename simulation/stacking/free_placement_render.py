"""Reproduce archived rule and Astra trials and optionally render both views."""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path

import numpy as np

from bench import HERE, POINTS
from evaluate import atomic
from free_placement import NewObject, ObjectBench, publish_dashboard
from recheck import STOP_S, complete as rule_place, gate, prepare
from render import encoder


def replay(bench, lift, case, row, observation):
    approach, observed = prepare(bench, lift, case)
    if observed != observation:
        raise RuntimeError(f"{row['id']} did not reproduce its handoff")
    if row["policy"] == "rule":
        result = rule_place(bench, approach, gate(observed))
        fields = ("remeasure", "remeasured_offset_mm", "release_tcp_x_mm", "robot_time_s", "outcome")
        if {key: row[key] for key in fields} != result:
            raise RuntimeError(f"{row['id']} did not reproduce its rule outcome")
        return

    if row["remeasure"]:
        bench.run([{**approach, "time_s": STOP_S}], "remeasure")
    if row["waypoints"]:
        bench.run(row["waypoints"], "place")
    active = round(float(bench.data.time), 3)
    if row["waypoints"]:
        bench.settle()
        if bench.verdict() != row["outcome"]:
            raise RuntimeError(f"{row['id']} did not reproduce its placement outcome")
    if active != row["robot_time_s"]:
        raise RuntimeError(f"{row['id']} did not reproduce its action time")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--examples", nargs="+")
    parser.add_argument("--physics-only", action="store_true")
    args = parser.parse_args()
    data = json.loads(args.results.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    replays = {}
    for row in data["trials"]:
        case = NewObject(**data["cases"][row["case"]])
        if args.examples and case.name not in args.examples:
            continue
        trial = row["id"]
        bench = ObjectBench(args.scene, case)
        frames = []
        try:
            with ExitStack() as stack:
                if not args.physics_only:
                    writers = [encoder(stack, args.output/f"{trial}{suffix}.mp4", size)
                               for suffix, size in (("", "1280x480"), ("-cameras", "1280x180"))]

                    def capture(images, sample):
                        frames.append(sample)
                        for writer, image in zip(writers, images):
                            writer.stdin.write(np.ascontiguousarray(image).tobytes())

                    bench.frame_sink = capture
                replay(bench, data["lift"], case, row, data["shared_observations"][case.name])
                if not args.physics_only:
                    for writer in writers:
                        writer.stdin.close()
                        if writer.wait():
                            raise RuntimeError(f"Failed to encode {trial}")
                    replays[trial] = frames
            print(f"Verified {trial}", flush=True)
        finally:
            bench.close()
    if replays:
        atomic(args.output/"replays.json", {"points": POINTS, "trials": replays})
        atomic(args.output/"points.json", POINTS)
        atomic(args.output/"results.json", data)
        publish_dashboard(args.output, data)
        (args.output/"index.html").write_bytes((HERE/"free_placement.html").read_bytes())


if __name__ == "__main__":
    main()
