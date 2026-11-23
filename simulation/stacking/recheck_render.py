"""Render representative remeasurement trials from the archived decisions."""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path

import numpy as np

from bench import HERE, POINTS
from recheck import Case, RailBench, complete, prepare
from render import encoder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--examples", nargs="+", default=["rail-a", "rail-c"])
    args = parser.parse_args()
    result = json.loads(args.results.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    replays = {}
    for row in result["trials"]:
        if row["case"] not in args.examples:
            continue
        record = result["observations"][row["case"]]
        bench = RailBench(args.scene, Case(**record["case"]))
        frames = []
        try:
            with ExitStack() as stack:
                streams = [encoder(stack, args.output/f"{row['id']}{suffix}.mp4", size)
                           for suffix, size in (("", "1280x480"), ("-cameras", "1280x180"))]

                def capture(images, sample):
                    frames.append(sample)
                    for stream, image in zip(streams, images):
                        stream.stdin.write(np.ascontiguousarray(image).tobytes())

                bench.frame_sink = capture
                approach, observation = prepare(bench, result["lift"], Case(**record["case"]))
                if observation != record["observation"]:
                    raise RuntimeError(f"Observation drift in {row['id']}")
                replay = complete(bench, approach, row["remeasure"])
                if replay["outcome"] != row["outcome"] or replay["robot_time_s"] != row["robot_time_s"]:
                    raise RuntimeError(f"Trial did not reproduce {row['id']}")
                for stream in streams:
                    stream.stdin.close()
                    if stream.wait():
                        raise RuntimeError(f"Could not encode {row['id']}")
            replays[row["id"]] = frames
            print(row["id"], "ready", flush=True)
        finally:
            bench.close()
    path = args.output/"replays.json"
    path.write_text(json.dumps({"points":POINTS,"trials":replays},separators=(",", ":")))


if __name__ == "__main__":
    main()
