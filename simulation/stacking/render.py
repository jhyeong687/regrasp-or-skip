"""Replay archived Astra waypoints and export synchronized R7 video and sensor traces."""

import argparse
from contextlib import ExitStack
import gzip
import json
from pathlib import Path
import subprocess

import numpy as np

from bench import HERE, POINTS, StackBench


def encoder(stack, path, size):
    return stack.enter_context(subprocess.Popen([
        "ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", size,
        "-framerate", "12.5", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-crf", "19", "-movflags", "+faststart", str(path),
    ], stdin=subprocess.PIPE))


def simulate(scene, row, lift, output, render=True):
    bench = StackBench(scene, row["hidden_cg_mm_eval"])
    frames = []
    try:
        with ExitStack() as resources:
            if render:
                recorders = [encoder(resources, output/(row["id"]+suffix+".mp4"), size)
                             for suffix, size in (("", "1280x480"), ("-cameras", "1280x180"))]
                def capture(images, sample):
                    frames.append(sample)
                    for process, image in zip(recorders, images):
                        process.stdin.write(np.ascontiguousarray(image).tobytes())
                bench.frame_sink = capture
            bench.run(lift, "lift")
            bench.run(row["place_waypoints"], "place")
            bench.settle()
            for process in recorders if render else ():
                process.stdin.close()
                if process.wait():
                    raise RuntimeError("FFmpeg could not write a replay")
        return bench.verdict(), frames, bench.trace
    finally:
        bench.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--physics-only", action="store_true")
    args = parser.parse_args()
    results = json.loads(args.results.read_text())
    lift = results["lift"]["calls"][-1]["response"]["waypoints"]
    args.output.mkdir(parents=True, exist_ok=True)
    videos, traces = {}, {}
    for row in results["trials"]:
        if "place_waypoints" not in row:
            continue
        verdict, frames, trace = simulate(args.scene.resolve(), row, lift, args.output, not args.physics_only)
        if verdict != row["outcome"]:
            raise RuntimeError(f"{row['id']} did not reproduce its archived outcome: {verdict}")
        videos[row["id"]] = frames
        traces[row["id"]] = trace
        print(row["id"], verdict, flush=True)
    if not args.physics_only:
        (args.output/"replays.json").write_text(json.dumps({"points": POINTS, "trials": videos}, separators=(",", ":")))
    payload = json.dumps({"points": POINTS, "trials": traces}, separators=(",", ":")).encode()
    (args.output/"sensor-traces.json.gz").write_bytes(gzip.compress(payload, mtime=0))

    tactile_trials = [row for row in results["trials"] if row["available_paxini"] and "place_waypoints" in row]
    swaps = []
    for row in tactile_trials:
        for donor in tactile_trials:
            if donor["hidden_cg_mm_eval"] == row["hidden_cg_mm_eval"]:
                continue
            counterfactual = {**row, "place_waypoints": donor["place_waypoints"]}
            verdict, _, _ = simulate(args.scene.resolve(), counterfactual, lift, args.output, render=False)
            swaps.append({"physical_trial": row["id"], "placement_donor": donor["id"], "outcome": verdict})
    (args.output/"swapped-placements.json").write_text(json.dumps(swaps, indent=2)+"\n")


if __name__ == "__main__":
    main()
