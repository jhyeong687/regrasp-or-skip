"""Export workbench replays for selected evaluated conveyor decisions."""

import argparse
import json
from pathlib import Path
import subprocess

import numpy as np

from evaluate import read_decision
from run import Conveyor, HERE, PARTS, RATES

DEFAULT_CLIPS = (
    "stepped-loaded:slow:astra",
    "stepped-loaded:fast:astra",
    "elbow:fast:force_rule",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--clips", nargs="+", default=DEFAULT_CLIPS)
    args = parser.parse_args()
    rows = json.loads(args.results.read_text())["trials"]
    args.output.mkdir(parents=True, exist_ok=True)
    for clip in args.clips:
        name, speed, policy = clip.split(":")
        part = next(part for part in PARTS if part.name==name)
        row = next(row for row in rows if row["item"]==name and row["speed"]==speed)
        choice = row[policy]["choice"]
        path = args.output/f"{name}-{speed}-{policy}.mp4"
        command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
                   "-video_size", "1280x480", "-framerate", "12.5", "-i", "-",
                   "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "19", "-movflags", "+faststart", str(path)]
        bench = Conveyor(args.robot_scene.resolve(), part)
        try:
            with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
                def write_frames(images):
                    encoder.stdin.write(np.ascontiguousarray(images[0]).tobytes())

                bench.frame_sink = write_frames
                bench.first_probe()
                placed = bench.execute(*read_decision(choice), RATES[speed])
                encoder.stdin.close()
                if encoder.wait() != 0 or placed != row[policy]["placed_in_tray"]:
                    raise RuntimeError("Replay did not match scored trial")
        finally:
            bench.close()


if __name__ == "__main__":
    main()
