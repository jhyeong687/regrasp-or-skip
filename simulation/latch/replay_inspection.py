"""Replay one logged inspection in the live dashboard without calling a model."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

from dashboard import write_frame
from inspection import FlushBench, HERE, POINT_XY, atomic


def replay(archive, trial_id, scene, output):
    data = json.loads((archive / "results.json").read_text())
    trial = next(row for row in data["trials"] if row["id"] == trial_id and "answer" in row)
    trace = json.loads(gzip.decompress((archive / f"{trial_id}.json.gz").read_bytes()))
    output.mkdir(parents=True, exist_ok=True)
    frames_dir = output / "replay" / trial_id
    frames_dir.mkdir(parents=True, exist_ok=True)
    frames = []
    bench = FlushBench(scene)

    def capture(robot, sample):
        index = len(frames)
        paths = [frames_dir / f"{index:03d}-{kind}.jpg" for kind in ("wide", "cams", "gripper")]
        frame = write_frame(robot, sample, *paths)
        links = {k: str(p.relative_to(output)) for k, p in zip(("wide", "cams", "gripper"), paths)}
        frames.append(dict(frame, image=index + 1, stage="저장된 검사 재생", **links))

    try:
        bench.reset(trial["initial"], trial["variant"])
        capture(bench, bench.samples[-1])
        bench.sink = capture
        for action in trial["actions"]:
            bench.probe(action)
        bench.move(0, bench.observation_dz, 0, .5, "final_wait")
        if bench.samples != trace or bench.transitions != trial["transitions"]:
            raise ValueError(f"State or sensor replay differs from {trial_id}")
    finally:
        bench.close()
    summary = f"{trial_id} · {len(trial['actions'])} probes · {trial['inference_s']:.1f} s inference"
    atomic(output / "replay.json", {"id": trial_id, "policy": trial["policy"], "frames": frames,
                                  "description": summary, "points": POINT_XY,
                                  "trace_sha256": hashlib.sha256((archive / f"{trial_id}.json.gz").read_bytes()).hexdigest()})
    print(f"Replayed {trial_id} in {len(frames)} frames")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("trial")
    parser.add_argument("--scene", type=Path, default=HERE.parent / "scene.xml")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    replay(args.archive, args.trial, args.scene, args.output)
