"""Exploratory replay: ask Astra to confirm a placement from the hover position."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path

from bench import HERE, TOWER, StackBench
from evaluate import (COMMON_PROMPT, atomic, check_choice, model_call, observation_image, summarize)

HOVER_PROMPT = """You are already holding the part above the destination at roughly z82 mm. The previous plan targeted the documented x, then lowered to z58 mm. You may preserve its lateral coordinate or revise it. From the current pose issue a waypoint above the destination, then lower, open, and withdraw. Do not return to the pick mat. A downward load of magnitude N and a y moment in mNm imply x offset relative to the TCP in mm if the part is steady and unsupported. The latest evidence supplied here is the available data. Give the actual XYZ values to execute."""


def prepare(scene, lift, approach, hover):
    bench = StackBench(scene, -16)
    try:
        bench.run(lift, "lift")
        bench.run(approach, "transport")
        bench.run([{**hover, "time_s": .44}], "hover")
        return bench
    except Exception:
        bench.close()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = json.loads(args.results.read_text())
    scene = args.scene.resolve()
    lift = results["lift"]
    row = next(row for row in results["trials"] if row["id"] == "left-16-astra_aligned")
    path = row["place_waypoints"]
    stop = next(i for i,w in enumerate(path) if w["xyz_mm"][1] == TOWER[1] and w["xyz_mm"][2] >= 78)
    approach, hover = path[:stop+1], path[stop]
    image = args.output/"hover.jpg"
    bench = prepare(scene, lift, approach, hover)
    try:
        checksum = observation_image(bench, image)
        common_obs = summarize(bench, tactile=True)
    finally:
        bench.close()
    common_obs.update(planned_x_before_transport_mm=row["release_tcp_x_mm"])

    def evaluate(label):
        sensor = row["input"]["paxini"] if label == "earlier" else common_obs["paxini"]
        observation = {**common_obs, "paxini": sensor}
        checkpoint = args.output/f"hover-{label}-call.json"
        if checkpoint.exists():
            record = json.loads(checkpoint.read_text())
            if record["observation"] != observation or record["image_sha256"] != checksum:
                raise ValueError("Archived hover inputs do not match")
        else:
            prompt = COMMON_PROMPT + "\n" + HOVER_PROMPT + "\n" + json.dumps(observation, indent=2)
            record = {"observation": observation, "image_sha256": checksum, **model_call(prompt, image)}
            atomic(checkpoint, record)
        bench = prepare(scene, lift, approach, hover)
        try:
            answer = record["response"]
            check_choice(bench, answer, "place")
            bench.run(answer["waypoints"], "place")
            bench.settle()
            return {"tactile_window": label, **record, "outcome": bench.verdict()}
        except ValueError as error:
            return {"tactile_window": label, **record, "outcome": {"stable": False, "rejection": str(error)}}
        finally:
            bench.close()

    with ThreadPoolExecutor(max_workers=2) as pool:
        trials = list(pool.map(evaluate, ("earlier", "hover")))
    atomic(args.output/"hover-model-results.json", {"case": row["id"], "trials": trials})
    for trial in trials:
        print(trial["tactile_window"],trial["response"],trial["outcome"],flush=True)


if __name__ == "__main__":
    main()
