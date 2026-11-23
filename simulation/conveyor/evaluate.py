"""Compare decisions from a force rule and GPT-6.2 Astra on matched simulated items."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
import time

from PIL import Image
import numpy as np

from run import ACTIONS, HERE, PARTS, PITCH, RATES, Conveyor

OFFSETS = (-14, 0, 14)
GAPS = (18, 24)
PICK_S = 2.64
OFFSET_ENUM = {"left": -14, "center": 0, "right": 14}
GAP_ENUM = {"firm": 18, "gentle": 24}
MODEL = "gpt-6.2-astra"
EFFORT = "xhigh"
SCHEMA = {
    "type": "object",
    "properties": {"action": {"type": "string", "enum": list(ACTIONS)},
                   "region": {"type": "string", "enum": list(OFFSET_ENUM)},
                   "grip": {"type": "string", "enum": list(GAP_ENUM)},
                   "evidence": {"type": "string"}},
    "required": ["action", "region", "grip", "evidence"],
    "additionalProperties": False,
}


def evidence_image(bench):
    top = bench._render("arm_top", (560, 340))
    side = bench._render("rgb_scene_right", (560, 340))
    return Image.fromarray(np.column_stack((top, side)))


def phases(rate):
    return {"retry": PICK_S, "wait": PICK_S + .08 / rate, "skip": .135 / rate}


def make_observation(evidence, rate):
    return {
        "belt_rate_mm_s": rate * 1000,
        "remaining_s": round(PITCH / rate, 3),
        "maneuver_time_s": {name: round(duration, 3) for name, duration in phases(rate).items()},
        "first_probe": evidence,
    }


def astra(observation, image_path):
    prompt = """You are the decision controller for a simulated SO-ARM101. Use the supplied readings and one two-camera image. Do not call shell, filesystem, internet or other tools. Return a single structured decision with an evidence sentence.
After the shared center grasp and 10 mm test lift, the arm has put the part down and opened. Choose retry (grasp at the far station now), wait (index 80 mm closer, then grasp), or skip (let the item exit). Choose a reachable grasp region so the contact pads land on a broad opposing section, and a jaw closure. In the LEFT image panel the belt runs horizontally. The left of the part corresponds to region left (-14 mm), the right corresponds to right (+14 mm); center is the nominal fixture coordinate. The RIGHT panel is a second oblique RGB view. Choose firm (18 mm command) or gentle (24 mm command); actual closure depends on contact. The goal is a stable handoff into a tray before the next arrival. The maneuver times include the handoff. Do not choose any pick that cannot fit the window. The shared probe may succeed at lifting a few millimeters even when transfer at the far reach fails. A high elbow torque together with tangential load suggests a close-station pickup may be safer. When skipping, provide arbitrary bounded region and grip fields. The evaluator will execute your chosen grasp in MuJoCo.
"""
    prompt += json.dumps(observation, ensure_ascii=False, indent=2)
    with tempfile.TemporaryDirectory(prefix="belt-astra-") as directory:
        schema = Path(directory)/"decision.json"
        schema.write_text(json.dumps(SCHEMA))
        command = [
            "codex", "exec", "--json", "--ephemeral", "--ignore-user-config", "--skip-git-repo-check",
            "--sandbox", "read-only", "-C", directory, "-m", MODEL,
            "-c", f'model_reasoning_effort="{EFFORT}"', "--output-schema", str(schema),
            "--image", str(image_path.resolve()), "-",
        ]
        start = time.monotonic()
        run = subprocess.run(command, input=prompt, text=True, capture_output=True, timeout=150)
        duration = time.monotonic()-start
    events = [json.loads(line) for line in run.stdout.splitlines() if line.startswith("{")]
    replies = [event["item"]["text"] for event in events if event["type"] == "item.completed"
               and event["item"]["type"] == "agent_message"]
    violations = [event["item"]["type"] for event in events if event["type"] == "item.completed"
                  and event["item"]["type"] not in ("agent_message", "reasoning", "plan")]
    usage = next((event.get("usage") for event in events if event["type"]=="turn.completed"), None)
    if run.returncode or not replies or violations or not usage:
        diagnostics = [event.get("message", event.get("error")) for event in events
                       if event["type"] in ("error", "turn.failed")]
        raise RuntimeError(f"Astra failed: code={run.returncode}, diagnostics={diagnostics}, actions={violations}")
    result = json.loads(replies[-1])
    return {"choice": result, "wall_s": round(duration, 3), "usage": usage, "observation": observation}


def force_rule(observation):
    evidence = observation["first_probe"]
    near = (evidence["elbow_Nm"] >= .98 and max(evidence["pad_shear_N"]) > 1.0)
    desired = "wait" if near else "retry"
    if observation["maneuver_time_s"][desired] > observation["remaining_s"]:
        desired = "skip"
    return {"action": desired, "region": "center", "grip": "firm",
            "evidence": "Elbow ≥ 0.98 N·m and pad shear > 1 N routes to the near station when time permits."}


def read_decision(choice):
    return choice["action"], OFFSET_ENUM[choice["region"]], GAP_ENUM[choice["grip"]]


def score(bench, choice, rate, elapsed_model_s=0.):
    action, offset, gap = read_decision(choice)
    start = bench.data.time
    if action not in ACTIONS:
        raise ValueError("Unexpected action")
    placed = bench.execute(action, offset, gap, rate)
    duration = round(float(bench.data.time-start), 3)
    deadline = PITCH/rate
    return {"choice": choice, "duration_s": duration, "placed_in_tray": placed,
            "on_time_frozen_clock": placed and duration <= deadline,
            "on_time_with_measured_inference": placed and duration + elapsed_model_s <= deadline,
            "first_obj_move_mm_eval": bench.trace[-1]["part_xyz_mm_eval"],
            "peak_torque_Nm": max(sample["torque_Nm"] for sample in bench.trace)}


def save_inputs(scene, output):
    observations = {}
    for part in PARTS:
        bench = Conveyor(scene, part, markings=False)
        try:
            evidence = bench.first_probe()
            image = output / f"{part.name}.jpg"
            evidence_image(bench).save(image, quality=88)
            for speed, rate in RATES.items():
                observations[f"{part.name}-{speed}"] = (make_observation(evidence, rate), image)
        finally:
            bench.close()
    return observations


def run_astra(observations, output, parallelism):
    path = output/"astra-calls.json"
    responses = json.loads(path.read_text()) if path.exists() else {}
    check_responses(observations, responses)
    pending = [key for key in observations if key not in responses]
    if pending:
        with ThreadPoolExecutor(max_workers=parallelism) as pool:
            futures = {key: pool.submit(astra, *observations[key]) for key in pending}
            for key, future in futures.items():
                responses[key] = future.result()
                path.write_text(json.dumps(responses, indent=2)+"\n")
                print(key, responses[key]["choice"], responses[key]["wall_s"], flush=True)
    return responses


def check_responses(observations, responses):
    if any(key not in observations or record["observation"] != observations[key][0]
           for key, record in responses.items()):
        raise ValueError("Saved Astra observations differ from those in this scene.")


def run_evaluation(scene, observations, responses):
    evaluations = []
    for part in PARTS:
        bench = Conveyor(scene, part)
        try:
            for speed, rate in RATES.items():
                key = f"{part.name}-{speed}"
                observation = observations[key][0]
                row = {"item": part.name, "speed": speed, "observation": observation}
                policies = {"force_rule": force_rule(observation),
                            "always_retry": {"action": "retry", "region": "center", "grip": "firm"}}
                if key in responses:
                    policies["astra"] = responses[key]["choice"]
                for name, choice in policies.items():
                    bench.reset()
                    bench.first_probe()
                    wall = responses[key]["wall_s"] if name == "astra" else 0.
                    row[name] = score(bench, choice, rate, wall)
                options = []
                # A bounded physical reference enumerates all available contact regions/gaps for timely picks.
                for action, region, grip in itertools.product(("retry", "wait"), OFFSET_ENUM, GAP_ENUM):
                    if phases(rate)[action] > PITCH/rate:
                        continue
                    bench.reset()
                    bench.first_probe()
                    scored = score(bench, {"action": action, "region": region, "grip": grip}, rate)
                    if scored["on_time_frozen_clock"]:
                        options.append(scored["choice"])
                row["feasible_picks_eval"] = options
                evaluations.append(row)
        finally:
            bench.close()
    return evaluations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--output", type=Path, required=True)
    backend = parser.add_mutually_exclusive_group()
    backend.add_argument("--astra", action="store_true", help="Call GPT-6.2 Astra via the logged-in Codex CLI")
    backend.add_argument("--responses", type=Path, help="Execute saved choices without calling the model")
    parser.add_argument("--parallelism", type=int, default=2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scene = args.robot_scene.resolve()
    observations = save_inputs(scene, args.output)
    responses = run_astra(observations, args.output, args.parallelism) if args.astra else {}
    if args.responses:
        responses = json.loads(args.responses.read_text())
        check_responses(observations, responses)
    results = run_evaluation(scene, observations, responses)
    path = args.output/"results.json"
    path.write_text(json.dumps({"model": MODEL, "effort": EFFORT, "trials": results}, indent=2)+"\n")


if __name__ == "__main__":
    main()
