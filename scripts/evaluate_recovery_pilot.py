"""Score bounded recovery decisions and run paired local VLM input ablations."""

import argparse
from collections import Counter, defaultdict
import csv
from importlib.metadata import version
import json
from pathlib import Path
import random
from statistics import mean, median
from time import perf_counter


RETRIES = ("regrasp_in", "regrasp_out", "regrasp_firmer")
ACTIONS = ("continue", *RETRIES, "skip")
FIRST_PROBE_S = 3.0
SET_DOWN_S = 1.76
RETRY_S = SET_DOWN_S + .72 + FIRST_PROBE_S
MODES = ("precontact_vlm", "vision_arm_vlm", "tactile_vlm", "shuffled_tactile_vlm")
PROMPT = """You control a simulated SO-ARM101 at a fixed pick station. The task is to retain useful picks while avoiding unnecessary attempts. Objects can have identical exteriors with different weight or traction. Your action is one of these five exact strings:
continue: accept the first grasp if the object is held above the work surface.
regrasp_in: reopen and move the grasp 18 mm toward the robot base, then repeat the low lift once.
regrasp_out: reopen and move 18 mm away from the base, then repeat once.
regrasp_firmer: repeat at the same position with more squeeze, within the preset simulated limit.
skip: stop after this item and move to the next.

The table frame's +X points away from the base. A tactile contact_x_mm value represents the contact position relative to the corresponding pad's center, projected onto this axis. Tangential force is listed as shear. Lack of tactile contact alone does not distinguish a misplaced grasp from a load or surface that will defeat a better grasp. The image's pixel rises are noisy under occlusion. Compare the object against the same table and gripper in both views. No second retry is available. Select an action based on the observations.
Return one JSON object with exactly these keys: action, evidence. The action must contain only one of the five action strings. Keep the evidence to at most 12 words.
"""


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def write_json(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def mean_available(values):
    values = [value for value in values if value is not None]
    return mean(values) if values else 0.0


def recovery_class(action):
    return "regrasp" if action in RETRIES else action


def received_action(response):
    start, stop = response.find("{"), response.rfind("}") + 1
    try:
        record = json.loads(response[start:stop])
        action, evidence = record["action"], record["evidence"]
    except (ValueError, KeyError, TypeError):
        return "invalid", None
    return (action, evidence) if action in ACTIONS and isinstance(evidence, str) else ("invalid", None)


def calibrate(observations, labels):
    # Only the calibration split is used to choose the vision rule's cutoff and reference pixel.
    calibration = [row for row in observations if row["split"] == "calibration"]
    centered = [row for row in calibration if labels[row["id"]]["case_truth"]["offset_mm"] == 0]
    anchor = [median([row["vision"]["shell_center_before_px_lr"][side][0] for row in centered])
              for side in range(2)]
    measured = [(mean_available(row["vision"]["shell_rise_px_lr"]),
                 labels[row["id"]]["label"] == "continue") for row in calibration]
    measured_values = sorted({value for value, _ in measured})
    candidates = [measured_values[0] - 1, measured_values[-1] + 1]
    candidates += [(lo + hi) / 2 for lo, hi in zip(measured_values, measured_values[1:])]
    threshold = max(candidates, key=lambda value: (sum((x >= value) == y for x, y in measured),
                                                   min(abs(x - value) for x in measured_values)))
    return {"visual_lift_px": threshold, "center_px_lr": anchor,
            "edge_offset_px": 15.0, "contact_N": .15, "elbow_skip_Nm": 1.05}


def observed_lift(row, thresholds):
    return mean_available(row["vision"]["shell_rise_px_lr"]) >= thresholds["visual_lift_px"]


def rules(row, thresholds):
    vision = row["vision"]
    tactile = row["tactile"]["end_hold"]
    elbow = row["arm"]["end_hold"]["elbow_Nm"]
    visible_lift = observed_lift(row, thresholds)
    stable_bilateral = min(tactile["normal_N_lr"]) >= thresholds["contact_N"]
    x0, x1 = thresholds["center_px_lr"]
    left, right = vision["shell_center_before_px_lr"]
    pixel_offset = mean_available((None if left is None else x0 - left[0],
                                   None if right is None else right[0] - x1))
    toward = "regrasp_out" if pixel_offset > 0 else "regrasp_in"
    vision_action = toward if abs(pixel_offset) >= thresholds["edge_offset_px"] else "regrasp_firmer"
    threshold_action = "regrasp_firmer" if stable_bilateral else "skip"
    tactile_action = "skip" if elbow >= thresholds["elbow_skip_Nm"] else (
        toward if abs(pixel_offset) >= thresholds["edge_offset_px"] else threshold_action)
    yield "vision_arm_rule", "continue" if visible_lift else vision_action
    yield "normal_force_rule", "continue" if visible_lift else threshold_action
    yield "fixed_firmer_retry", "continue" if visible_lift else "regrasp_firmer"
    yield "adaptive_rule", "continue" if visible_lift else tactile_action


def apply_guard(rollout, observation, thresholds, fallback):
    """Offline harness ablation that uses the same observations as the rule baseline."""
    action = rollout["action"]
    reason = None
    if action == "invalid":
        reason = "invalid_response"
    elif action == "continue" and not observed_lift(observation, thresholds):
        reason = "continue_without_observed_lift"
    elif action in RETRIES and observation["arm"]["end_hold"]["elbow_Nm"] >= thresholds["elbow_skip_Nm"]:
        reason = "retry_under_elevated_torque"
    return {**rollout, "policy": "guarded_tactile_vlm", "action": fallback if reason else action,
            "raw_action": action, "fallback_reason": reason}


def shuffle_map(rows, seed):
    by_scene = defaultdict(list)
    rng = random.Random(seed)
    for row in rows:
        by_scene[row["scene"]].append(row["id"])
    swaps = {}
    for group in by_scene.values():
        if len(group) < 2:
            raise ValueError("The shuffled control needs at least two trials per scene")
        rng.shuffle(group)
        swaps.update(zip(group, group[1:] + group[:1], strict=True))
    return swaps


def model_inputs(row, mode, tactile):
    if mode == "precontact_vlm":
        # Hypothetical first decision with only the pre-contact view. It has the same action vocabulary.
        note = ("This input has only the pre-contact scene. Predict the appropriate decision for the initial "
                "grasp and test lift at the nominal position. Do not assume motion has been observed.")
        return row["prior_image"], {}, note
    payload = {"arm": row["arm"], "rgb_measurements": row["vision"]}
    if mode in {"tactile_vlm", "shuffled_tactile_vlm"}:
        payload["paxini"] = tactile
    return row["image"], payload, "The first pinch and test lift have already run."


def rows_for_models(data, output, rows, model_path, seed, limit):
    from mlx_vlm import generate, load
    from mlx_vlm.prompt_utils import apply_chat_template

    t0 = perf_counter()
    model, processor = load(str(model_path))
    by_id = {row["id"]: row for row in rows}
    shuffle = shuffle_map(rows, seed)
    is_snapshot = model_path.is_dir() and model_path.parent.name == "snapshots"
    name = model_path.parent.parent.name.removeprefix("models--").replace("--", "/") if is_snapshot else str(model_path)
    info = {"model": name, "checkpoint": model_path.name if is_snapshot else None,
            "mlx_vlm_version": version("mlx-vlm"), "mlx_version": version("mlx"),
            "temperature": 0, "max_tokens": 96, "shuffle_seed": seed,
            "load_seconds": round(perf_counter() - t0, 3), "modes": MODES, "prompt": PROMPT,
            "shuffled_tactile_from": shuffle}
    write_json(output / "model-run.json", info)
    with (output / "model-responses.jsonl").open("w") as raw:
        for row in rows[:limit]:
            for mode in MODES:
                donor = shuffle[row["id"]] if mode == "shuffled_tactile_vlm" else row["id"]
                image, payload, note = model_inputs(row, mode, by_id[donor]["tactile"])
                prompt = apply_chat_template(processor, model.config,
                                             PROMPT + "\n" + note + "\n" + json.dumps(payload), num_images=1)
                start = perf_counter()
                response = generate(model, processor, prompt, image=str(data / image),
                                    max_tokens=info["max_tokens"], temperature=0, verbose=False)
                action, evidence = received_action(response.text)
                record = {"id": row["id"], "policy": mode, "action": action, "evidence": evidence,
                          "image": image, "payload": payload, "response": response.text,
                          "model_seconds": round(perf_counter() - start, 3)}
                raw.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
                raw.flush()
                print(row["id"], mode, action, f'{record["model_seconds"]:.2f}s', flush=True)


def score(rollout, truth):
    action = rollout["action"]
    is_retry = action in RETRIES
    label = truth["label"]
    branch = truth["retries"].get(action) if is_retry else (truth["first"] if action == "continue" else None)
    pick = bool(branch and branch["success"])
    elapsed = FIRST_PROBE_S + (RETRY_S if is_retry else (0 if pick else SET_DOWN_S))
    return {"id": rollout["id"], "policy": rollout["policy"], "action": action, "label": label,
            "class_correct": recovery_class(action) == label, "useful_pick": pick,
            "false_skip": action == "skip" and label != "skip", "failed_retry": is_retry and not pick,
            "false_continue": action == "continue" and not pick, "retried": is_retry,
            "avoidable_retry": is_retry and label == "skip", "invalid": action == "invalid",
            "lost_lift": bool(branch and branch["lost_lift"]), "raw_action": rollout.get("raw_action"),
            "fallback_reason": rollout.get("fallback_reason"),
            "modeled_probe_seconds": round(elapsed, 2), "model_seconds": rollout.get("model_seconds")}


def summarize_policy(trials):
    return {"n": len(trials), **{name: sum(row[name] for row in trials) for name in (
        "class_correct", "useful_pick", "false_skip", "failed_retry", "false_continue", "retried",
        "avoidable_retry", "invalid", "lost_lift")},
        "fallbacks": sum(row["fallback_reason"] is not None for row in trials),
        "mean_probe_seconds": round(mean(row["modeled_probe_seconds"] for row in trials), 3),
        "median_model_seconds": (round(median(row["model_seconds"] for row in trials), 3)
                                 if all(row["model_seconds"] is not None for row in trials) else None)}


def evaluate(data, output, model, responses, seed, limit):
    output.mkdir(parents=True, exist_ok=True)
    observations = records(data / "observations.jsonl")
    rows = [row for row in observations if row["split"] == "evaluation"]
    if not rows or any(not row.get("image") for row in rows):
        raise ValueError("Record the evaluation trials with RGB frames before evaluating policies")
    if model:
        # No outcomes are read until all calls to the model are complete.
        rows_for_models(data, output, rows, model, seed, limit)
        responses = output / "model-responses.jsonl"
    labels = {row["id"]: row for row in records(data / "outcomes.jsonl")}
    thresholds = calibrate(observations, labels)
    rows = rows[:limit]
    rollouts = [dict(id=row["id"], policy=policy, action=action)
                for row in rows for policy, action in rules(row, thresholds)]
    if responses:
        by_id = {row["id"]: row for row in rows}
        fallback = {row["id"]: row["action"] for row in rollouts if row["policy"] == "adaptive_rule"}
        model_rows = [row for row in records(responses) if row["id"] in by_id]
        actual = {(row["id"], row["policy"]) for row in model_rows}
        expected = {(key, mode) for key in by_id for mode in MODES}
        if actual != expected or len(model_rows) != len(expected):
            raise ValueError("Model response log must contain one response per episode and input mode")
        rollouts.extend(model_rows)
        rollouts.extend(apply_guard(row, by_id[row["id"]], thresholds, fallback[row["id"]])
                        for row in model_rows if row["policy"] == "tactile_vlm")
    scored = [score(row, labels[row["id"]]) for row in rollouts]
    with (output / "policy-rollouts.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=scored[0].keys())
        writer.writeheader()
        writer.writerows(scored)
    by_policy = defaultdict(list)
    for row in scored:
        by_policy[row["policy"]].append(row)
    result = {"evaluation_episodes": len(rows), "calibration_episodes": len(observations) - len(
        [row for row in observations if row["split"] == "evaluation"]),
        "labels": dict(Counter(labels[row["id"]]["label"] for row in rows)), "rule_thresholds": thresholds,
        "policies": {policy: summarize_policy(trials) for policy, trials in by_policy.items()}}
    write_json(output / "summary.json", result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--model", type=Path, help="Optional MLX VLM checkpoint path or Hugging Face ID")
    source.add_argument("--responses", type=Path, help="Score a previously saved model response log")
    parser.add_argument("--seed", type=int, default=71, help="Seed for the matched-scene shuffle control")
    parser.add_argument("--limit", type=int, help="Limit the number of evaluation trials for a smoke test")
    args = parser.parse_args()
    evaluate(args.data, args.output, args.model, args.responses, args.seed, args.limit)
