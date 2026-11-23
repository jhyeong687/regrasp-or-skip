"""Audit recorded Astra calls and calculate paired placement results."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import mean, median


def trim(value):
    return round(value, 3)


def describe(rows):
    return {
        "n": len(rows),
        "successes": sum(row["outcome"]["stable"] for row in rows),
        "remeasures": sum(row["remeasure"] for row in rows),
        "no_valid_action": sum(row["outcome"].get("no_valid_action", False) for row in rows),
        "mean_motion_s": trim(mean(row["robot_time_s"] for row in rows)),
        "mean_inference_s": trim(mean(row["inference_wall_s"] for row in rows)),
        "mean_motion_and_inference_s": trim(mean(row["motion_plus_inference_s"] for row in rows)),
        "median_motion_and_inference_s": trim(median(row["motion_plus_inference_s"] for row in rows)),
        "mean_runner_wall_s": trim(mean(row["suite_wall_s"] for row in rows)),
    }


def score(pairs):
    base = [trials["rule"] for trials in pairs.values()]
    agent = [trials["astra"] for trials in pairs.values()]
    wins = sum(a["outcome"]["stable"] and not b["outcome"]["stable"] for b, a in zip(base, agent))
    losses = sum(b["outcome"]["stable"] and not a["outcome"]["stable"] for b, a in zip(base, agent))
    discordant = wins + losses
    p_value = min(1.0, 2 * sum(math.comb(discordant, i) for i in range(min(wins, losses)+1))/2**discordant)
    return {
        "rule": describe(base),
        "astra": describe(agent),
        "astra_only": wins,
        "rule_only": losses,
        "both": sum(b["outcome"]["stable"] and a["outcome"]["stable"] for b, a in zip(base, agent)),
        "neither": sum(not b["outcome"]["stable"] and not a["outcome"]["stable"] for b, a in zip(base, agent)),
        "exact_mcnemar_p": round(p_value, 6),
    }


def audit(path):
    data = json.loads(path.read_text())
    pairs = {}
    for row in data["trials"]:
        pairs.setdefault(row["case"], {})[row["policy"]] = row
    pairs = {name: rows for name, rows in pairs.items() if len(rows)==2}
    if not pairs:
        raise ValueError("No matched trials have completed")

    seen_threads, seen_turns, receipts, evaluations = set(), set(), [], []
    for name, rows in pairs.items():
        calls = data["model_receipts"][name]
        actual = rows["astra"]
        if actual["model_calls"] != len(calls) or actual["inference_wall_s"] != trim(sum(c["wall_s"] for c in calls)):
            raise ValueError(f"Call accounting was inconsistent for {name}")
        for call in calls:
            image = path.parent/call["image"]
            if hashlib.sha256(image.read_bytes()).hexdigest() != call["image_sha256"]:
                raise ValueError(f"Image integrity check failed for {image}")
            if any(term in call["prompt"] for term in
                   ("trigger_threshold_mm", "hidden_ballast_mm", "gate_recommendation")):
                raise ValueError(f"Policy privileged fields appeared in {name}")
            thread, turn = call.get("thread_id"), call.get("turn_id")
            if (thread and thread in seen_threads) or (turn and turn in seen_turns):
                raise ValueError(f"Reused thread or turn for {name}")
            if thread:
                seen_threads.add(thread)
            if turn:
                seen_turns.add(turn)
            if "error" not in call:
                runtime = call["runtime"]
                if (runtime["model"],runtime["reasoningEffort"]) != (data["requested_model"], data["requested_effort"]):
                    raise ValueError(f"Model configuration mismatch in {name}")
                if call["turn_status"] != "completed":
                    raise ValueError(f"Incomplete turn accepted in {name}")
                events = call["events"]
                messages = [event["params"] for event in events if event["method"] == "item/completed"]
                if json.loads(messages[-1]["item"]["text"]) != call["response"]:
                    raise ValueError(f"Recorded response mismatch in {name}")
                if messages[-1]["threadId"] != thread or messages[-1]["turnId"] != turn:
                    raise ValueError(f"Session identifiers did not match in {name}")
                if "usage" in call:
                    usage_events = [event["params"] for event in events if event["method"] == "thread/tokenUsage/updated"]
                    last = usage_events[-1]
                    if last["tokenUsage"] != call["usage"] or last["threadId"] != thread or last["turnId"] != turn:
                        raise ValueError(f"Usage records did not match in {name}")
                evaluations.append(call)
            receipts.append(call)
        accepted = [c for c in calls if "rejection" not in c]
        if accepted and (accepted[0]["response"]["decision"]=="remeasure") != actual["remeasure"]:
            raise ValueError(f"Executed remeasurement diverged from Astra's first accepted decision in {name}")
        if actual["waypoints"] and accepted[-1]["response"]["waypoints"] != actual["waypoints"]:
            raise ValueError(f"Executed waypoints diverged from Astra's plan for {name}")

    timings = sorted(c["wall_s"] for c in evaluations)
    usage = [c["usage"]["total"] for c in evaluations if "usage" in c]
    totals = {key: sum(u.get(key, 0) for u in usage) for key in
              ("inputTokens", "cachedInputTokens", "cacheWriteInputTokens", "outputTokens", "reasoningOutputTokens")}
    latency = ({"mean": trim(mean(timings)), "median": median(timings),
                "min": min(timings), "max": max(timings),
                "p95_nearest_rank": timings[math.ceil(.95*len(timings))-1]} if timings else {})
    groups = {}
    for name, rows in pairs.items():
        shape = data["cases"][name]["shape"]
        groups.setdefault(shape, {})[name] = rows
    return {
        "budget_wall_s": data["budget_wall_s"],
        "elapsed_wall_s": data.get("elapsed_wall_s"),
        "started_utc": data["started_utc"], "finished_utc": data.get("finished_utc"),
        "all_matched": score(pairs),
        "first_block": score({name: rows for name, rows in pairs.items() if name.startswith("b01-")}),
        "by_shape": {shape: score(rows) for shape, rows in groups.items()},
        "inference": {"attempted": len(receipts), "completed": len(evaluations),
                      "usage_reported": len(usage), "distinct_threads": len(seen_threads),
                      "distinct_turns": len(seen_turns),
                      "handoff_calls": sum(c["stage"]=="handoff" for c in receipts),
                      "after_reread_calls": sum(c["stage"]=="after-reread" for c in receipts),
                      "ik_or_schema_retries": sum(c["attempt"]>1 for c in receipts),
                      "invalid_calls": sum("rejection" in c for c in receipts),
                      "runtime_errors": sum("error" in c for c in receipts),
                      "nonzero_reasoning_turns": sum(u.get("reasoningOutputTokens",0)>0 for u in usage),
                      "usage_totals": totals, "completed_call_wall_s": latency},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    text = json.dumps(audit(args.results), indent=2)
    if args.output:
        args.output.write_text(text+"\n")
    else:
        print(text)


if __name__ == "__main__":
    main()
