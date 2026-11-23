"""Find exact first-probe ambiguities and compare paired policy outcomes."""

import argparse
from collections import Counter, defaultdict
import csv
import gzip
from hashlib import sha256
import json
from pathlib import Path

from evaluate_recovery_pilot import records, write_json, RETRIES


def sensor_signature(data, row):
    image = sha256((data / row["image"]).read_bytes()).hexdigest()
    features = json.dumps({k: row[k] for k in ("vision", "arm", "tactile")}, sort_keys=True)
    return image, sha256(features.encode()).hexdigest()


def paired_comparison(by_policy, experimental, control):
    if experimental not in by_policy or control not in by_policy:
        return None
    a, b = by_policy[experimental], by_policy[control]
    if a.keys() != b.keys():
        raise ValueError(f"Mismatched case IDs for {experimental} and {control}")
    matches = [(a[key]["useful_pick"] == "True", b[key]["useful_pick"] == "True") for key in a]
    counts = Counter(matches)
    return {"experimental_only": counts[True, False], "control_only": counts[False, True],
            "both": counts[True, True], "neither": counts[False, False]}


def diagnose(data, results):
    outcomes = {row["id"]: row for row in records(data / "outcomes.jsonl")}
    observations = {row["id"]: row for row in records(data / "observations.jsonl")
                    if row["split"] == "evaluation"}
    groups = defaultdict(list)
    for key, row in observations.items():
        groups[sensor_signature(data, row)].append(key)
    conflicts = []
    for (image_hash, feature_hash), ids in groups.items():
        labels = {outcomes[key]["label"] for key in ids}
        if len(labels) > 1:
            conflicts.append({"image_sha256": image_hash, "feature_sha256": feature_hash,
                              "episodes": [{"id": key, "label": outcomes[key]["label"],
                                            "successful_retries": outcomes[key]["successful_retries"]} for key in ids]})
    identical_input_ceiling = sum(max(Counter(outcomes[key]["label"] for key in group).values())
                                  for group in groups.values())
    by_policy = defaultdict(dict)
    with (results / "policy-rollouts.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            by_policy[row["policy"]][row["id"]] = row
    return {"evaluation_episodes": len(observations), "exact_conflicts": conflicts,
            "deterministic_3_way_ceiling_count": identical_input_ceiling,
            "retry_success_counts_among_first_failures": {
                action: sum(row["retries"][action]["success"] for key, row in outcomes.items()
                            if key in observations and row["label"] != "continue") for action in RETRIES},
            "paired_useful_picks": {name: value for (name, experimental, control) in (
                ("tactile_vs_vision_arm", "tactile_vlm", "vision_arm_vlm"),
                ("tactile_vs_shuffled", "tactile_vlm", "shuffled_tactile_vlm"),
                ("adaptive_vs_fixed_firmer", "adaptive_rule", "fixed_firmer_retry"),
            ) if (value := paired_comparison(by_policy, experimental, control)) is not None}}


def plot(data, target):
    import matplotlib.pyplot as plt
    import numpy as np

    ids = ("eva-05-02", "eva-05-03")
    with gzip.open(data / "raw-traces.jsonl.gz", "rt") as stream:
        rows = {row["id"]: row["traces"] for line in stream if (row := json.loads(line))["id"] in ids}
    figure, axes = plt.subplots(1, 3, figsize=(11.7, 3.5), layout="constrained")
    for key, title, color in zip(ids, ("Regrasp succeeds", "All retries fail"), ("#6144a8", "#197879"), strict=True):
        first = rows[key]["first"]
        retry = rows[key]["regrasp_firmer"]
        force = np.asarray([sample["net_force_N"] for sample in first])
        time = [sample["time_s"] - 6.2 for sample in first]
        axes[0].plot(time, force[:, :, 2].mean(axis=1), color=color, lw=2, label=title)
        axes[1].plot(time, np.linalg.norm(force[:, :, :2], axis=2).mean(axis=1), color=color, lw=2)
        time = [sample["time_s"] - (retry[0]["time_s"] - .04) for sample in retry]
        axes[2].plot(time, [sample["object_z_mm_truth"] for sample in retry], color=color, lw=2)
    axes[0].legend(loc="lower right", frameon=False, fontsize=8)
    for ax, title, y, x in zip(axes, ("First probe · squeeze", "First probe · shear", "Firmer regrasp · actual lift"),
                               ("Mean Fz per pad (N)", "Mean shear per pad (N)", "Object rise (mm)"),
                               ("Time after probe start (s)", "Time after probe start (s)", "Time after retry start (s)"), strict=True):
        ax.set(title=title, xlabel=x, ylabel=y)
        ax.grid(alpha=.19)
        ax.spines[["top", "right"]].set_visible(False)
    for ax in axes[:2]:
        ax.axvline(1.28, color="#88909a", lw=1, ls=":")
    axes[2].axhline(8, color="#88909a", ls="--", lw=1, label="8 mm criterion")
    axes[2].legend(loc="upper left", frameon=False, fontsize=8)
    figure.savefig(target, dpi=185)
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--figure", type=Path, help="Optional comparison PNG (requires matplotlib)")
    args = parser.parse_args()
    write_json(args.output, diagnose(args.data, args.results))
    if args.figure:
        plot(args.data, args.figure)
