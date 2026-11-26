"""Choose a contact coefficient on separate poses before the paired experiment."""

import argparse
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from recovery.run import (ACTIONS, Case, RecoveryBench, SHAPES, evaluate_probe, grasp_parameters,
                 probe, retry, rgbd, write_json)
from run import proposals

CALIBRATION_SEED = 261127
COEFFICIENTS = (.060, .075, .090, .105, .120, .135, .150, .175, .200, .225, .250)


def calibrate(scene, output):
    rng = np.random.default_rng(CALIBRATION_SEED)
    result = {"seed": CALIBRATION_SEED, "grid": list(COEFFICIENTS), "shapes": {}, "chosen": {}}
    for shape in SHAPES:
        bench = RecoveryBench(scene, shape)
        try:
            poses = [Case(i, shape, *np.round([rng.uniform(194, 229), rng.uniform(-15, 31),
                                              rng.uniform(-20, 20)], 2).tolist())
                     for i in range(101, 105)]
            candidates = []
            for case in poses:
                bench.reset_trial(case, .8)
                candidates.append(proposals(bench, rgbd(bench)[0])["A"])
            scores = []
            for coefficient in COEFFICIENTS:
                pairs = []
                for case, selected in zip(poses, candidates):
                    pair = {}
                    for action in ACTIONS:
                        bench.reset_trial(case, coefficient)
                        initial = float(bench.data.xpos[bench.object_id, 2] * 1000)
                        nominal, first, gap = grasp_parameters(bench, selected, "low_friction", 1, shape)
                        probe(bench, first, gap, lambda _: ())
                        checked = evaluate_probe(bench, initial)
                        retry(first, gap, action, bench.run, nominal)
                        bench.settle(.60)
                        pair[action] = {"success": bench.score()["success"], "probe": checked}
                    pairs.append(pair)
                strong = sum(p[ACTIONS[1]]["success"] for p in pairs)
                move = sum(p[ACTIONS[0]]["success"] for p in pairs)
                probe_pass = sum(p[ACTIONS[0]]["probe"]["lifted"] for p in pairs)
                data = {"coefficient": coefficient, "stronger_successes": strong,
                        "position_successes": move, "probe_successes": probe_pass, "pairs": pairs}
                scores.append(data)
                print(f"{shape} mu={coefficient:.3f} boost={strong}/4 move={move}/4 probe={probe_pass}/4", flush=True)
            # Prefer a distinguishable intervention, then recovery coverage.
            best = max(scores, key=lambda s: (s["stronger_successes"] - s["position_successes"],
                                              s["stronger_successes"], -s["probe_successes"]))
            result["chosen"][shape] = best["coefficient"]
            result["shapes"][shape] = {"poses": [vars(case) for case in poses], "scores": scores}
            write_json(output, result)
        finally:
            bench.close()
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    calibrate(args.scene, args.out)
