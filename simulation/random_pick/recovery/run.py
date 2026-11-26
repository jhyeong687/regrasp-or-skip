"""Paired contact-recovery trials from the RGB-D pick pilot."""

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
from statistics import median
import sys
import time

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from run import (
    Case, EFFORT, MODEL, PickBench, SHAPES, infer,
    TRAY, proposals, rgbd, waypoint, write_json,
)
from sim import COMPRESSION_POSITIVE, CONTACT_NORMAL_N, POINTS
from recovery import vision_alignment as visual

ACTIONS = ("position_correction", "grip_reinforcement")
MODES = ("with_tactile", "without_tactile")
NORMAL_FRICTION = .8
EMPTY_CLOSURE_TOLERANCE_MM = 1.0
TASK = """You are choosing exactly one recovery action after a short tabletop test lift
of the orange workpiece by this SO-ARM101 parallel-jaw gripper. The input contains three
pre-contact RGB views, calibrated wrist depth and fixed stereo RGB at the pinch and probe hold.
It also includes measured jaw gap and the TCP path. If a PaXini packet is present,
left/right loads are calibrated in N, positive normal is compression, and world X is the
long direction of the workbench. Read the pinch, early lift, and final hold as a time series.
Select position_correction to open and regrasp around the estimated visual center with the
original closing command. Select grip_reinforcement to close more firmly at the original target after lowering
the gripper back to the table, with the same target pose. Both then attempt the same tray placement. Use position_correction
for a pinch at an edge, poorly aligned or one-sided contact. Use grip_reinforcement for contact
near the center with insufficient grip to support the test lift. Choose the best of the two
even with incomplete observations. The trial's fault settings are hidden. Give one concise
observation from the supplied measurements or images as evidence."""
SCHEMA = {
    "type": "object", "additionalProperties": False,
    "properties": {"action": {"type": "string", "enum": list(ACTIONS)},
                   "evidence": {"type": "string"}},
    "required": ["action", "evidence"],
}


CONTACT_GUIDANCE = """First diagnose pad engagement from paxini_contact_history.
Use the complete closure before interpreting the final hold. When neither pad crosses the
contact threshold during closure and the measured gap reaches the command, a visually
centered silhouette can still be an empty closure. Stronger pressure at the same pose
requires an actual opposing contact pair; in a missed or one-sided engagement, recover
alignment. A bilateral contact established at closure and lost during lift supports slip;
pressure centered on both pads with sustained shear can support adding grip. With bilateral
contact near a pad edge, consider realignment even if contact was initially present.
In the evidence name the phase and observation you used. Treat simulator observations as
evidence and choose one of the same two allowed actions."""


def contact_history(trace, commanded_gap):
    """Summarize contact timing using pad forces and the gripper command."""
    phase_groups = {
        "closure": {"close", "pinch"},
        "lift": {"probe_lift"},
        "hold": {"probe_hold"},
    }
    phases = {}
    for name, labels in phase_groups.items():
        samples = [sample for sample in trace if sample["phase"] in labels]
        forces = np.asarray([[pad["normal_n"] for pad in sample["pads"]] for sample in samples])
        detected = forces >= CONTACT_NORMAL_N
        paired = np.all(detected, axis=1)
        centers = [[sample["pads"][i]["cop_world_x_offset_mm"] for sample, mask in zip(samples, detected)
                    if mask[i] and sample["pads"][i]["cop_world_x_offset_mm"] is not None]
                   for i in range(2)]
        phases[name] = {
            "samples": len(samples),
            "contact_samples_left_right": detected.sum(axis=0).tolist(),
            "bilateral_samples": int(paired.sum()),
            "max_normal_N_left_right": forces.max(axis=0).round(3).tolist(),
            "median_contact_x_mm_left_right": [round(median(vals), 2) if vals else None for vals in centers],
        }
    close = phases["closure"]
    endings = [sample for sample in trace if sample["phase"] == "pinch"][-5:]
    actual_gap = median(s["jaw_gap_mm"] for s in endings)
    count = sum(n > 0 for n in close["contact_samples_left_right"])
    if count == 0:
        state = "no_pad_contact_during_closure"
    elif count == 1:
        state = "only_one_pad_contact_during_closure"
    elif close["bilateral_samples"] == 0:
        state = "no_simultaneous_bilateral_contact"
    elif phases["hold"]["bilateral_samples"] == 0:
        state = "bilateral_contact_at_closure_absent_at_hold"
    else:
        state = "bilateral_contact_at_closure_and_hold"
    return {
        "contact_threshold_N": CONTACT_NORMAL_N,
        "contact_pattern": state,
        "commanded_gap_mm": commanded_gap,
        "measured_end_of_closure_gap_mm": round(actual_gap, 3),
        "measured_minus_commanded_gap_mm": round(actual_gap - commanded_gap, 3),
        "phases": phases,
    }


def layouts(source):
    if len(source["runs"]) != 12:
        raise ValueError("The source must contain exactly 12 pick-pilot layouts")
    rng = np.random.default_rng(source["seed"] + 4306)
    assigned = {}
    for shape in SHAPES:
        group = [r["trial"] for r in source["runs"] if r["shape"] == shape]
        if len(group) != 4:
            raise ValueError("Expected four source layouts per shape")
        numbers = rng.permutation(group)
        for i, number in enumerate(numbers):
            assigned[number] = ("off_center" if i < 2 else "low_friction", (-1, 1)[i % 2])
    condition_counts = Counter()
    for row in source["runs"]:
        case = Case(**row["case_eval"])
        condition, sign = assigned[case.trial]
        order = MODES if condition_counts[condition] % 2 == 0 else MODES[::-1]
        condition_counts[condition] += 1
        yield case, condition, sign, order, row["candidates"][row["selection"]]


class RecoveryBench(PickBench):
    archive_raw = False

    def plan_grasp(self, selected, condition, sign, shape):
        return grasp_parameters(self, selected, condition, sign, shape)

    def reset_trial(self, case, friction):
        for side in ("left", "right"):
            pair = self.model.pair(f"{side}_tactile_pair").id
            self.model.pair_friction[pair, :3] = [friction, friction, .004 * friction / .8]
        super().reset(case)

    def sample(self, phase):
        raw = self.read_tactile() * COMPRESSION_POSITIVE
        features = []
        for i, pad in enumerate(raw):
            summed = pad.sum(axis=0)
            cop = self._center_xy_mm(pad)
            basis = self.data.site_xmat[self.sites[i]].reshape(3, 3)
            features.append({
                "normal_n": round(float(summed[2]), 4),
                "shear_n": round(float(np.linalg.norm(summed[:2])), 4),
                "cop_xy_mm": np.round(cop, 3).tolist() if cop is not None else None,
                "cop_world_x_offset_mm": round(float(basis[0, :2] @ cop), 3) if cop is not None else None,
                "active_taxels": int((pad[:, 2] > .008).sum()),
            })
        sample = {
            "sim_s": round(float(self.data.time), 3), "phase": phase,
            "tcp_mm": self.describe(self.data.qpos[:5])["xyz_mm"],
            "jaw_gap_mm": round(float(82.4137 - 1000*self.data.qpos[5:7].sum()), 3),
            "pads": features,
            "object_xyz_mm_eval": (self.data.xpos[self.object_id] * 1000).round(4).tolist(),
            "tray_contact_eval": self.on(self.tray_geom),
        }
        if self.archive_raw:
            sample["_taxels"] = raw.astype(np.float32)
        return sample


def setpoint(pose, height, pitch, gap, seconds):
    x, y, _, yaw = pose
    return waypoint(x, y, height, pitch, gap, seconds, yaw)


def grasp_parameters(bench, selected, condition, sign, shape, *, geometry=None):
    nominal = [*selected["tcp_world_mm"], selected["yaw_deg"]]
    first = nominal.copy()
    if geometry is None:
        gap = {"brick": 26, "cylinder": 30, "bar": 22}[shape]
        half_x_mm = SHAPES[shape][1][0] * 1000
    else:
        gap, half_x_mm = geometry
    if condition == "off_center":
        # Preserve the workpiece layout and bias the initial tool target along X.
        for margin in (9., 6., 3.):
            first[0] = nominal[0] + sign * (half_x_mm + margin)
            try:
                bench.preview(probe_waypoints(first, gap))
                break
            except ValueError:
                continue
        else:
            first[0] = nominal[0] - sign * (half_x_mm + 6)
            bench.preview(probe_waypoints(first, gap))
    return nominal, first, gap


def probe_waypoints(pose, gap):
    z = pose[2]
    return [
        setpoint(pose, 59, -13, 76, 1.12),
        setpoint(pose, z, -1, 76, .81),
        setpoint(pose, z, -1, gap, .65),
        setpoint(pose, z, -1, gap, .32),
        setpoint(pose, z + 16, -3, gap, .62),
        setpoint(pose, z + 16, -3, gap, .40),
    ]


def probe(bench, pose, gap, capture, *, at_pinch=None):
    steps = probe_waypoints(pose, gap)
    for phase, point in zip(("approach", "descend", "close", "pinch"), steps[:4]):
        bench.run([point], phase)
    pinch = capture(("rgb_scene_left", "rgb_scene_right"))
    if at_pinch is not None:
        at_pinch()
    for phase, point in zip(("probe_lift", "probe_hold"), steps[4:]):
        bench.run([point], phase)
    hold = capture(("rgb_scene_left", "rgb_scene_right"))
    return pinch, hold


def retry(first, gap, action, run, corrected_pose, *, clear_before_descent=False):
    z = first[2]
    if clear_before_descent:
        # When the jaws reached their commanded closure, reopen at the probe
        # height so the descending fingers leave room around the workpiece.
        run([setpoint(first, z + 16, -3, 76, .58)], "clear_jaws")
        run([setpoint(first, z, -1, 76, 1.14)], "descend_clear")
    else:
        run([setpoint(first, z, -1, gap, .75)], "lower_probe")
    if action == ACTIONS[0]:
        run([setpoint(first, z, -1, 76, .65)], "open_probe")
        pose, close_gap = corrected_pose, gap
        z = pose[2]
        reposition = [
            ("retract", setpoint(first, 54, -11, 76, .75)),
            ("reposition", setpoint(pose, 54, -11, 76, .72)),
            ("descend_retry", setpoint(pose, z, -1, 76, .82)),
        ]
        for name, point in reposition:
            run([point], name)
    else:
        pose, close_gap = first, 14
    run([setpoint(pose, z, -1, close_gap, .78)], "close_retry")
    if clear_before_descent:
        run([setpoint(pose, z, -1, close_gap, .32)], "settle_grip")
        run([setpoint(pose, z + 8, -1, close_gap, 1.07)], "lift_clear")
    run([setpoint(pose, 51, -9, close_gap, 1.25)], "lift_retry")
    run([setpoint(pose, 51, -9, close_gap, .36)], "hold_retry")
    # Preserve the jaw orientation during the transport. With low friction,
    # rotating the wrist while carrying creates a separate slip disturbance.
    tray_pose = [*TRAY, z, pose[3]]
    place = [setpoint(tray_pose, 51, -9, close_gap, 1.85),
             setpoint(tray_pose, z + 7.63, -1, close_gap, 1.06),
             setpoint(tray_pose, z + 7.63, -1, 76, .72),
             setpoint(tray_pose, 51, -9, 76, .88)]
    for name, point in zip(("transfer", "lower_tray", "release", "retreat"), place):
        run([point], name)


def capture_rgb(bench, camera_names):
    return tuple(Image.fromarray(bench._render(name, (384, 216))) for name in camera_names)


def get_vision(bench, sensed=None):
    width, height = 384, 216
    names = ("rgb_scene_left", "rgb_scene_right", "wrist_rgb")
    frames = capture_rgb(bench, names)
    _, depth, valid, pixels, *_ = rgbd(bench) if sensed is None else sensed
    fraction = np.clip((depth - .195) / .26, 0, 1)[..., None]
    color = (np.array([239, 194, 111]) * (1-fraction) + np.array([18, 66, 117]) * fraction).astype(np.uint8)
    color[~valid] = [15, 21, 31]
    color[pixels] = [57, 174, 190]
    return (*frames, Image.fromarray(color).resize((width, height), Image.Resampling.LANCZOS))


def montage(path, before, pinch, hold):
    w, h, label_h = 384, 216, 22
    grid = Image.new("RGB", (w*2, (h+label_h)*4), "#142032")
    draw = ImageDraw.Draw(grid)
    labels = (
        "BEFORE | LEFT RGB", "BEFORE | RIGHT RGB",
        "BEFORE | WRIST RGB", "BEFORE | WRIST DEPTH, cyan: target",
        "PINCH | LEFT RGB", "PINCH | RIGHT RGB",
        "TEST LIFT HOLD | LEFT RGB", "TEST LIFT HOLD | RIGHT RGB",
    )
    for index, (name, image) in enumerate(zip(labels, (*before, *pinch, *hold))):
        x, y = index % 2 * w, index // 2 * (h+label_h)
        draw.text((x+7, y+4), name, fill="white")
        grid.paste(image, (x, y+label_h))
    grid.save(path, quality=88)


def summarize(trace):
    vision_arm, tactile = {}, {}
    windows = (("pinch", "pinch"), ("early_lift", "probe_lift"), ("hold", "probe_hold"))
    for name, phase in windows:
        window = [entry for entry in trace if entry["phase"] == phase]
        n = 5
        window = window[:n] if name == "early_lift" else window[-n:]
        vision_arm[name] = {"jaw_gap_mm": round(median(s["jaw_gap_mm"] for s in window), 3),
                            "tcp_z_mm": window[-1]["tcp_mm"][2]}
        sides = {}
        for i, side in enumerate(("left", "right")):
            pads = [s["pads"][i] for s in window]
            cop = [p["cop_world_x_offset_mm"] for p in pads if p["cop_world_x_offset_mm"] is not None]
            sides[side] = {
                "normal_N": round(float(np.mean([p["normal_n"] for p in pads])), 3),
                "shear_N": round(float(np.mean([p["shear_n"] for p in pads])), 3),
                "normal_sd_N": round(float(np.std([p["normal_n"] for p in pads])), 3),
                "contact_x_from_pad_center_mm": round(float(median(cop)), 2) if cop else None,
                "contact_samples": len(cop),
            }
        tactile[name] = sides
    return vision_arm, tactile


def evaluate_probe(bench, baseline):
    hold = [entry["object_xyz_mm_eval"][2] for entry in bench.trace if entry["phase"] == "probe_hold"]
    return {"min_hold_rise_mm": round(min(hold) - baseline, 3),
            "lifted": bool(min(hold) - baseline >= 8)}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize_results(records):
    counts = {}
    for condition in ("all", "off_center", "low_friction"):
        rows = [r for r in records if condition == "all" or r["fault_eval"]["condition"] == condition]
        by_mode = {}
        for mode in MODES:
            arm = [r for r in rows if r["mode"] == mode]
            calls = [r["inference_s"] for r in arm if r["inference_s"] is not None]
            by_mode[mode] = {
                "trials": len(arm), "successes": sum(r["success"] for r in arm),
                "median_total_s": median(r["total_s"] for r in arm) if arm else None,
                "median_inference_s": median(calls) if calls else None,
                "actions": dict(Counter(r["action"] for r in arm if r["action"] is not None)),
            }
        grouped = {}
        for row in rows:
            grouped.setdefault(row["trial"], {})[row["mode"]] = row
        pairs = [group for group in grouped.values() if len(group) == 2]
        discordant = {mode: sum(p[mode]["success"] and not p[other]["success"] for p in pairs)
                      for mode, other in (MODES, MODES[::-1])}
        pairs_match = sum(p[MODES[0]]["input_replay_verified"] and p[MODES[1]]["input_replay_verified"]
                          for p in pairs)
        speed = [p[MODES[0]]["total_s"] - p[MODES[1]]["total_s"] for p in pairs]
        counts[condition] = {"modes": by_mode, "pairs": len(pairs), "pairs_match": pairs_match,
                             "discordant": discordant,
                             "median_paired_wall_delta_s": median(speed) if speed else None}
    return counts


def run_trial(bench, case, condition, sign, mode, selected, friction, out, paired_inputs, stage, *, use_contact_history=False, clearance_recovery=False, vision_alignment=False, prior=None, inference_retries=0):
    start = time.monotonic()
    row = {"trial": case.trial, "shape": case.shape, "mode": mode,
           "fault_eval": {"condition": condition, "pad_friction": friction,
                          "spawn_mm": [case.x_mm, case.y_mm], "spawn_yaw_deg": case.yaw_deg},
           "action": None, "inference_s": None, "success": False, "input_replay_verified": False}
    prefix = f"pair-{case.trial:02d}-{mode}"
    frames = []
    last_frame = 0.
    aligned = None
    archive = {}

    def capture(image, sample=None):
        nonlocal last_frame
        frames.append(Image.fromarray(image).resize((540, 304), Image.Resampling.BILINEAR))
        now = time.monotonic()
        if now - last_frame > .25:
            temp = out / "frame.tmp"
            Image.fromarray(image).save(temp, format="JPEG", quality=83)
            temp.replace(out / "frame.jpg")
            last_frame = now

    def move(points, name):
        stage(name)
        bench.run(points, name)

    try:
        bench.reset_trial(case, friction)
        baseline = float(bench.data.xpos[bench.object_id, 2] * 1000)
        sensed = rgbd(bench) if vision_alignment or selected is None else None
        if selected is None:
            selected = proposals(bench, sensed[0])["A"]
        nominal, first, gap = bench.plan_grasp(selected, condition, sign, case.shape)
        row["fault_eval"]["first_target_offset_x_mm"] = round(first[0] - nominal[0], 3)
        row["nominal_gap_mm"] = gap
        before = get_vision(bench, sensed)

        def read_alignment():
            nonlocal aligned
            aligned = visual.capture_alignment(bench)
        bench.frame_sink = capture
        capture(bench.frame())
        stage("probe")
        pinch, hold = probe(bench, first, gap, lambda names: capture_rgb(bench, names),
                            at_pinch=read_alignment if vision_alignment else None)
        row["probe_eval"] = evaluate_probe(bench, baseline)
        arm, tactile = summarize(bench.trace)
        image = out / f"{prefix}-observations.jpg"
        montage(image, before, pinch, hold)
        base_digest = digest(image)
        row["original_image"] = image.name
        row["original_image_sha256"] = base_digest
        alignment = None
        if vision_alignment:
            alignment, overlay = visual.measure_alignment(visual.wrist_footprint(sensed), aligned)
            expanded = out / f"{prefix}-rgbd-alignment.jpg"
            visual.visual_packet(image, expanded, aligned, overlay, alignment)
            image = expanded
            row["visual_alignment"] = alignment
            archive = dict(zip(("pre_left_rgb", "pre_right_rgb", "pre_wrist_rgb", "pre_wrist_depth_preview",
                                "pinch_left_rgb", "pinch_right_rgb", "hold_left_rgb", "hold_right_rgb"),
                               map(np.asarray, (*before, *pinch, *hold))))
            _, depth, valid, mask, k, transform = sensed
            front_rgb, front_depth, front_k, front_world = aligned
            archive.update(wrist_depth_m=depth, wrist_depth_valid=valid, wrist_target_mask=mask,
                           wrist_K=k, wrist_world_from_optical=transform, pinch_front_rgb=front_rgb,
                           pinch_front_depth_m=front_depth, pinch_front_K=front_k,
                           pinch_front_world_from_optical=front_world)
        row["image"] = image.name
        visual_digest = digest(image)
        reference = (visual_digest, arm, tactile, row["probe_eval"], alignment)
        if case.trial in paired_inputs and paired_inputs[case.trial] != reference:
            raise ValueError("The shared pre-decision observations diverged across the two modalities")
        paired_inputs[case.trial] = reference
        row["input_replay_verified"] = True
        inputs = {"measured_arm": arm}
        if prior is not None:
            expected = (prior.get("original_image_sha256", prior["input_image_sha256"]),
                        prior["fault_eval"], prior["probe_eval"])
            if (base_digest, row["fault_eval"], row["probe_eval"]) != expected:
                raise ValueError("Physical replay differs from the previous iteration")
            row["baseline_replay_verified"] = True
        task = TASK
        if vision_alignment:
            inputs["visual_alignment"] = alignment
            task += "\n\n" + visual.GUIDANCE
        if mode == MODES[0]:
            inputs["paxini"] = tactile
            row["tactile"] = tactile
            if use_contact_history:
                row["contact_history"] = inputs["paxini_contact_history"] = contact_history(bench.trace, gap)
                task += "\n\n" + CONTACT_GUIDANCE
        prompt = task + "\n" + json.dumps(inputs, separators=(",", ":"))
        row["input_image_sha256"] = visual_digest
        row["input_json"] = f"{prefix}-input.json"
        write_json(out / row["input_json"], inputs)
        stage("inference", image=image.name, wait_started_unix_s=time.time(),
              tactile=tactile if mode == MODES[0] else None, contact_history=row.get("contact_history"), visual_alignment=alignment)
        attempts = []
        for index in range(inference_retries + 1):
            receipt = infer(prompt, image, SCHEMA)
            receipt.update(image=image.name, image_sha256=visual_digest, prompt=prompt, trial=case.trial, mode=mode)
            if "error" not in receipt or index == inference_retries:
                break
            archived = f"{prefix}-astra-attempt-{index + 1}.json"
            write_json(out / archived, receipt)
            attempts.append({"receipt": archived, "wall_s": receipt["wall_s"], "error": receipt["error"]})
            stage("inference", wait_started_unix_s=time.time())
        if attempts:
            row["inference_retries"] = attempts
        row["inference_s"] = round(receipt["wall_s"] + sum(r["wall_s"] for r in attempts), 3)
        row["receipt"] = f"{prefix}-astra.json"
        row["turn_id"] = receipt.get("turn_id")
        write_json(out / row["receipt"], receipt)
        if "error" in receipt:
            row["outcome"] = "inference_error"
            raise ValueError(receipt["error"])
        response = receipt["response"]
        row.update(action=response["action"], evidence=response["evidence"])
        if row["action"] not in ACTIONS:
            raise ValueError("Unrecognized recovery action")
        measured_gap = arm["hold"]["jaw_gap_mm"]
        clear_before_descent = (clearance_recovery and row["action"] == ACTIONS[1]
                                and abs(measured_gap - gap) <= EMPTY_CLOSURE_TOLERANCE_MM)
        row["recovery_motion"] = {
            "clear_before_descent": clear_before_descent,
            "commanded_probe_gap_mm": gap,
            "measured_probe_hold_gap_mm": measured_gap,
        }
        stage("recovery", action=row["action"], recovery_motion=row["recovery_motion"])
        retry(first, gap, row["action"], move, nominal, clear_before_descent=clear_before_descent)
        stage("settle")
        bench.settle(.60)
        row["score"] = bench.score()
        row["success"] = row["score"]["success"]
        row["outcome"] = "placed" if row["success"] else "placement_failed"
    except (ValueError, RuntimeError, KeyError) as error:
        row.setdefault("outcome", "execution_error")
        row["error"] = str(error)
        print(f"FAIL {prefix}: {row['outcome']}: {error}", flush=True)
    finally:
        bench.frame_sink = None

    row["sim_s"] = round(float(bench.data.time), 3)
    row["trace"] = f"{prefix}-trace.json"
    pad_arrays = [s.pop("_taxels") for s in bench.trace] if bench.archive_raw else []
    write_json(out / row["trace"], bench.trace)
    row["total_s"] = round(time.monotonic() - start, 3)
    if archive:
        row["rgbd_raw"] = f"{prefix}-rgbd.npz"
        np.savez_compressed(out / row["rgbd_raw"], **archive)
    if pad_arrays:
        row["paxini_raw"] = f"{prefix}-paxini.npz"
        np.savez_compressed(out / row["paxini_raw"], forces_N=np.stack(pad_arrays),
                            sim_s=[s["sim_s"] for s in bench.trace],
                            phase=[s["phase"] for s in bench.trace],
                            pad_names=["left", "right"], taxel_xy_mm=[[p["x_mm"], p["y_mm"]] for p in POINTS])
    if frames:
        row["replay"] = f"{prefix}-motion.webp"
        frames[0].save(out / row["replay"], save_all=True, append_images=frames[1:],
                       duration=80, loop=0, quality=67, method=1)
    return row


def run_batch(args):
    args.out.mkdir(parents=True, exist_ok=True)
    if any((args.out / name).exists() for name in ("results.json", "live.json")):
        raise FileExistsError("Use a fresh output directory for each comparison")
    source = json.loads(args.source.read_text())
    calibration = json.loads(args.calibration.read_text())
    if digest(args.scene) != source["scene_sha256"]:
        raise ValueError("Use the same robot scene as the original pick pilot")
    by_shape = calibration["chosen"]
    if set(by_shape) != set(SHAPES) or any(not 0 < mu < NORMAL_FRICTION for mu in by_shape.values()):
        raise ValueError("Incomplete or invalid friction calibration")
    design = list(layouts(source))
    prior_runs = {}
    previous = None
    if args.baseline:
        previous = json.loads(args.baseline.read_text())
        expected_design = (digest(args.scene), digest(args.source), digest(args.calibration))
        if (previous["scene_sha256"], previous["source_sha256"], previous["calibration_sha256"]) != expected_design:
            raise ValueError("The baseline must use the identical scene, layouts and calibration")
        prior_runs = {(r["trial"], r["mode"]): r for r in previous["records"]}
        expected_keys = {(case.trial, mode) for case, _, _, _, _ in design for mode in MODES}
        if set(prior_runs) != expected_keys:
            raise ValueError("Expected a complete baseline with all paired trials")
    result = {"model": MODEL, "effort": EFFORT, "layout_seed": source["seed"], "scene_sha256": digest(args.scene),
              "source_sha256": digest(args.source), "calibration_sha256": digest(args.calibration),
              "mujoco": mj.__version__, "normal_friction": NORMAL_FRICTION, "low_friction_by_shape": by_shape,
              "planned": 2 * len(design), "records": [], "batch_started_unix_s": time.time()}
    result["policy"] = "contact_history" if args.contact_history else "windowed_tactile"
    result["visual_alignment"] = args.vision_alignment
    result["recovery_controller"] = "clearance_recovery" if args.clearance_recovery else "original"
    if args.clearance_recovery:
        result["empty_closure_tolerance_mm"] = EMPTY_CLOSURE_TOLERANCE_MM
    if previous:
        result["previous"] = {
            "source_sha256": digest(args.baseline),
            "summary": previous["summary"],
            "records": [{k: row[k] for k in
                         ("trial", "mode", "action", "success", "total_s", "inference_s", "evidence")}
                        for row in previous["records"]],
        }
    start = time.monotonic()
    active = None
    done = False
    benches = {}
    paired_inputs = {}

    def publish():
        write_json(args.out / "live.json", {**result, "summary": summarize_results(result["records"]),
                                           "active": active, "done": done})

    publish()
    try:
        for case, condition, sign, order, selected in design:
            if case.shape not in benches:
                fixture = visual.add_alignment_view if args.vision_alignment else None
                benches[case.shape] = RecoveryBench(args.scene, case.shape, visual_fixture=fixture)
                benches[case.shape].archive_raw = args.vision_alignment
            bench = benches[case.shape]
            friction = NORMAL_FRICTION if condition == "off_center" else by_shape[case.shape]
            for mode in order:
                active = {"trial": case.trial, "shape": case.shape, "mode": mode,
                          "fault": condition, "phase": "reset", "trial_started_unix_s": time.time()}
                publish()

                def stage(phase, **update):
                    active.update(phase=phase, **update)
                    publish()

                row = run_trial(bench, case, condition, sign, mode, selected, friction, args.out,
                                paired_inputs, stage, use_contact_history=args.contact_history,
                                clearance_recovery=args.clearance_recovery, vision_alignment=args.vision_alignment,
                                prior=prior_runs.get((case.trial, mode)))
                result["records"].append(row)
                active = None
                publish()
                print(f"{case.trial:02d} {mode}: {row['action']} {row['outcome']}, "
                      f"{row['total_s']:.2f} s total, {row['inference_s']} s inference", flush=True)
    finally:
        result["batch_s"] = round(time.monotonic() - start, 3)
        result["aborted"] = len(result["records"]) != result["planned"]
        result["summary"] = summarize_results(result["records"])
        fields = ("trial", "shape", "condition", "mode", "action", "success", "outcome",
                  "total_s", "inference_s", "sim_s", "turn_id", "error")
        with (args.out / "runs.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fields, extrasaction="ignore")
            writer.writeheader()
            for row in result["records"]:
                writer.writerow({**row, "condition": row["fault_eval"]["condition"]})
        write_json(args.out / "results.json", result)
        done = True
        active = None
        publish()
        for bench in benches.values():
            bench.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("scene", "source", "calibration", "out"):
        parser.add_argument(f"--{name}", required=True, type=Path)
    parser.add_argument("--contact-history", action="store_true", help="Expose contact history to the tactile policy")
    parser.add_argument("--clearance-recovery", action="store_true",
                        help="Reopen before descending when the probe grip closes without a load")
    parser.add_argument("--vision-alignment", action="store_true", help="Add optical jaw alignment and archive sensors")
    parser.add_argument("--baseline", type=Path, help="Verify against a prior paired comparison and display it")
    run_batch(parser.parse_args())
