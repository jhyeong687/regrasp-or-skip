"""Compare Astra's own sensing and placement decisions against the existing rule."""

import argparse
from dataclasses import asdict, dataclass
import gzip
import hashlib
import json
from pathlib import Path
import random
import signal
import subprocess
import time

import numpy as np
from PIL import Image, ImageDraw

from astra_runtime import infer, utc
from bench import HERE, POINTS, StackBench, waypoint
from evaluate import EFFORT, MODEL, Monitor, POSE_FIELDS, atomic
from recheck import DEST, RailBench, complete as rule_place, gate, new_fixture, prepare, STOP_S

# Half-length of the shell at the bottom and top and the signed top shift.
SHAPES = {
    "trapezoid": (35, 26, 0, ".21 .67 .66 1"),
    "reverse-taper": (29, 36, 0, ".86 .48 .26 1"),
    "slanted-cap": (35, 30, 6, ".50 .51 .88 1"),
}
BASE = """Stack the held shell on the two visible rails using the SO-ARM101 in MuJoCo. Use the actual camera images and PaXini records below. Choose whether a fresh stationary measurement is worth taking before placement, or directly author the full placement trajectory. You will be evaluated on stable stacking and time. A stationary remeasurement holds the current pose for 0.44 simulated seconds and triggers another model call; this might help if the current evidence is inconsistent. If you request one, send decision 'remeasure' and an empty waypoint list. Otherwise send 'place' with the actual waypoints. Include a short evidence sentence tied to this sample. No tools, shell or external data.
All coordinates are in the R7 base frame. +X points across the table away from the robot, +Z is up, yaw 0 closes the jaws along Y. TCP is the midpoint of the pads. PaXini axes per pad are X right, Y up as viewed facing the pad, and positive Z compression. The world wrench is the force of the shell on the pads and is reported about TCP, with gravity downward. The loaded shell can overhang the rails; its internal weight can be eccentric. The signed world Y moment is affected by the X offset and by motion.
The two rail tops are at z=38.4 mm. The shell started on a mat so its current underside and height are visible relative to the fixture in the camera composite. All lengths sent to IK are in mm and times are per segment. For orientation calibration, the robot can typically reach pitch about -27° at z82 and -16° at z58 in this region. Gap 22 mm holds the object, and gap 76 mm opens. Supply 3–6 waypoints: move and descend, open at the intended placement pose, then retract above the fixture. Keep the two pads aligned with the part during contact. Tool paths are checked for kinematic validity. The harness applies your waypoint values verbatim after validation.
Image panels left to right: left RGB, right RGB, wrist RGB, fixture documentation view. Bottom: left and right PaXini patch maps. Cyan arrows show tangential components and color shows compression. Use these together with the numeric readings."""
SCHEMA = {
    "type": "object", "properties": {
        "decision": {"type": "string", "enum": ["place", "remeasure"]},
        "waypoints": {"type": "array", "items": {"type": "object", "properties": POSE_FIELDS,
            "required": list(POSE_FIELDS), "additionalProperties": False}},
        "evidence": {"type": "string"}},
    "required": ["decision", "waypoints", "evidence"], "additionalProperties": False,
}


@dataclass(frozen=True)
class NewObject:
    name: str
    shape: str
    ballast_mm: int
    rail_half_mm: int
    pitch_handoff_deg: int
    route: str


def cases(block):
    rng = random.Random(91733 + block)
    specs = []
    for shape in SHAPES:
        for width in (7, 11):
            for ballast in (-15, -9, 9, 15):
                # Alternate transport profiles without changing test availability.
                move = rng.choice(((rng.choice((-27, -29, -31)), "raised"),
                                   (rng.choice((-26, -28, -30)), "direct")))
                specs.append((shape, ballast + rng.randint(-2, 2), width, *move))
    rng.shuffle(specs)
    return [NewObject(f"b{block+1:02d}-{i+1:02d}", *spec) for i, spec in enumerate(specs)]


def shape_fixture(case):
    mount = new_fixture(case)
    bottom, top, shift, color = SHAPES[case.shape]
    def customize(world, asset):
        mount(world, asset)
        points = []
        for half, z, dx in ((bottom, -16, 0), (top, 16, shift)):
            y, b = 18, 3.5
            outline = ((half-b,y),(half,y-b),(half,-y+b),(half-b,-y),
                       (-half+b,-y),(-half,-y+b),(-half,y-b),(-half+b,y))
            points.extend((px+dx, py, z) for px,py in outline)
        asset.find("mesh[@name='top_shell']").set("vertex", " ".join(str(v/1000) for p in points for v in p))
        asset.find("material[@name='handle_finish']").set("rgba", color)
        # Align the decorative stripe to the shorter top, away from the pad faces.
        world.find("body[@name='top_layer']/geom[@material='vise']").set("pos", f"{shift/1000} 0 .01612")
    return customize


class ObjectBench(RailBench):
    def __init__(self, scene, case):
        self.shape = SHAPES[case.shape]
        StackBench.__init__(self, scene, case.ballast_mm, fixture=shape_fixture(case))
        self.other_rail = self.model.geom("second_rail").id

    def _surface_distance(self, points):
        local = (points-self.data.xpos[self.object_id]) @ self.data.xmat[self.object_id].reshape(3, 3)
        bottom, top, shift, _ = self.shape
        x, y, z = (local*1000).T
        f = np.clip((z+16)/32, 0, 1)
        ax = np.abs(x-shift*f)
        ay = np.abs(y)
        half = bottom+(top-bottom)*f
        # Plane-distance approximation for coloring the estimated patch; contact forces come from MuJoCo.
        return np.maximum.reduce([ax-half, ay-18, np.abs(z)-16, (ax+ay-half-14.5)/np.sqrt(2)]) / 1000


def summary(samples):
    tensors = np.asarray([s["pad_force_N"] for s in samples])
    out = []
    coords = np.array([[p["x_mm"], p["y_mm"]] for p in POINTS])
    for sample, pads in zip(samples, tensors):
        contacts = []
        for pad in pads:
            weight = np.maximum(pad[:,2], 0)
            total = weight.sum()
            center = (weight @ coords/total).round(2).tolist() if total > 0 else None
            contacts.append({"sum_Fx_Fy_Fz_N": pad.sum(axis=0).round(3).tolist(),
                             "pressure_center_xy_mm": center})
        out.append({"t_s": sample["t"], "world_force_N": sample["pad_load_world_N"],
                    "world_moment_at_TCP_mNm": sample["pad_moment_about_tcp_mNm"], "pads": contacts})
    return out


def image_of(bench, file, sample):
    views = [bench._render(camera, (400, 225)) for camera in
             ("rgb_scene_left", "rgb_scene_right", "wrist_rgb", "stack_detail")]
    image = Image.new("RGB", (1600, 390), "#152235")
    image.paste(Image.fromarray(np.column_stack(views)), (0, 20))
    draw = ImageDraw.Draw(image)
    for i, name in enumerate(("LEFT RGB", "RIGHT RGB", "WRIST RGB", "FIXTURE")):
        draw.text((10+i*400, 4), name, fill="white")
    for side, forces in enumerate(sample["pad_patch_N"]):
        x0 = 550+side*290
        draw.text((x0-50, 256), ("LEFT PAXINI", "RIGHT PAXINI")[side], fill="white")
        for point, (fx,fy,fz) in zip(POINTS, forces):
            x, y = x0+6*point["x_mm"], 335-6*point["y_mm"]
            size = 1.5+4*min(1, max(fz,0)**.5)
            draw.ellipse((x-size,y-size,x+size,y+size),
                         fill=(235,177,116) if fz > .006 else (66,83,107))
            if abs(fx)+abs(fy) > .012:
                draw.line((x,y,x+fx*45,y-fy*45), fill=(72,230,197), width=2)
    image.save(file, quality=87, optimize=True)
    return hashlib.sha256(file.read_bytes()).hexdigest()


def validate(bench, reply, stage):
    paths = reply["waypoints"]
    if reply["decision"] == "remeasure":
        if stage != "handoff" or paths:
            raise ValueError("Remeasure is available once at the handoff, with no waypoints")
        return
    if reply["decision"] != "place" or not 3 <= len(paths) <= 6:
        raise ValueError("Place requires 3–6 complete waypoints")
    if paths[-1]["gap_mm"] < 60 or paths[-1]["xyz_mm"][2] < 74:
        raise ValueError("Final waypoint must be open and retracted above z74")
    bench.preview(paths)


def decide(bench, case, stage, observation, folder, calls, monitor, persist):
    name = f"{case.name}-{stage}"
    image = folder/f"{name}.jpg"
    digest = image_of(bench, image, bench.trace[-1])
    correction = ""
    for attempt in range(1, 3):
        info = ("This is your only opportunity to request remeasurement." if stage == "handoff"
                else "Remeasurement just completed. You must now send the full placement waypoints.")
        prompt = BASE + "\n" + info + "\n" + json.dumps(observation, separators=(",", ":")) + correction
        monitor.status("waiting", trial=case.name, policy="astra")
        monitor.state.update(wait_started_ms=int(time.time()*1000), call_stage=stage, attempt=attempt)
        atomic(monitor.path, monitor.state)
        receipt = infer(prompt, image, SCHEMA)
        receipt.update(case=case.name, stage=stage, attempt=attempt, image=image.name,
                       image_sha256=digest, prompt=prompt, observation=observation)
        calls.append(receipt)
        try:
            if "error" in receipt:
                raise RuntimeError(receipt["error"])
            validate(bench, receipt["response"], stage)
        except (RuntimeError, ValueError, IndexError, TypeError, KeyError) as error:
            receipt["rejection"] = str(error)
            correction = "\nA previous proposed action was not executed. Validation error: " + str(error)
            persist()
            if "error" in receipt:
                # A rerouted or unavailable service is recorded, never scored as a successful Astra decision.
                break
        else:
            persist()
            return receipt["response"]
    return None


def evidence(bench, case, first, transit, stationary=None):
    current = bench.sample("observation")
    return {"tcp_world": current["tcp"], "gap_mm": current["gap_mm"],
            "fixture_from_camera_calibration": {"rail_top_z_mm": 38.4,
                "rail_centers_xy_mm": [[DEST[0], DEST[1]-13], [DEST[0], DEST[1]+13]],
                "rail_width_x_mm": case.rail_half_mm*2},
            "first_static_hold": first, "end_of_transport": transit,
            "stationary_reread": stationary}


def astra_place(bench, approach, case, folder, calls, monitor, persist):
    first = summary([s for s in bench.trace if s["phase"]=="lift"][-8:])
    transit = summary(bench.trace[-8:])
    observation = evidence(bench, case, first, transit)
    reply = decide(bench, case, "handoff", observation, folder, calls, monitor, persist)
    reread = bool(reply and reply["decision"] == "remeasure")
    if reread:
        monitor.status("remeasure", trial=case.name, policy="astra")
        bench.frame_sink = monitor.capture
        bench.run([{**approach, "time_s": STOP_S}], "remeasure")
        bench.frame_sink = None
        observation = evidence(bench, case, first, transit, summary(bench.trace[-8:]))
        reply = decide(bench, case, "after-reread", observation, folder, calls, monitor, persist)
    monitor.status("execute_place" if reply else "rejected", trial=case.name, policy="astra")
    waypoints = reply["waypoints"] if reply else []
    if reply:
        bench.frame_sink = monitor.capture
        bench.run(waypoints, "place")
    active_s = round(float(bench.data.time), 3)
    if reply:
        bench.settle()
    return {"remeasure": reread, "waypoints": waypoints,
            "robot_time_s": active_s,
            "outcome": bench.verdict() if reply else {"stable": False, "no_valid_action": True},
            "reason": None if reply is None else reply["evidence"]}


def save_trace(folder, trial, trace):
    payload = json.dumps(trace, separators=(",", ":")).encode()
    (folder/f"{trial}.json.gz").write_bytes(gzip.compress(payload, mtime=0))


def run_case(scene, lift, case, folder, monitor, dataset, persist):
    print(case.name, case.shape, flush=True)
    calls = dataset["model_receipts"][case.name]
    observed = {}
    for policy in ("rule", "astra"):
        bench = ObjectBench(scene, case)
        try:
            monitor.status("execute_lift", trial=case.name, policy=policy)
            start = time.monotonic()
            approach, record = prepare(bench, lift, case)
            if policy == "rule":
                observed = record
                monitor.capture(bench.frame(), bench.trace[-1])
                monitor.status("execute_place", trial=case.name, policy=policy)
                bench.frame_sink = monitor.capture
                result = rule_place(bench, approach, gate(record))
            else:
                if observed != record:
                    raise RuntimeError(f"Matched observations diverged for {case.name}")
                monitor.capture(bench.frame(), bench.trace[-1])
                result = astra_place(bench, approach, case, folder, calls, monitor, persist)
            bridge_wall = round(time.monotonic()-start, 3)
            inference = round(sum(c["wall_s"] for c in calls), 3)
            trial = f"{case.name}-{policy}"
            save_trace(folder, trial, bench.trace)
            dataset["trials"].append({"id": trial, "case": case.name, "policy": policy,
                "suite_wall_s": bridge_wall, "inference_wall_s": inference,
                "motion_plus_inference_s": round(result["robot_time_s"]+inference, 3),
                "model_calls": len(calls), **result})
            dataset["shared_observations"][case.name] = observed
            print(f"  {policy} stable={result['outcome']['stable']} reread={result['remeasure']} "
                  f"motion={result['robot_time_s']:.2f}s inference={inference:.2f}s", flush=True)
            persist()
        finally:
            bench.close()


def publish_dashboard(folder, dataset):
    fields = ("case", "stage", "runtime", "response", "error", "rejection",
              "wall_s", "finished_utc", "usage")
    calls = [call for records in dataset["model_receipts"].values() for call in records]
    latest = calls[-1] if calls else {}
    summary = {key: dataset[key] for key in ("started_utc", "elapsed_wall_s", "cases", "trials")
               if key in dataset}
    summary.update(model_calls=len(calls),
                   input_tokens=sum(c.get("usage", {}).get("total", {}).get("inputTokens", 0)
                                    for c in calls),
                   latest={key: latest[key] for key in fields if key in latest})
    atomic(folder/"dashboard.json", summary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--source", type=Path,
        default=HERE.parent.parent/"data/stacking-pilot/selective-recheck/results.json")
    parser.add_argument("--seconds", type=float, default=1800)
    parser.add_argument("--physics-only", action="store_true")
    args = parser.parse_args()
    lift = json.loads(args.source.read_text())["lift"]
    if args.physics_only:
        for case in cases(0):
            bench = ObjectBench(args.scene, case)
            try:
                approach, record = prepare(bench, lift, case)
                result = rule_place(bench, approach, gate(record))
                print(case.name, case.shape, record, result["outcome"], flush=True)
            finally:
                bench.close()
        return
    folder = args.output
    folder.mkdir(parents=True, exist_ok=True)
    if (folder/"results.json").exists():
        raise FileExistsError("Choose a new output directory to prevent mixing runs")
    (folder/"index.html").write_bytes((HERE/"free_placement.html").read_bytes())
    atomic(folder/"points.json", POINTS)
    monitor = Monitor(folder)
    dataset = {"requested_model": MODEL, "requested_effort": EFFORT, "lift": lift, "trials": [],
               "shared_observations": {}, "cases": {}, "budget_wall_s": args.seconds,
               "model_receipts": {}, "cli_version": subprocess.check_output(["codex","--version"], text=True).strip(),
               "auth_mode": subprocess.run(["codex","login","status"],capture_output=True,text=True).stderr.strip(),
               "started_utc": utc()}
    def persist():
        atomic(folder/"results.json", dataset)
        publish_dashboard(folder, dataset)

    stop_requested = False
    def finish_pair(_signal, _frame):
        nonlocal stop_requested
        stop_requested = True
    signal.signal(signal.SIGINT, finish_pair)

    persist()
    started = time.monotonic()
    block = 0
    while time.monotonic()-started < args.seconds:
        for case in cases(block):
            if stop_requested or time.monotonic()-started >= args.seconds:
                break
            dataset["cases"][case.name] = asdict(case)
            dataset["model_receipts"][case.name] = []
            run_case(args.scene, lift, case, folder, monitor, dataset, persist)
            consecutive_outages = list(dataset["model_receipts"].values())[-3:]
            if len(consecutive_outages) == 3 and all(calls and calls[-1].get("error") for calls in consecutive_outages):
                monitor.status("blocked", trial="모델 호출 3회 연속 실패", policy="astra")
                dataset["stop_reason"] = "three_consecutive_runtime_failures"
                break
        if stop_requested or dataset.get("stop_reason"):
            break
        block += 1
    dataset["elapsed_wall_s"] = round(time.monotonic()-started, 3)
    dataset["finished_utc"] = utc()
    if stop_requested:
        dataset.setdefault("stop_reason", "user_requested_wrap_up")
    persist()
    if dataset.get("stop_reason") != "three_consecutive_runtime_failures":
        monitor.status("complete", trial=f"{len(dataset['trials'])//2}개 조건", policy="astra")


if __name__ == "__main__":
    main()
