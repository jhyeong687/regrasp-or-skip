"""Run matched R7 pick-and-stack trials using a Codex Astra waypoint policy."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
from PIL import Image

from bench import BALLAST_MM, HERE, HOME, LEDGE_TOP_MM, PICK, TOWER, StackBench, waypoint
from sim import POINTS, jpeg

MODEL = "gpt-6.2-astra"
EFFORT = "xhigh"
POSE_FIELDS = {"xyz_mm": {"type": "array", "items": {"type": "number"}},
               "pitch_deg": {"type": "number"}, "yaw_deg": {"type": "number"},
               "gap_mm": {"type": "number"}, "time_s": {"type": "number"}}
SCHEMA = {
    "type": "object",
    "properties": {
        "waypoints": {"type": "array", "items": {
            "type": "object", "properties": POSE_FIELDS, "required": list(POSE_FIELDS),
            "additionalProperties": False}},
        "evidence": {"type": "string"}},
    "required": ["waypoints", "evidence"], "additionalProperties": False,
}
CALIBRATION_LIFT = [
    waypoint(222, 36, 48, -13, 76), waypoint(222, 36, 12, -1, 76),
    waypoint(222, 36, 12, -1, 22), waypoint(222, 36, 66, -16, 22), waypoint(222, 36, 66, -16, 22, .52),
]

COMMON_PROMPT = """Control an SO-ARM101 in MuJoCo. Output only the structured waypoint sequence and one sentence giving evidence. Do not use shell or other tools. The harness will execute your waypoints directly after IK validation.
Frame is the R7 base: +x farther along the table, +z up. A pose is TCP at the midpoint of the two pad frames. Pitch -20 raises the grip appropriately for transport, yaw 0 closes along the world y axis. Each time_s is duration from the preceding waypoint. Typical feasible pitch near this bench is -1 at z12, -13 at z48, -16 at z66, -27 at z82, -32 at z90. Keep yaw 0 to preserve the orientation of the long axis. Gap 76 is open and gap 22 is a firm closure command. Use smooth poses and allow around one second for cross-table transport.
Scene geometry: top part starts on the mat at x222 y36. The bottom two layers are fixed in the experiment. The 14 mm wide top ledge is centered at x204 y-101, with top z38.4. The long top shell can overhang the ledge. It weighs 85 g. An unknown hidden mass offset may require an intentional lateral offset of the shell so its COM ends over the ledge. No layout tag encodes its direction. Images show simulator cameras, and XYZ and signed forces are in base coordinates unless specified. A stable lift lets you estimate the load. For a fully supported hold, an x-displaced downward load on the pads contributes a moment about base y of the same sign as its x displacement from the TCP.
"""
LIFT_PROMPT = """Phase 1. Use the images and bench coordinates to pick the top layer at the source and briefly hold above the mat for sensing. Approach from above with open jaws, close on the lower portion of the part near z12, rise to about z66, and provide a separate 0.5-second settling waypoint without opening. Give the actual positions and orientations to execute. The next call will handle placement."""
PLACE_PROMPT = """Phase 2. The arm is holding the top layer. Choose where the gripper should release so the released shell will remain balanced. The measured wrench, when present, is the object's force on the two pads, with gravity downward. In that case an approximate x displacement from the TCP is y moment (mNm) divided by downward load (N), in mm. Centering the mass, and permitting a few mm of placement drift, is useful. Estimate the unknown offset using the observations actually provided. If it cannot be determined, choose your best hypothesis from available evidence. For a validated route, move laterally above the destination at around z82 and pitch -27, lower the pad midpoint near z58 and pitch -16, open at the placement, then withdraw above. Use actual x and y targets, not symbolic choices. Return one sequence ending open and retracted."""


def observation_image(bench, path):
    panels = [bench._render(name, (460, 258))
              for name in ("rgb_scene_left", "rgb_scene_right", "wrist_rgb")]
    image = np.column_stack(panels)
    Image.fromarray(image).save(path, quality=89, optimize=True)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def model_call(prompt, image_path, *, schema_definition=SCHEMA):
    with tempfile.TemporaryDirectory(prefix="r7-stack-model-") as directory:
        schema = Path(directory)/"schema.json"
        schema.write_text(json.dumps(schema_definition))
        command = ["codex", "exec", "--json", "--ephemeral", "--ignore-user-config",
                   "--skip-git-repo-check", "--sandbox", "read-only", "-C", directory,
                   "-m", MODEL, "-c", f'model_reasoning_effort="{EFFORT}"',
                   "--output-schema", str(schema), "--image", str(image_path.resolve()), "-"]
        start = time.monotonic()
        process = subprocess.run(command, input=prompt, text=True, capture_output=True, timeout=180)
        elapsed = time.monotonic()-start
    events = [json.loads(line) for line in process.stdout.splitlines() if line.startswith("{")]
    items = [event["item"] for event in events if event["type"] == "item.completed"]
    unexpected = [item["type"] for item in items if item["type"] not in ("reasoning", "agent_message", "plan")]
    replies = [item["text"] for item in items if item["type"] == "agent_message"]
    usage = next((event.get("usage") for event in events if event["type"]=="turn.completed"), None)
    if process.returncode or unexpected or not replies or not usage:
        errors = [event.get("message", event.get("error")) for event in events
                  if event["type"] in ("error", "turn.failed")]
        raise RuntimeError(f"Codex call failed with {process.returncode}, {unexpected}, {errors}")
    return {"response": json.loads(replies[-1]), "wall_s": round(elapsed, 3), "usage": usage}


def summarize(bench, tactile):
    samples = bench.trace[-10:]
    common = {"tcp": bench.describe(bench.data.qpos[:5]), "gripper_gap_mm": samples[-1]["gap_mm"],
              "joint_rad": bench.data.qpos[:7].round(5).tolist(),
              "signed_actuator_torque_Nm": np.mean([s["arm_torque_Nm"] for s in samples], axis=0).round(4).tolist()}
    if tactile:
        common["paxini"] = {
            "window_samples": len(samples),
            "pad_resultant_local_N": np.mean([s["pad_resultant_N"] for s in samples], axis=0).round(4).tolist(),
            "sum_force_base_N": np.mean([s["pad_load_world_N"] for s in samples], axis=0).round(4).tolist(),
            "sum_moment_about_tcp_base_mNm": np.mean([s["pad_moment_about_tcp_mNm"] for s in samples], axis=0).round(3).tolist(),
            "z_load_std_N": round(float(np.std([s["pad_load_world_N"][2] for s in samples])), 4)}
    return common


def atomic(path, data):
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n")
    temp.replace(path)


class Monitor:
    def __init__(self, directory):
        self.directory = directory
        self.path = directory/"live.json"
        self.state = {"phase": "initialize"}

    def status(self, phase, **fields):
        self.state.update(phase=phase, **fields)
        atomic(self.path, self.state)

    def capture(self, images, sample):
        self.state.update({"bench": jpeg(images[0]), "cameras": jpeg(images[1]), "sample": sample})
        atomic(self.path, self.state)


def calibration_place(cg):
    x = TOWER[0] - cg
    return [waypoint(x, -101, 82, -27, 22, 1.1), waypoint(x, -101, 58, -16, 22),
            waypoint(x, -101, 58, -16, 76), waypoint(x, -101, 90, -32, 76)]


def record_measurement(bench):
    hold = bench.trace[-10:]
    heights = [s["top_xyz_mm_eval"][2] for s in hold]
    return {"airborne": bool(min(heights)>30 and not any(s["table_eval"] for s in hold)),
            "height_mm": round(float(np.mean(heights)), 3),
            "moment_y_mNm": round(float(np.mean([s["pad_moment_about_tcp_mNm"][1] for s in hold])), 3),
            "down_load_N": round(float(-np.mean([s["pad_load_world_N"][2] for s in hold])), 4)}


def prompt_for(phase, observation, correction=None):
    prompt = COMMON_PROMPT + "\n" + phase + "\nMeasurements:\n" + json.dumps(observation, indent=2)
    if correction is not None:
        prompt += "\nPreview rejected the unexecuted path with the following error. Revise the complete path.\n" + correction
    return prompt


def check_choice(bench, response, phase):
    waypoints = response["waypoints"]
    if phase == "lift":
        if not waypoints or waypoints[-1]["gap_mm"] > 30 or waypoints[-1]["xyz_mm"][2] < 49:
            raise ValueError("The final sensing pose must hold the part in the air with the jaws closed")
    elif not waypoints or waypoints[-1]["gap_mm"] < 60 or waypoints[-1]["xyz_mm"][2] < 74:
        raise ValueError("The final pose must open and withdraw after placing the part")
    return bench.preview(waypoints)


def call_checked(bench, phase, observation, image, retries=1):
    calls = []
    instruction = LIFT_PROMPT if phase == "lift" else PLACE_PROMPT
    reason = None
    for _ in range(retries+1):
        result = model_call(prompt_for(instruction, observation, reason), image)
        calls.append(result)
        try:
            check_choice(bench, result["response"], phase)
            return calls
        except (ValueError, IndexError, TypeError, KeyError) as error:
            reason = str(error)
            result["rejection"] = reason
    return calls


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=HERE.parent/"scene.xml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--reuse", action="store_true", help="Use archived model calls in the output directory")
    args = parser.parse_args()
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    scene = args.scene.resolve()
    monitor = Monitor(directory)
    log = {"model": MODEL, "effort": EFFORT, "calibration_only": args.calibrate, "trials": []}
    shared = StackBench(scene, BALLAST_MM[0])
    shared.frame_sink = monitor.capture
    try:
        monitor.status("calibration" if args.calibrate else "plan_lift", trial="공통 첫 들기")
        monitor.capture(shared.frame(), shared.sample("ready"))
        (directory/"index.html").write_bytes((HERE/"index.html").read_bytes())
        atomic(directory/"points.json", POINTS)
        if args.calibrate:
            lift = CALIBRATION_LIFT
        else:
            initial = directory/"before.jpg"
            checksum = observation_image(shared, initial)
            shared_obs = {"tcp": shared.describe(shared.data.qpos[:5]), "gripper_gap_mm": 76,
                          "rgb_view_order": ["left", "right", "wrist"]}
            shared_path = directory/"lift-call.json"
            if args.reuse and shared_path.exists():
                log["lift"] = json.loads(shared_path.read_text())
                if log["lift"]["image_sha256"] != checksum:
                    raise ValueError("The archived initial RGB did not match this robot scene")
            else:
                log["lift"] = {"input": shared_obs, "image_sha256": checksum,
                               "calls": call_checked(shared, "lift", shared_obs, initial)}
                atomic(shared_path, log["lift"])
            if "rejection" in log["lift"]["calls"][-1]:
                raise RuntimeError("Both proposed lift paths were rejected")
            lift = log["lift"]["calls"][-1]["response"]["waypoints"]
    finally:
        shared.close()

    prepared = []
    for index, cg in enumerate(BALLAST_MM):
        tag = f"pair-{index+1}"
        with_tactile = StackBench(scene, cg)
        try:
            monitor.status("execute_lift", trial=tag, policy="공통 첫 들기")
            with_tactile.frame_sink = monitor.capture
            with_tactile.run(lift, "lift")
            hold = record_measurement(with_tactile)
            print(tag, "lift", hold, flush=True)
            if not hold["airborne"]:
                raise RuntimeError(f"{tag} failed to lift the object; no matched placement observations")
            image = directory/f"{tag}-hold.jpg"
            checksum = observation_image(with_tactile, image) if not args.calibrate else None
            for tactile in ((True,) if args.calibrate else (False, True)):
                key = f"{tag}-{'touch' if tactile else 'vision-arm'}"
                prepared.append({"id": key, "hidden_cg_mm_eval": cg, "available_paxini": tactile,
                                 "lift_eval": hold, "input": summarize(with_tactile, tactile),
                                 "image": image.name, "image_sha256": checksum})
        finally:
            with_tactile.close()

    if not args.calibrate:
        monitor.status("plan_place", trial="PaXini 있음 / 없음", policy="Astra xhigh")
        def resolve(record):
            path = directory/(record["id"] + "-decision.json")
            if args.reuse and path.exists():
                return json.loads(path.read_text())
            # Each worker builds an independent preview model; only the main thread uses OpenGL.
            preview_bench = StackBench(scene, record["hidden_cg_mm_eval"])
            try:
                preview_bench.run(lift, "lift")
                calls = call_checked(preview_bench, "place", record["input"], directory/record["image"])
            finally:
                preview_bench.close()
            atomic(path, calls)
            return calls

        with ThreadPoolExecutor(max_workers=4) as pool:
            futures = {pool.submit(resolve, row): row for row in prepared}
            finished = 0
            for future in as_completed(futures):
                row = futures[future]
                row["calls"] = future.result()
                finished += 1
                print(row["id"], "plan", row["calls"][-1]["response"], flush=True)
                monitor.status("plan_place", trial=f"응답 {finished}/{len(prepared)}", policy="Astra xhigh")

    for row in prepared:
        # Re-execute the exact submitted lift in each matched branch.
        bench = StackBench(scene, row["hidden_cg_mm_eval"])
        try:
            monitor.status("execute_place", trial=row["id"], policy="PaXini 사용" if row["available_paxini"] else "카메라·팔 상태")
            bench.run(lift, "lift")
            if args.calibrate:
                path = calibration_place(row["hidden_cg_mm_eval"])
            else:
                current = row["calls"][-1]
                if "rejection" in current:
                    row["outcome"] = {"stable": False, "rejection": current["rejection"]}
                    log["trials"].append(row)
                    continue
                path = current["response"]["waypoints"]
            bench.frame_sink = monitor.capture
            bench.run(path, "place")
            bench.settle()
            row["place_waypoints"] = path
            row["outcome"] = bench.verdict()
            print(row["id"], row["outcome"], flush=True)
            log["trials"].append(row)
            atomic(directory/"results.json", log)
        finally:
            bench.close()
    atomic(directory/"results.json", log)
    monitor.status("complete", trial="비교 결과", policy="Astra xhigh" if not args.calibrate else "물리 검증")

if __name__ == "__main__":
    main()
