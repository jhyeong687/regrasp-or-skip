"""Evaluate selective tactile remeasurement on a new split-rail stacking fixture."""

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
import gzip
import json
from pathlib import Path
from statistics import mean, median, pstdev
import xml.etree.ElementTree as ET

from bench import HERE, POINTS, StackBench, waypoint
from evaluate import EFFORT, MODEL, Monitor, atomic, model_call, observation_image

DEST = (204, -104)
HOVER_Z = 82
STOP_S = .44
ALIGN_S = .40
WRENCH_N = .085 * 9.80665
DRIFT_FLOOR_MM = 2.0
WIDTH_FACTOR = .36
SCHEMA = {
    "type": "object",
    "properties": {"remeasure": {"type": "boolean"}, "evidence": {"type": "string"}},
    "required": ["remeasure", "evidence"], "additionalProperties": False,
}
POLICIES = ("first", "always", "selective")
PROMPT = """Decide whether to pause for a new stable PaXini reading before placing an object held by an SO-ARM101. You can see the current RGB views, the original stable signed load estimate, a cheap cue from samples already streaming during transport, and the visible rail width. Do not call shell or other tools. The output must be one boolean remeasure and an evidence sentence. The simulator will execute this decision. This held-out policy uses the predeclared trigger: need a new measurement if valid transit samples are unavailable, or if the absolute estimated shift in offset is at least the supplied trigger threshold. Compare the values in millimeters. A pause costs 0.84 simulation seconds. The rapid transit estimate is a trigger only; when true, the arm stops and takes a separate quiet reading before moving laterally. There is no access to the real hidden ballast or later outcomes."""


@dataclass(frozen=True)
class Case:
    name: str
    ballast_mm: int
    rail_half_mm: int
    pitch_handoff_deg: int
    route: str
    split: str


CASES = (
    Case("dev-light", -7, 11, -26, "direct", "calibration"),
    Case("dev-shift", -16, 7, -27, "raised", "calibration"),
    Case("dev-right", 10, 8, -30, "raised", "calibration"),
    Case("rail-a", -17, 7, -27, "raised", "evaluation"),
    Case("rail-b", -11, 7, -26, "raised", "evaluation"),
    Case("rail-c", -5, 12, -27, "direct", "evaluation"),
    Case("rail-d", 7, 11, -28, "direct", "evaluation"),
    Case("rail-e", 12, 8, -29, "direct", "evaluation"),
    Case("rail-f", 17, 7, -30, "raised", "evaluation"),
    Case("rail-g", -14, 8, -31, "raised", "evaluation"),
    Case("rail-h", 5, 12, -26, "raised", "evaluation"),
)


def new_fixture(case):
    """Replace the single ledge with two rails on the same mounting plate."""
    def customize(world, asset):
        for name in ("base_layer", "middle_layer"):
            geom = world.find(f"geom[@name='{name}']")
            pos = geom.get("pos").split()
            geom.set("pos", f"{DEST[0]/1000} {DEST[1]/1000} {pos[2]}")
        ledge = world.find("geom[@name='ledge']")
        ledge.attrib.update(pos=f"{DEST[0]/1000} {(DEST[1]-13)/1000} .0354",
                            size=f"{case.rail_half_mm/1000} .0085 .003",
                            material="silver")
        ET.SubElement(world, "geom", {**ledge.attrib, "name": "second_rail",
                      "pos": f"{DEST[0]/1000} {(DEST[1]+13)/1000} .0354"})
        for dx in (-14,14):
            ET.SubElement(world, "geom", type="box", size=".0011 .001 .0003",
                          pos=f"{(DEST[0]+dx)/1000} {(DEST[1]-27)/1000} .03445",
                          material="handle_mark", contype="0", conaffinity="0")
    return customize


class RailBench(StackBench):
    def __init__(self, scene, case):
        super().__init__(scene, case.ballast_mm, fixture=new_fixture(case))
        self.other_rail = self.model.geom("second_rail").id

    def verdict(self):
        outcome = super().verdict()
        on_either = outcome["ledge_contact"] or self.on(self.other_rail)
        outcome["ledge_contact"] = on_either
        outcome["stable"] = bool(on_either and outcome["released"] and outcome["tilt_deg"] < 5
                                  and outcome["speed_m_s"] < .03)
        return outcome


def x_offset(sample):
    return sample["pad_moment_about_tcp_mNm"][1] / -sample["pad_load_world_N"][2]


def quiet_measurement(samples):
    down = [-s["pad_load_world_N"][2] for s in samples]
    valid = abs(mean(down)-WRENCH_N) < .12 and pstdev(down) < .06
    return mean(x_offset(s) for s in samples) if valid else None


def transit_cue(samples, old):
    # Samples already available while moving. No added settling window.
    valid = [s for s in samples if abs(-s["pad_load_world_N"][2]-WRENCH_N) < .18]
    estimate = median(x_offset(s) for s in valid) if len(valid) >= 2 else None
    return None if estimate is None else abs(estimate-old), len(valid)


def bounded_target(offset):
    return DEST[0] - max(-22., min(22., offset))


def prepare(bench, lift, case):
    bench.run(lift, "lift")
    initial = quiet_measurement(bench.trace[-8:])
    if initial is None:
        raise RuntimeError(f"{case.name} did not achieve a valid initial suspension")
    x = bounded_target(initial)
    approach = waypoint(x, DEST[1], HOVER_Z, case.pitch_handoff_deg, 22, 1.24)
    if case.route == "raised":
        bench.run([waypoint(222, 36, 82, -27, 22, .60)], "raise_for_transport")
    bench.run([approach], "transport")
    drift, valid = transit_cue(bench.trace[-4:], initial)
    observation = {"rail_width_mm": 2*case.rail_half_mm, "initial_offset_mm": round(initial, 3),
                   "transit_offset_shift_mm": None if drift is None else round(drift, 3),
                   "valid_transit_samples": valid,
                   "trigger_threshold_mm": round(max(DRIFT_FLOOR_MM,WIDTH_FACTOR*case.rail_half_mm),3),
                   "current_tcp": bench.describe(bench.data.qpos[:5])}
    return approach, observation


def gate(observation):
    drift = observation["transit_offset_shift_mm"]
    return drift is None or drift >= observation["trigger_threshold_mm"]


def complete(bench, approach, remeasure):
    x, y, z = approach["xyz_mm"]
    new_offset = None
    if remeasure:
        bench.run([{**approach, "time_s": STOP_S}], "remeasure")
        new_offset = quiet_measurement(bench.trace[-8:])
        if new_offset is not None:
            x = bounded_target(new_offset)
        bench.run([waypoint(x, y, z, approach["pitch_deg"], 22, ALIGN_S)], "realign")
    bench.run([waypoint(x,y,58,-16,22,.8), waypoint(x,y,58,-16,76,.6),
               waypoint(x,y,82,-27,76,.8)], "place")
    active_time = bench.data.time
    bench.settle()
    return {"remeasure": remeasure, "remeasured_offset_mm": new_offset, "release_tcp_x_mm": x,
            "robot_time_s": round(float(active_time),3),
            "outcome": bench.verdict()}


def observe_cases(scene, output, lift, cases, monitor):
    observations = {}
    for case in cases:
        bench = RailBench(scene, case)
        try:
            if case.name == "rail-a" and monitor is not None:
                monitor.status("execute_lift", trial=f"{case.name} / 새 이중 레일")
                monitor.capture(bench.frame(), bench.sample("ready"))
                bench.frame_sink = monitor.capture
            _, observation = prepare(bench, lift, case)
            image = output/(case.name+"-transit.jpg")
            digest = observation_image(bench, image) if case.split == "evaluation" else None
            observations[case.name] = {"case": asdict(case), "observation": observation, "image": image.name,
                                       "image_sha256": digest, "gate_recommendation": gate(observation)}
        finally:
            bench.close()
    return observations


def choose(record, output):
    name = record["case"]["name"]
    path = output/(name+"-call.json")
    if path.exists():
        cached = json.loads(path.read_text())
        if cached["image_sha256"] != record["image_sha256"] or cached["observation"] != record["observation"]:
            raise ValueError(f"Archived call input mismatch for {name}")
        return cached
    experiment_prompt = PROMPT + "\n" + json.dumps(record["observation"], indent=2)
    reply = model_call(experiment_prompt, output/record["image"], schema_definition=SCHEMA)
    if type(reply["response"].get("remeasure")) is not bool:
        raise ValueError(f"Invalid decision for {name}")
    result = {"requested_model": MODEL, "requested_reasoning_effort": EFFORT,
              "image_sha256": record["image_sha256"], "observation": record["observation"], **reply}
    atomic(path,result)
    return result


def run_trials(scene, lift, output, cases, records, calls, monitor):
    rows, traces = [], {}
    for case in cases:
        record = records[case.name]
        for policy in POLICIES:
            trial_id = f"{case.name}-{policy}"
            if case.split == "calibration" and policy == "selective":
                continue
            if case.split == "evaluation" and policy == "selective" and case.name not in calls:
                continue
            bench = RailBench(scene,case)
            try:
                if monitor and case.name == "rail-a":
                    monitor.status("execute_place", trial=trial_id, policy=policy)
                    bench.frame_sink = monitor.capture
                approach, observation = prepare(bench,lift,case)
                if observation != record["observation"]:
                    raise RuntimeError(f"Branches for {case.name} did not match")
                decision = (calls[case.name]["response"]["remeasure"] if policy=="selective" else policy=="always")
                result = complete(bench,approach,decision)
                inference = calls[case.name]["wall_s"] if policy=="selective" else 0.
                rows.append({"id": trial_id, "case": case.name, "split": case.split, "policy": policy,
                             **result, "astra_wall_s": inference,
                             "robot_plus_inference_s": round(result["robot_time_s"]+inference,3)})
                traces[trial_id] = bench.trace
                print(trial_id, "cue",observation["transit_offset_shift_mm"], "reread",decision,
                      "time",result["robot_time_s"], "outcome",result["outcome"],flush=True)
            finally:
                bench.close()
    return rows, traces


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sources=parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--source",type=Path,help="Original stacking result with the shared pickup waypoints")
    sources.add_argument("--replay-from",type=Path,help="Verify an archived comparison without new model calls")
    parser.add_argument("--scene",type=Path,default=HERE.parent/"scene.xml")
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--calibration-only",action="store_true")
    args=parser.parse_args()
    output=args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    (output/"index.html").write_bytes((HERE/"recheck.html").read_bytes())
    atomic(output/"points.json",POINTS)
    scene=args.scene.resolve()
    if args.replay_from:
        archived=json.loads(args.replay_from.read_text())
        records=archived["observations"]
        cases=[Case(**record["case"]) for record in records.values()]
        rows,_=run_trials(scene,archived["lift"],output,cases,records,archived["model_receipts"],None)
        expected=[{k:v for k,v in row.items() if k!="measurement_compute_s"} for row in archived["trials"]]
        if rows!=expected:
            raise RuntimeError("Archived trial outcomes or timing did not reproduce")
        return
    lift=json.loads(args.source.read_text())["lift"]
    selected=[c for c in CASES if not args.calibration_only or c.split=="calibration"]
    monitor=None if args.calibration_only else Monitor(output)
    records=observe_cases(scene,output,lift,selected,monitor)
    atomic(output/"observations.json",records)
    calls={}
    pending=[v for v in records.values() if v["case"]["split"]=="evaluation"]
    if pending:
        monitor.status("plan_place",trial="Astra 응답 0/8",policy="gpt-6.2-astra · xhigh")
        with ThreadPoolExecutor(max_workers=4) as pool:
            futures={pool.submit(choose,record,output):record["case"]["name"] for record in pending}
            for f in as_completed(futures):
                name=futures[f]
                calls[name]=f.result()
                print(name,calls[name]["response"],calls[name]["wall_s"],calls[name]["usage"],flush=True)
                atomic(output/"model-receipts.json",calls)
                monitor.status("plan_place",trial=f"Astra 응답 {len(calls)}/{len(pending)}",policy="gpt-6.2-astra · xhigh")
    rows,traces=run_trials(scene,lift,output,selected,records,calls,monitor)
    result={"requested_model":MODEL,"requested_reasoning_effort":EFFORT,"lift":lift,
            "gate":{"drift_floor_mm":DRIFT_FLOOR_MM,"rail_half_width_factor":WIDTH_FACTOR,
                    "window_samples":4,"read_stop_s":STOP_S,"alignment_s":ALIGN_S},
            "observations":records,"model_receipts":calls,"trials":rows}
    atomic(output/"results.json",result)
    payload=json.dumps({"points":POINTS,"trials":traces},separators=(",",":")).encode()
    (output/"sensor-traces.json.gz").write_bytes(gzip.compress(payload,mtime=0))
    if monitor:
        monitor.status("complete",trial="비교 완료",policy="Astra xhigh / 고정 규칙")


if __name__=="__main__":
    main()
