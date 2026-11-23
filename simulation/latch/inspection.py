"""Paired, initial-state-preserving inspection on the R7. Model I/O excludes the judge."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import random
import shutil
import sys
import time

import numpy as np

from dashboard import photo, write_frame
from flush_bench import FlushBench, HERE, POINTS, SETUP_REVISION, VIEWS
sys.path.insert(0, str(HERE.parent/"stacking"))
from astra_runtime import infer, utc
from evaluate import atomic, MODEL, EFFORT

FIXED = dict(x_mm=0,z_mm=-1,dx_mm=2,dz_mm=0,gap_mm=24)
AXES = list(FIXED)
POLICIES = ("on","off","rule","current_rule")
BUDGET = 2
PROMPT = """You control a real-time-sampled SO-ARM101 R7 in MuJoCo for incoming inspection. Determine whether a latch was already engaged or loose on arrival. Preserve that original condition all the way through release. A sufficiently strong inspection can change the mechanism. The outside tab of both parts starts flush at the same nominal location. An engaged latch normally restrains play; grasp slip, arm compliance and bias can complicate interpretation. You have no calibration examples or labeled thresholds. The four tiles are fixed left and right RGB above, wrist RGB and masked wrist depth below. Dark invalid depth is missing. On follow-up the image is from the loaded hold of your last probe; control has since returned to open home. Each record describes your own earlier action and readings in order. Base axes +X forward and +Z up; currents are signed mA proxies, TCP is measured in mm. When supplied, touch is reaction on pads in base axes and local per-pad compression/shear. Select a contact point (x_mm along the handle, z_mm high or low relative to its calibrated center), motion dx_mm,dz_mm, and jaw gap. Common driver uses IK to pinch at your point, move by your displacement, hold, return, open and withdraw. It does not select further probes. Reachable ranges x [-8,8], z [-5,5], dx [-4,4], dz [-9,4], gap [23,31] mm. You may execute up to two probes, or stop before either. Return kind=finish with answer engaged/loose/defer when ready. For finish unused geometric fields may be zero. For probe the answer field is unused. Explain briefly which cue motivated your decision. Use only supplied data."""
FIELDS = {k:{"type":"number"} for k in AXES}
FIELDS.update(kind={"type":"string","enum":["probe","finish"]},
              answer={"type":"string","enum":["engaged","loose","defer"]}, reason={"type":"string"})
SCHEMA = {"type":"object","properties":FIELDS,"required":list(FIELDS),"additionalProperties":False}
POINT_XY=[[p["x_mm"],p["y_mm"]] for p in POINTS]


class Live:
    def __init__(self,folder):
        self.folder=folder
        self.data={"stage":"치구 검증","results":[],"points":POINT_XY,"image":0}
        self.contact_image=None
        shutil.copyfile(HERE/"inspection.html",folder/"index.html")

    def status(self,**items):
        self.data.update(items)
        atomic(self.folder/"status.json",self.data)

    def frame(self,b,s):
        frame = write_frame(b, s, self.folder / "wide.jpg", self.folder / "cams.jpg", self.folder / "gripper.jpg")
        if s["phase"]=="hold":
            self.contact_image=b.view()
        self.status(image=time.time_ns(), **frame)
        # Pace the display during motion. Model latency is measured independently.
        time.sleep(.095)


def metrics(samples):
    pinch=[v for v in samples if v["phase"]=="pinch"][-3:]
    probe=[v for v in samples if v["phase"] in ("probe","hold")]
    offset=np.mean([v["current_mA"] for v in pinch],axis=0)
    return {"rule":max(abs(v["pad_base_N"][0]) for v in probe),
            "current_rule":float(max(np.linalg.norm(np.array(v["current_mA"])-offset) for v in probe))}


def observation(samples, touch):
    out=[]
    for phase in ("pinch","probe","hold","return"):
        window=[s for s in samples if s["phase"]==phase]
        for j in np.unique(np.linspace(0,len(window)-1,3,dtype=int)):
            s=window[j]
            x={k:s[k] for k in ("phase","t","tcp_xz_mm","gap_mm","current_mA")}
            if touch:
                pads=np.asarray(s["pads"])
                x.update(reaction_base_xyz_N=s["pad_base_N"],pad_resultant_xyz_N=pads.sum(axis=1).round(3).tolist())
                if phase=="hold" and j==len(window)-1:
                    x["active_taxels"]=[[[i+1,*vec] for i,vec in enumerate(pad.tolist()) if max(map(abs,vec))>=.01] for pad in pads]
            out.append(x)
    return out


def calibrate(b,folder):
    prep={}
    for st in ("loose","engaged"):
        b.reset(st,0)
        entry={"handle_center_mm":(b.data.geom_xpos[b.object_geom]*1000).tolist()}
        # Compare raw, pre-colorized metric depth as well as all RGB pixels.
        views=b.feeds()
        entry["pixel_hashes"]={key:hashlib.sha256(v.tobytes()).hexdigest() for key,v in zip(VIEWS,views)}
        raw=b._render("wrist_depth",(320,180),"depth")
        entry["raw_depth_sha256"]=hashlib.sha256(raw.tobytes()).hexdigest()
        entry["fixed"]=metrics(b.probe(FIXED))
        entry["fixed_transitions"]=b.transitions
        b.reset(st,0)
        b.probe({**FIXED,"dx_mm":0,"dz_mm":-9})
        entry["deep_press_transitions"]=b.transitions
        entry["deep_press_final"]="engaged" if b.engaged() else "loose"
        prep[st]=entry
    for key in ("handle_center_mm","pixel_hashes","raw_depth_sha256"):
        if prep["loose"][key]!=prep["engaged"][key]:
            raise ValueError("Visible reset leak: "+key)
    if any(v["fixed_transitions"] for v in prep.values()) or not prep["loose"]["deep_press_transitions"]:
        raise ValueError("Fixture failed mechanical controls")
    atomic(folder/"calibration.json",prep)
    return prep


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene",type=Path,default=HERE.parent/"scene.xml")
    p.add_argument("--output",type=Path,required=True)
    selection = p.add_mutually_exclusive_group()
    selection.add_argument("--rule-only", action="store_true", help="Run both fixed rules without model calls")
    selection.add_argument("--model-only", action="store_true", help="Run both Astra conditions without fixed rules")
    p.add_argument("--timeout", type=float, default=180, help="Seconds allowed for each Codex call")
    p.add_argument("--stop-after-errors", type=int, default=0,
                   help="Stop after this many consecutive episode-level backend errors (0 disables)")
    p.add_argument("--retry-archive", type=Path, help="Rerun inference errors from a saved inspection run")
    args=p.parse_args();f=args.output
    if (args.rule_only or args.model_only) and args.retry_archive:
        p.error("Policy selection and --retry-archive cannot be combined")
    policies = ("rule", "current_rule") if args.rule_only else ("on", "off") if args.model_only else POLICIES
    f.mkdir(parents=True,exist_ok=True)
    if (f/"results.json").exists(): raise FileExistsError("Use a new run directory")
    previous = json.loads((args.retry_archive / "results.json").read_text()) if args.retry_archive else None
    retry_ids = set()
    if previous:
        retry_ids = {r["id"] for r in previous["trials"] if any(c.get("error") for c in r["calls"])}
        if previous.get("setup_revision") != SETUP_REVISION:
            raise ValueError("Retry requires a run with the same gripper and observation setup")
        if not retry_ids:
            raise ValueError("The archive has no inference errors to retry")
        if (previous["model"], previous["effort"], previous["probe_budget"]) != (MODEL, EFFORT, BUDGET):
            raise ValueError("The archive uses a different experimental configuration")
    live=Live(f);b=FlushBench(args.scene)
    if previous:
        live.data["results"] = [
            {key: row[key] for key in ("id", "policy", "answer", "preserved", "success")}
            for row in previous["trials"] if row["id"] not in retry_ids
        ]
    rows=[]; start=time.monotonic()
    backend_errors = 0
    data={"model":MODEL,"effort":EFFORT,"started_utc":utc(),"probe_budget":BUDGET,
          "policies":policies,"camera_names":VIEWS,"trials":rows,"setup_revision":SETUP_REVISION,"mujoco":__import__("mujoco").__version__}
    if previous:
        data["retry_of"] = previous["started_utc"]
        data["retry_ids"] = sorted(retry_ids)
        live.status(stage="오류 시행 재실행 준비", action=f"Astra 오류 시행 {len(retry_ids)}회 재실행")
    try:
        prep=calibrate(b,f)
        cases=[(v,s) for v in (1,2,3) for s in ("loose","engaged")]
        rng=random.Random(20261123);rng.shuffle(cases)
        data["design"]=[{"variant":v,"state":s} for v,s in cases]
        if previous and previous["design"] != data["design"]:
            raise ValueError("Archive fixture and case order differ")
        for j,(variant,st) in enumerate(cases,1):
            order=list(POLICIES);rng.shuffle(order)
            for policy in order:
                rid=f"B{j:02}-{policy}"
                if policy not in policies or (previous and rid not in retry_ids):
                    continue
                b.sink=None;b.reset(st,variant)
                current_img=b.view()
                live.contact_image=None
                live.status(stage="초기 관찰",id=f"B{j:02}",policy=policy,action="초기 카메라 관찰",wait=0)
                live.frame(b,b.samples[-1]);b.sink=live.frame
                row={"id":rid,"variant":variant,"policy":policy,"initial":st,"actions":[],"calls":[]}
                rows.append(row)
                start_time=time.monotonic()
                begin_sim=b.data.time
                history=[];answer="defer";error=None;failure_streak=0
                while True:
                    if policy in ("rule","current_rule"):
                        action=FIXED
                    else:
                        path=f/f"{rid}-{len(row['calls'])}.jpg"
                        photo(current_img,path)
                        obs={"remaining_probes":BUDGET-len(history),"history":history,
                             "initial_or_current_robot":{k:b.samples[-1][k] for k in ("tcp_xz_mm","gap_mm","current_mA")}}
                        if history and policy=="on":
                            obs["taxel_xy_mm_by_index_1_to_77"]=POINT_XY
                        prompt=PROMPT+"\n"+json.dumps(obs,separators=(",",":"))
                        if failure_streak: prompt+="\nLast proposed action was invalid: "+str(row["invalid"][-1])
                        live.status(stage="Astra 6.2 xhigh · 추론 중",phase=None,wait=time.time())
                        call=infer(prompt,path,SCHEMA,timeout=args.timeout)
                        call.update(image=path.name,image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),prompt=prompt)
                        row["calls"].append(call)
                        atomic(f/"results.json",data)
                        if "error" in call:
                            error=call["error"];break
                        decision=call["response"];row["last_reason"]=decision["reason"]
                        if decision["kind"]=="finish":
                            answer=decision["answer"];break
                        action={k:decision[k] for k in AXES}
                    live.status(stage="접촉 동작 실행 중",wait=0,action="접촉점 X/Z %g / %g mm  ·  이동 ΔX/ΔZ %g / %g mm  ·  턱 %g mm"%tuple(action[k] for k in AXES))
                    try:
                        if len(history)>=BUDGET: raise ValueError("Budget exhausted; finish now")
                        samples=b.probe(action)
                    except ValueError as exc:
                        row.setdefault("invalid",[]).append(str(exc));failure_streak+=1
                        if failure_streak>=2: error=str(exc);break
                        continue
                    failure_streak=0
                    row["actions"].append(dict(action))
                    if policy in ("rule","current_rule"):
                        observed=metrics(samples)[policy]
                        lo,hi=[prep[k]["fixed"][policy] for k in ("loose","engaged")]
                        answer="engaged" if abs(observed-hi)<abs(observed-lo) else "loose"
                        row["metric"]=observed
                        break
                    history.append({"action":action,"samples":observation(samples,policy=="on")})
                    if live.contact_image is None: raise RuntimeError("Missing loaded camera observation")
                    current_img=live.contact_image
                b.sink=None
                b.move(0,b.observation_dz,0,.5,"final_wait")
                preserved=not b.transitions
                row.update(answer=answer,error=error,preserved=preserved,correct=answer==st,success=bool(answer==st and preserved and error is None),
                           final="engaged" if b.engaged() else "loose",transitions=b.transitions,
                           sim_s=round(b.data.time-begin_sim,3), wall_s=round(time.monotonic()-start_time,3),
                           inference_s=round(sum(c["wall_s"] for c in row["calls"]),3))
                trace=f/f"{rid}.json.gz"
                trace.write_bytes(gzip.compress(json.dumps(b.samples,separators=(",",":")).encode(),mtime=0))
                live.data["results"].append({k:row[k] for k in ("id","policy","answer","preserved","success")})
                live.status(stage="시행 종료",phase=None,wait=0,action=row.get("last_reason","고정 경로·개발 치구 보정 기준"))
                atomic(f/"results.json",data)
                print(rid,st,answer,preserved,len(row["calls"]),row["actions"],error,flush=True)
                if policy in ("on", "off"):
                    backend_errors = backend_errors + 1 if any(c.get("error") for c in row["calls"]) else 0
                    if args.stop_after_errors and backend_errors >= args.stop_after_errors:
                        data.update(stopped_utc=utc(), stop_reason="consecutive_backend_errors",
                                    elapsed_wall_s=round(time.monotonic()-start,3))
                        atomic(f/"results.json",data)
                        live.status(stage="Astra 호출 오류 · 실행 중단", phase=None, wait=0,
                                    action=f"연속 {backend_errors}회 호출 오류 · 완료된 시행은 결과표에 표시")
                        return
        data.update(finished_utc=utc(),elapsed_wall_s=round(time.monotonic()-start,3))
        atomic(f/"results.json",data)
        summary = f"오류 시행 {len(rows)}회 재실행 완료" if previous else "전체 시행 완료"
        live.status(stage="비교 완료",phase=None,wait=0,action=summary + " · 아래에 판정과 상태 보존 결과 표시")
    finally: b.close()


if __name__=="__main__":
    main()
