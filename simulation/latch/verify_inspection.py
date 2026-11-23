"""Replay saved inspection decisions without model calls and audit the archived inputs."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from inspection import FIXED, HERE, SETUP_REVISION, VIEWS, FlushBench, metrics


def verify(scene, archive):
    doc=json.loads((archive/"results.json").read_text())
    bench=FlushBench(scene, revised=doc.get("setup_revision") == SETUP_REVISION)
    checked=[]
    prechecks={}
    ids=set()
    errors_without_turn=0
    try:
        for variant in sorted({r["variant"] for r in doc["trials"]}):
            pairs=[]
            for state in ("loose","engaged"):
                bench.reset(state,variant)
                raw=bench._render("wrist_depth",(320,180),"depth")
                pairs.append({"center_mm":(bench.data.geom_xpos[bench.object_geom]*1000).tolist(),
                              "display_sha256":[hashlib.sha256(v.tobytes()).hexdigest() for v in bench.feeds()],
                              "raw_depth_sha256":hashlib.sha256(raw.tobytes()).hexdigest()})
                bench.move(0,bench.observation_dz,0,4,"untouched")
                if bench.transitions: raise ValueError("Unforced transition")
            if pairs[0]!=pairs[1]: raise ValueError("Precontact visual leak")
            prechecks[str(variant)]=pairs[0]
        for row in doc["trials"]:
            if "answer" not in row: continue
            bench.reset(row["initial"],row["variant"])
            for a in row["actions"]:
                samples=bench.probe(a)
            bench.move(0,bench.observation_dz,0,.5,"final_wait")
            t=row["transitions"]
            if bench.transitions!=t or bench.engaged()!=(row["final"]=="engaged"):
                raise ValueError("State replay mismatch: "+row["id"])
            saved=json.loads(gzip.decompress((archive/(row["id"]+".json.gz")).read_bytes()))
            if saved!=bench.samples: raise ValueError("Sensor replay mismatch: "+row["id"])
            for call in row["calls"]:
                if hashlib.sha256((archive/call["image"]).read_bytes()).hexdigest()!=call["image_sha256"]:
                    raise ValueError("Input image hash mismatch")
                turn=call.get("turn_fingerprint",call.get("turn_id"))
                if turn:
                    if turn in ids: raise ValueError("Repeated turn")
                    ids.add(turn)
                elif call.get("error"):
                    errors_without_turn += 1
                else:
                    raise ValueError("Missing successful turn")
                if not call.get("error"):
                    assert call["turn_status"]=="completed" and call["runtime"]["model"]==doc["model"]
                    assert call["runtime"]["reasoningEffort"]==doc["effort"]
                if row["policy"]=="off" and any(x in call["prompt"].split("Use only supplied data.")[-1]
                      for x in ('"active_taxels"','"reaction_base_xyz_N"','"pad_resultant_xyz_N"')):
                    raise ValueError("Off-policy input contains touch")
            chosen=[{k:c["response"][k] for k in FIXED} for c in row["calls"]
                    if c.get("response",{}).get("kind")=="probe"]
            if not row.get("invalid") and row["calls"] and chosen!=row["actions"]:
                raise ValueError("Agent/execution mismatch")
            if row["policy"] in ("rule","current_rule"):
                assert abs(row["metric"]-metrics(samples)[row["policy"]])<1e-8
            assert row["success"]==(row["answer"]==row["initial"] and not t and row["error"] is None)
            checked.append(row["id"])
    finally: bench.close()
    return {"verified_episodes":checked,"unique_turns":len(ids),"precontact_by_variant":prechecks,
            "failed_calls_without_turn":errors_without_turn,"camera_order":VIEWS,"untouched_control_s":4,"trace_replay":"identical at storage precision"}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive",type=Path)
    parser.add_argument("--scene",type=Path,default=HERE.parent/"scene.xml")
    parser.add_argument("--out",type=Path)
    args=parser.parse_args()
    result=json.dumps(verify(args.scene,args.archive),indent=2)
    if args.out: args.out.write_text(result+"\n")
    else: print(result)
