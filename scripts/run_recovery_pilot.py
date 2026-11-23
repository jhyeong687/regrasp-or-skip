"""Generate a bounded grasp-recovery pilot with physical MuJoCo retry branches."""

import argparse
from dataclasses import asdict, dataclass
import gzip
import json
from pathlib import Path
import sys

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from sim import COMPRESSION_POSITIVE, KEYS, POINTS, Sim


STEP_S = 0.04
FIRST_SLIDE_M = 0.0262
RETRY_SLIDE_M = 0.0335
SHIFT_M = 0.018
MIN_LIFT_MM = 8.0
MAX_TILT_DEG = 20.0
HOLD_SAMPLES = 6
SNAPSHOT_SPEC = mj.mjtState.mjSTATE_INTEGRATION
RETRY_ACTIONS = {
    "regrasp_in": (-SHIFT_M, FIRST_SLIDE_M),
    "regrasp_out": (SHIFT_M, FIRST_SLIDE_M),
    "regrasp_firmer": (0.0, RETRY_SLIDE_M),
}


@dataclass(frozen=True)
class Case:
    id: str
    scene: str
    split: str
    offset_mm: float
    mass_g: float
    friction: float


def case_list():
    """Offset is visible; payload and pad friction are independent hidden factors."""
    design = (
        ("calibration", -30, ((50, .8), (600, .8), (50, .04))),
        ("calibration", 0, ((50, .04), (50, .8), (600, .8), (180, .08))),
        ("calibration", 30, ((50, .8), (600, .8), (50, .04))),
        ("evaluation", -33, ((45, .65), (75, .74), (540, .65), (45, .025))),
        ("evaluation", -23, ((42, .52), (80, .74), (460, .74), (50, .035))),
        ("evaluation", -3, ((45, .76), (65, .32), (45, .065), (510, .76), (125, .055))),
        ("evaluation", 2, ((48, .74), (80, .28), (48, .055), (550, .74), (48, .012))),
        ("evaluation", 21, ((45, .74), (70, .21), (520, .74), (45, .032))),
        ("evaluation", 31, ((44, .68), (75, .74), (570, .68), (44, .026))),
    )
    return [Case(f"{split[:3]}-{i:02d}-{j:02d}", f"scene-{i:02d}", split, x, mass, friction)
            for i, (split, x, factors) in enumerate(design)
            for j, (mass, friction) in enumerate(factors)]


def vector(values, places=4):
    return np.asarray(values).round(places).tolist()


class ProbeRunner:
    def __init__(self):
        self.sim = Sim()
        self.model = self.sim.model
        self.fit = mj.MjData(self.model)
        self.object_q = int(self.model.joint("object_free").qposadr[0])
        self.table = self.model.geom("table").id
        self.grasp, self.lift = KEYS[3][1], KEYS[5][1]
        self.poses = {
            action: (self.shift(self.grasp, dx), self.shift(self.lift, dx), slide)
            for action, (dx, slide) in RETRY_ACTIONS.items()
        }

    def hand_pose(self, angles):
        self.fit.qpos[self.sim.arm_q] = angles
        mj.mj_forward(self.model, self.fit)
        x, z = self.fit.site_xpos[self.sim.sites].mean(axis=0)[[0, 2]]
        up = self.fit.site_xmat[self.sim.sites[0]].reshape(3, 3)[:, 1]
        return np.array([x, z, np.arctan2(up[0], up[2])])

    def shift(self, angles, offset_m):
        """Hold gripper height and pitch while shifting along the table's X axis."""
        if not offset_m:
            return angles.copy()
        moved = angles.copy()
        goal = self.hand_pose(angles) + [offset_m, 0, 0]
        eps = 1e-5
        for _ in range(8):
            current = self.hand_pose(moved)
            perturbations = [self.hand_pose(moved + d * eps) for d in np.eye(5)[1:4]]
            jac = (np.asarray(perturbations) - current).T / eps
            moved[1:4] += np.linalg.solve(jac, goal - current)
        limits = self.model.actuator_ctrlrange[:5]
        if (np.linalg.norm(self.hand_pose(moved) - goal) > 1e-5
                or np.any(moved < limits[:, 0]) or np.any(moved > limits[:, 1])):
            raise ValueError("Retry pose could not be solved within the simulated arm limits")
        return moved

    def reset(self, case):
        sim = self.sim
        sim.reset("normal")
        mass_kg = case.mass_g / 1000
        self.model.body_mass[sim.object_id] = mass_kg
        self.model.body_inertia[sim.object_id] = sim.initial_inertia * mass_kg / sim.initial_mass
        self.model.pair_friction[sim.pairs, :2] = case.friction
        mj.mj_setConst(self.model, sim.data)
        mj.mj_resetDataKeyframe(self.model, sim.data, 0)
        sim.data.qpos[self.object_q] += case.offset_mm / 1000
        mj.mj_forward(self.model, sim.data)
        sim.advance(KEYS[3][0])
        self.initial_z = float(sim.data.xpos[sim.object_id, 2])

    def read_sample(self, phase):
        sim = self.sim
        data = sim.data
        wrench = sim.taxels * COMPRESSION_POSITIVE
        nets = wrench.sum(axis=1)
        contact_x = []
        for side, values in enumerate(wrench):
            center = sim._center_xy_mm(values)
            axes = data.site_xmat[sim.sites[side]].reshape(3, 3)
            contact_x.append(None if center is None else round(float(axes[0, :2] @ center), 2))
        object_tilt = np.rad2deg(np.arccos(np.clip(data.xmat[sim.object_id, 8], -1, 1)))
        floor_contact = any({int(contact.geom1), int(contact.geom2)} == {self.table, sim.object_geom}
                            for contact in data.contact[:data.ncon])
        torque = abs(data.actuator_force[:5])
        return {
            "time_s": round(data.time, 3), "phase": phase,
            "joint_rad": vector(data.qpos[sim.arm_q], 5), "command_rad": vector(data.ctrl[:5], 5),
            "arm_torque_Nm": vector(torque),
            "gap_mm": round(float(82.4137 - 1000 * data.qpos[sim.slide_q].sum()), 2),
            "paxini_samples_N": vector(wrench, 5), "net_force_N": vector(nets),
            "contact_x_mm": contact_x, "pad_torque_Nm": vector(sim.pad_torque, 7),
            "object_z_mm_truth": round(float(data.xpos[sim.object_id, 2] - self.initial_z) * 1000, 3),
            "object_tilt_deg_truth": round(float(object_tilt), 2),
            "object_on_table_truth": floor_contact,
        }

    def move(self, phase, duration, angles, slide):
        sim = self.sim
        previous = sim.data.ctrl.copy()
        target = np.r_[angles, slide, slide]
        start = float(sim.data.time)
        length = round(duration / STEP_S)
        samples = []
        for index in range(1, length + 1):
            progress = index / length
            ease = progress**3 * (10 + progress * (-15 + 6 * progress))
            sim.data.ctrl[:] = previous + ease * (target - previous)
            sim.advance(STEP_S, scripted=False)
            if sim.data.time < start + index * STEP_S - .001:
                raise RuntimeError("Probe exceeded the simulation time limit")
            samples.append(self.read_sample(phase))
        return samples

    def attempt(self, grasp, lift, slide, capture_pinch=False):
        trace = self.move("close", 1.28, grasp, slide)
        pinch = self.images() if capture_pinch else None
        trace += self.move("lift", 1.40, lift, slide)
        trace += self.move("hold", .32, lift, slide)
        hold = trace[-HOLD_SAMPLES:]
        height = min(sample["object_z_mm_truth"] for sample in hold)
        tilt = max(sample["object_tilt_deg_truth"] for sample in hold)
        floor = any(sample["object_on_table_truth"] for sample in hold)
        saturated = sum(max(sample["arm_torque_Nm"]) >= 1.345 for sample in hold)
        peak = max(sample["object_z_mm_truth"] for sample in trace)
        return trace, {"success": bool(height >= MIN_LIFT_MM and tilt <= MAX_TILT_DEG and not floor
                                      and saturated < HOLD_SAMPLES),
                       "hold_min_rise_mm": height, "hold_max_tilt_deg": tilt,
                       "hold_touched_floor": floor, "hold_saturation_samples": saturated,
                       "peak_rise_mm": peak, "lost_lift": peak >= MIN_LIFT_MM and peak - height >= 5.0}, pinch

    def snapshot(self):
        state = np.empty(mj.mj_stateSize(self.model, SNAPSHOT_SPEC))
        mj.mj_getState(self.model, self.sim.data, state, SNAPSHOT_SPEC)
        return state

    def restore(self, snapshot):
        mj.mj_setState(self.model, self.sim.data, snapshot, SNAPSHOT_SPEC)
        mj.mj_forward(self.model, self.sim.data)
        self.sim.tactile_history.clear()

    def images(self):
        # The two fixed side cameras use their calibrated 640×360 projection.
        return self.sim.side_rgb()

    def run(self, case, output, frames):
        self.reset(case)
        before = self.images() if frames else None
        first, first_outcome, pinch = self.attempt(self.grasp, self.lift, FIRST_SLIDE_M, capture_pinch=frames)
        after = self.images() if frames else None
        observation = {"id": case.id, "scene": case.scene, "split": case.split,
                       **summarize(first, before, pinch, after)}
        if frames:
            observation["prior_image"] = write_image(output, f"{case.id}-prior", (("Before contact", before),))
            observation["image"] = write_image(output, case.id, (("At pinch", pinch), ("End of test lift", after)))
        traces = {"first": first}
        retries = {}
        release = self.move("lower", .88, self.grasp, FIRST_SLIDE_M)
        release += self.move("release", .64, self.grasp, 0)
        release += self.move("settle", .24, self.grasp, 0)
        traces["set_down"] = release
        branch_point = self.snapshot()
        for name, (grasp, lift, slide) in self.poses.items():
            self.restore(branch_point)
            reposition = self.move("reposition", .72, grasp, 0)
            samples, retries[name], _ = self.attempt(grasp, lift, slide)
            traces[name] = reposition + samples
        successful = [name for name, result in retries.items() if result["success"]]
        label = "continue" if first_outcome["success"] else ("regrasp" if successful else "skip")
        outcome = {"id": case.id, "case_truth": asdict(case), "label": label,
                   "first": first_outcome, "retries": retries, "successful_retries": successful}
        return observation, outcome, {"id": case.id, "traces": traces}


def orange_center(rgb):
    """Observe the orange shell in image pixels using a color mask."""
    rgb = rgb.astype(np.int16)
    r, g, b = np.moveaxis(rgb, -1, 0)
    ys, xs = np.nonzero((r > 135) & (r - g > 37) & (g - b > 20))
    return (float(np.median(xs)), float(np.median(ys))) if len(xs) >= 25 else None


def summarize(first, before, pinch, after):
    windows = {"pinch": first[26:32], "early_lift": first[39:45], "end_hold": first[-HOLD_SAMPLES:]}
    arm, tactile = {}, {}
    for phase, window in windows.items():
        force = np.asarray([sample["net_force_N"] for sample in window])
        pad_fz = force[:, :, 2].mean(axis=0)
        shear = np.linalg.norm(force[:, :, :2], axis=-1).mean(axis=0)
        contact_x = [[sample["contact_x_mm"][pad] for sample in window
                      if sample["contact_x_mm"][pad] is not None] for pad in range(2)]
        tactile[phase] = {
            "normal_N_lr": vector(pad_fz, 3), "shear_N_lr": vector(shear, 3),
            "contact_x_mm_lr": [round(float(np.median(v)), 2) if v else None for v in contact_x],
            "normal_sd_N_lr": vector(force[:, :, 2].std(axis=0), 3),
        }
        arm[phase] = {
            "gap_mm": round(float(np.mean([sample["gap_mm"] for sample in window])), 2),
            "elbow_Nm": round(float(np.mean([sample["arm_torque_Nm"][2] for sample in window])), 3),
            "joint_tracking_error_deg": round(float(np.max(np.abs(np.rad2deg(
                np.asarray(window[-1]["command_rad"]) - window[-1]["joint_rad"])))), 2),
        }
    vision = None
    if before is not None:
        start = [orange_center(rgb) for rgb in before]
        a = [orange_center(rgb) for rgb in pinch]
        b = [orange_center(rgb) for rgb in after]
        vision = {"shell_center_before_px_lr": [vector(p, 2) if p else None for p in start],
                  "shell_rise_px_lr": [round(p[1] - q[1], 2) if p and q else None
                                       for p, q in zip(a, b, strict=True)]}
    return {"arm": arm, "tactile": tactile, "vision": vision}


def write_image(output, identifier, views):
    relative = f"images/{identifier}.jpg"
    canvas = Image.new("RGB", (640, 202 * len(views)), "#161621")
    draw = ImageDraw.Draw(canvas)
    for row, (name, images) in enumerate(views):
        for col, (side, image) in enumerate(zip(("Left RGB", "Right RGB"), images, strict=True)):
            x, y = col * 320, row * 202
            draw.text((x + 6, y + 5), f"{name} / {side}", fill="white")
            canvas.paste(Image.fromarray(image).resize((320, 180), Image.Resampling.LANCZOS), (x, y + 22))
    canvas.save(output / relative, quality=87, optimize=True)
    return relative


def run(output, no_images=False, limit=None):
    output.mkdir(parents=True, exist_ok=True)
    if not no_images:
        (output / "images").mkdir(exist_ok=True)
    cases = case_list()[:limit]
    setup = {"mujoco_version": mj.__version__, "numpy_version": np.__version__,
             "step_s": STEP_S, "first_slide_m": FIRST_SLIDE_M, "retry_actions": RETRY_ACTIONS,
             "hold_s": STEP_S * HOLD_SAMPLES, "min_lift_mm": MIN_LIFT_MM, "max_tilt_deg": MAX_TILT_DEG,
             "taxel_ids": [point["index"] for point in POINTS], "axis_order": ["Fx", "Fy", "Fz"],
             "finger_order": ["left", "right"], "normal_positive": "compression"}
    (output / "protocol.json").write_text(json.dumps(setup, indent=2) + "\n")
    runner = ProbeRunner()
    try:
        with ((output / "observations.jsonl").open("w") as observations,
              (output / "outcomes.jsonl").open("w") as outcomes,
              gzip.open(output / "raw-traces.jsonl.gz", "wt", compresslevel=5) as raw):
            for case in cases:
                observation, outcome, trace = runner.run(case, output, frames=not no_images)
                for file, record in ((observations, observation), (outcomes, outcome), (raw, trace)):
                    file.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
                    file.flush()
                print(case.id, outcome["label"], outcome["successful_retries"], flush=True)
    finally:
        runner.sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--no-images", action="store_true", help="Run physics without an OpenGL renderer")
    parser.add_argument("--limit", type=int, help="Stop after a small number of cases for a smoke test")
    args = parser.parse_args()
    run(args.output, args.no_images, args.limit)
