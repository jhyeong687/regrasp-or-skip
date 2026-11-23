"""Run tactile decisions against MuJoCo on a two-station indexing belt."""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from latch.run import SEED, camera, chamfer_box, load_model
from sim import COMPRESSION_POSITIVE, POINTS, SETUP, Sim

FAR = .250
NEAR = .170
PROBE = (0, 22)
RATES = {"slow": .025, "fast": .055}
PITCH = .165
ACTIONS = ("retry", "wait", "skip")
SHAPES = {
    "stepped": [((-.014, 0, .012), (.017, .010, .012)),
                ((.014, 0, .020), (.014, .019, .020))],
    "elbow": [((.015, 0, .016), (.018, .009, .016)),
              ((-.013, .005, .019), (.015, .018, .019)),
              ((-.024, -.011, .009), (.008, .012, .009))],
    "wedge": [((-.011, 0, .013), (.021, .012, .013)),
              ((.011, 0, .020), (.019, .018, .020))],
}
COLORS = {"stepped": ".89 .52 .21 1", "elbow": ".27 .65 .63 1", "wedge": ".80 .32 .28 1"}


@dataclass(frozen=True)
class Part:
    name: str
    shape: str
    mass: float
    friction: float


PARTS = (
    Part("stepped-light", "stepped", .052, .80),
    Part("stepped-loaded", "stepped", .350, .72),
    Part("elbow", "elbow", .088, .65),
    Part("wedge", "wedge", .075, .47),
)


def make_scene(robot_scene, part, markings=True):
    def customize(xml):
        world, asset = xml.find("worldbody"), xml.find("asset")
        xml.set("model", "SO-ARM101 indexing conveyor")
        if markings:
            for x in np.linspace(.086, .412, 20):
                ET.SubElement(world, "geom", type="box", pos=f"{x} 0 .0002", size=".00065 .044 .0002",
                              material="vise", contype="0", conaffinity="0")
            for x in (.095, .187, .289, .407):
                for y in (-.050, .050):
                    ET.SubElement(world, "geom", type="cylinder", pos=f"{x} {y} .0091", size=".0014 .0005",
                                  material="vise", contype="0", conaffinity="0")
        ET.SubElement(xml.find("actuator"), "position", name="index_motor", joint="index", kp="1800", kv="45")
        carrier = ET.SubElement(world, "body", name="test_object", pos=f"{FAR} 0 .0065")
        ET.SubElement(carrier, "freejoint", name="object_free")
        elements = SHAPES[part.shape]
        weights = np.array([math.prod(size) for _, size in elements])
        weights /= weights.sum()
        for i, ((pos, dims), fraction) in enumerate(zip(elements, weights)):
            mesh = f"shell_{i}"
            name = "object_surface" if i == 0 else f"object_surface_{i}"
            chamfer_box(asset, mesh, *dims, .0025)
            ET.SubElement(carrier, "geom", name=name, type="mesh", mesh=mesh,
                          pos=" ".join(map(str, pos)), mass=str(part.mass * fraction),
                          rgba=COLORS[part.shape], contype="2", conaffinity="21", condim="4",
                          friction=f"{part.friction} .004 .0002")
        for side in ("left", "right"):
            parent = xml.find("contact")
            template = parent.find(f"pair[@name='{side}_tactile_pair']")
            template.set("friction", f"{part.friction} {part.friction} .004 .0002 .0002")
            for i in range(1, len(elements)):
                ET.SubElement(parent, "pair", {**template.attrib, "name": f"{side}_surface_{i}",
                                               "geom2": f"object_surface_{i}"})
        camera(world, "line_view", (.655, -.663, .450), (.133, -.043, .090), 35)
        camera(world, "part_view", (.338, -.300, .225), (.223, -.050, .020), 55)
        camera(world, "arm_top", (.242, -.045, .245), (.242, 0, .018), 38)
    return load_model(robot_scene, HERE / "track.xml", customize)


class Conveyor(Sim):
    def __init__(self, robot_scene, part, *, markings=True):
        self.model = make_scene(robot_scene, part, markings)
        self.data, self.fit = mj.MjData(self.model), mj.MjData(self.model)
        self.sites = [self.model.site(f"{side}_sensor_frame").id for side in ("left", "right")]
        self.skin_ids = {self.model.geom(f"{side}_skin").id: i for i, side in enumerate(("left", "right"))}
        self.renderers = {}
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.part = self.model.body("test_object").id
        self.elements = [(np.asarray(pos), np.asarray(dims)) for pos, dims in SHAPES[part.shape]]
        self.part_geoms = set(range(int(self.model.body_geomadr[self.part]),
                                    int(self.model.body_geomadr[self.part] + self.model.body_geomnum[self.part])))
        self.receiver = self.model.geom("receiver").id
        self.belt_geom = self.model.geom("belt_contact").id
        self.carrier_q = int(self.model.joint("index").qposadr[0])
        self.carrier = self.model.actuator("index_motor").id
        self.reset()

    def reset(self):
        mj.mj_resetData(self.model, self.data)
        home = self.pose(FAR, 0, .100)
        self.data.qpos[:5] = home
        self.command = np.r_[home, 0., 0., 0.]
        self.data.ctrl[:] = self.command
        self.trace = []
        self.frame_sink = None
        self.stage = "initial"
        mj.mj_forward(self.model, self.data)

    def hand(self, angles):
        self.fit.qpos[:5] = angles
        mj.mj_kinematics(self.model, self.fit)
        xyz = self.fit.site_xpos[self.sites].mean(axis=0)
        up = self.fit.site_xmat[self.sites[0]].reshape(3, 3)[:, 1]
        return np.r_[xyz, np.arctan2(up[0], up[2])]

    def pose(self, x, y, z):
        target = [x, y, z, -max(0.,z-.030)*7.0 - max(0.,.188-x)*4.0]
        angles = SEED.copy()
        delta = np.eye(5)[:4] * 1e-5
        for _ in range(10):
            current = self.hand(angles)
            jac = (np.array([self.hand(angles+step) for step in delta])-current).T / 1e-5
            angles[:4] += np.linalg.lstsq(jac, target-current, rcond=None)[0]
        limits = self.model.actuator_ctrlrange[:5]
        if np.linalg.norm(self.hand(angles)-target) > .0004 or np.any(angles < limits[:,0]) or np.any(angles > limits[:,1]):
            raise ValueError(f"IK target outside the current arm workspace: {target[:3]}, {angles.round(3)}")
        return angles

    def contact_with(self, geom):
        return any((c.geom1 == geom and c.geom2 in self.part_geoms)
                   or (c.geom2 == geom and c.geom1 in self.part_geoms) for c in self.data.contact)

    def _object_contact(self, geom1, geom2):
        return geom1 in self.part_geoms or geom2 in self.part_geoms

    def _surface_distance(self, points):
        local = (points - self.data.xpos[self.part]) @ self.data.xmat[self.part].reshape(3,3)
        # Display-only patch approximates each chamfered component by its authored box.
        distances = [np.abs(local - center) - size for center, size in self.elements]
        return np.min([np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(d.max(axis=1), 0)
                       for d in distances], axis=0)

    def sample(self):
        raw, patch, torque = self._sample_tactile()
        force = raw * COMPRESSION_POSITIVE
        net = force.sum(axis=1)
        return {
            "t": round(float(self.data.time), 3), "phase": self.stage,
            "pad_force_N": force.round(4).tolist(), "pad_resultant_N": net.round(4).tolist(),
            "pad_patch_N": (patch * COMPRESSION_POSITIVE).round(4).tolist(),
            "pad_torque_Nm": torque.round(6).tolist(),
            "gap_mm": round(float(82.4137 - 1000*self.data.qpos[5:7].sum()), 3),
            "elbow_Nm": round(float(abs(self.data.actuator_force[2])), 4),
            "torque_Nm": round(float(np.max(np.abs(self.data.actuator_force[:5]))), 4),
            "belt_mm": round(float(-1000*self.data.qpos[self.carrier_q]), 2),
            "part_xyz_mm_eval": (self.data.xpos[self.part]*1000).round(3).tolist(),
            "receiver_eval": self.contact_with(self.receiver),
            "belt_contact_eval": self.contact_with(self.belt_geom),
        }

    def move(self, phase, duration, angles=None, gap=None, belt=None):
        self.stage = phase
        start = self.command.copy()
        target = self.command.copy()
        if angles is not None:
            target[:5] = angles
        if gap is not None:
            target[5:7] = np.clip((82.4137-gap)/2000, 0, .036)
        if belt is not None:
            target[self.carrier] = belt
        steps = round(duration / self.model.opt.timestep)
        for i in range(steps):
            u = (i+1)/steps
            self.data.ctrl[:] = start + (target-start) * u**3 * (10 - 15*u + 6*u*u)
            self.data.ctrl[:5] += self.data.qfrc_bias[:5]/170
            mj.mj_step(self.model, self.data)
            tick = round(self.data.time*1000)
            if tick % 40 == 0:
                self.trace.append(self.sample())
            if self.frame_sink is not None and tick % 80 == 0:
                self.frame_sink(self.frame())
        self.command = target

    def first_probe(self):
        offset, gap = PROBE
        x = FAR+offset/1000
        self.move("settle", .32)
        self.move("approach", .56, self.pose(x, 0, .032), 78)
        self.move("touch", .48, gap=gap)
        self.move("test_lift", .40, self.pose(x, 0, .042))
        evidence = self.trace[-5:]
        self.move("lower", .24, self.pose(x, 0, .032))
        self.move("release", .36, gap=78)
        self.move("clear", .40, self.pose(FAR, 0, .101))
        net = np.array([x["pad_resultant_N"] for x in evidence])
        return {
            "pad_normal_N": net[:, :, 2].mean(axis=0).round(3).tolist(),
            "pad_shear_N": np.linalg.norm(net[:, :, :2], axis=2).mean(axis=0).round(3).tolist(),
            "elbow_Nm": max(x["elbow_Nm"] for x in evidence),
            "jaw_gap_mm": evidence[-1]["gap_mm"],
            "initial_command": {"offset_mm": offset, "gap_mm": gap},
        }

    def execute(self, action, offset, gap, rate):
        if action not in ACTIONS or not -18 <= offset <= 21 or not 14 <= gap <= 28:
            raise ValueError("Invalid belt action or grasp parameters")
        if action == "skip":
            self.move("bypass", .135 / rate, belt=-.135)
            return False
        x = FAR
        if action == "wait":
            self.move("index_to_near", (FAR-NEAR)/rate, belt=NEAR-FAR)
            x = NEAR
        x += offset/1000
        self.move("grasp_approach", .40, self.pose(x, 0, .032), 78)
        self.move("grasp_close", .40, gap=gap)
        self.move("lift", .44, self.pose(x, 0, .055))
        self.move("handoff", .52, self.pose(.184, -.137, .070))
        self.move("place", .24, self.pose(.184, -.137, .049))
        self.move("ungrip", .28, gap=78)
        self.move("withdraw", .20, self.pose(.184, -.137, .089))
        self.move("verify_receiver", .16)
        xyz = self.data.xpos[self.part]
        return bool(self.contact_with(self.receiver) and -.190 < xyz[1] < -.089 and .12 < xyz[0] < .24
                    and np.linalg.norm(self.data.qvel[-6:-3]) < .04)

    def cameras(self):
        feeds = [self._render(name, (320, 180)) for name in ("rgb_scene_left", "rgb_scene_right", "wrist_rgb")]
        depth = self._render("wrist_depth", (320, 180), "depth")
        valid = (depth >= SETUP["depth_min_m"]) & (depth <= SETUP["depth_max_m"])
        shade = np.clip(np.where(valid, depth-.195, .45)/.45, 0, 1)[..., None]
        image = (np.array([244, 194, 121]) * (1-shade) + np.array([38, 57, 102])*shade).astype("uint8")
        image[~valid] = [19, 24, 32]
        return np.column_stack((*feeds, image))

    def frame(self):
        bench = self._render("line_view", (864, 480))
        close = self._render("part_view", (416, 480))
        return np.column_stack((bench, close)), self.cameras()


def report_trial(bench, part, action, rate, offset=14, gap=18):
    bench.reset()
    evidence = bench.first_probe()
    start = float(bench.data.time)
    done = bench.execute(action, offset, gap, rate)
    return {"id": part.name, "rate": rate, "action": action, "elapsed_s": round(bench.data.time-start, 3),
            "placed": done, "evidence": evidence, "last": bench.trace[-1]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-scene", type=Path, default=HERE.parent/"scene.xml")
    args = parser.parse_args()
    for part in PARTS:
        bench = Conveyor(args.robot_scene.resolve(), part)
        for action in ACTIONS:
            r = report_trial(bench, part, action, RATES["slow"])
            print({k: v for k, v in r.items() if k!="last"}, r["last"]["part_xyz_mm_eval"], flush=True)
        bench.close()
