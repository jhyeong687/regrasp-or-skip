"""R7 inspection fixture with an independent flush cover and a bistable internal slider."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw

from run import Bench, HERE, POINTS, SEED, SETUP, COMPRESSION_POSITIVE, camera, load_model

VIEWS = ("rgb_scene_left", "rgb_scene_right", "wrist_rgb", "wrist_depth")
SETUP_REVISION = "r7-bounded-grasp"
OBSERVATION_DZ = .145
OBSERVATION_PITCH = np.radians(-60)
PROBE_PITCH = np.radians(-15)
HALF_STROKE = .003


def flush_geometry(xml):
    fixture = xml.find(".//body[@name='fixture']")
    cover = fixture.find("body[@name='cover']")
    for name in ("guide_left", "guide_right", "bolt"):
        fixture.remove(fixture.find(f"*[@name='{name}']"))
    for name in ("ramp", "tongue", "guide_peg"):
        cover.remove(cover.find(f"geom[@name='{name}']"))
    cover.find("joint[@name='cover_down']").attrib.update(range="-.010 .005", stiffness="95", damping="1")
    cover.find("joint[@name='cover_side']").set("range", "-.005 .005")
    slider = ET.SubElement(fixture, "body", name="detent", gravcomp="1")
    ET.SubElement(slider, "inertial", pos="0 0 0", mass=".020", diaginertia=".000002 .000002 .000002")
    ET.SubElement(slider, "joint", name="detent_z", type="slide", axis="0 0 1",
                  range="-.007 .001", damping="2.5", solreflimit=".004 1")

    def inner(body, name, pos, size):
        ET.SubElement(body, "geom", {"class":"mechanism","name":name,"type":"box",
                      "group":"4","pos":pos,"size":size,"material":"brass"})
    # The plunger presses a bistable plate. Its fork surrounds the cap pin only in the lower well.
    inner(cover, "plunger", "0 .014 -.004", ".007 .003 .001")
    inner(slider, "strike", "0 .014 -.0065", ".010 .005 .001")
    inner(cover, "feel_pin", "0 -.010 -.012", ".001 .002 .0012")
    for sign in (-1, 1):
        inner(slider, f"fork_{sign}", f"{sign*.0031} -.010 -.006", ".00165 .004 .0025")
    # Second interface limits upward travel in the retained position.
    inner(cover, "hook_tongue", ".015 0 -.0115", ".002 .002 .001")
    inner(slider, "hook_lip", ".015 0 -.003", ".003 .003 .001")
    # External shell groups remain visible; hide the internal metrology/collision group in every camera.
    camera(xml.find("worldbody"), "gripper_check", (.44, -.105, .25), (.202, 0, .086), 32)


class FlushBench(Bench):
    def __init__(self, scene, *, revised=True):
        self.revised = revised
        self.observation_dz = OBSERVATION_DZ if revised else .022
        self.model = load_model(scene.resolve(), customize=flush_geometry)
        self.data, self.fit = mj.MjData(self.model), mj.MjData(self.model)
        self.renderers, self.option = {}, mj.MjvOption()
        self.option.geomgroup[3:5] = 0
        self.option.sitegroup[:] = 0
        self.sites = [self.model.site(f"{side}_sensor_frame").id for side in ("left","right")]
        self.skin_ids = {self.model.geom(f"{s}_skin").id:i for i,s in enumerate(("left","right"))}
        self.object_geom = self.model.geom("object_surface").id
        self.round_object = False
        self.fixture = self.model.body("fixture").pos.copy()
        joint = self.model.joint("detent_z")
        self.q, self.dof = int(joint.qposadr[0]), int(joint.dofadr[0])
        self.home = self.pose(0, self.observation_dz)
        self.sink = None

    def pose(self, dx, dz):
        if not self.revised:
            return super().pose(dx, dz)
        pitch = OBSERVATION_PITCH if dz == self.observation_dz else PROBE_PITCH
        target = np.array([self.fixture[0] + dx, self.fixture[2] + .026 + dz, pitch])
        angles = SEED.copy()
        delta = np.eye(5)[1:4] * 1e-5
        for _ in range(12):
            current = self.hand(angles)
            if np.linalg.norm(target - current) < 1e-8:
                break
            jacobian = (np.array([self.hand(angles + step) for step in delta]) - current).T / 1e-5
            step = np.linalg.solve(jacobian, target - current)
            angles[1:4] += step / max(1., np.linalg.norm(step) / .65)
        low, high = self.model.actuator_ctrlrange[:5].T
        margin = .035
        if np.linalg.norm(self.hand(angles) - target) > 1e-6 or np.any(angles < low + margin) or np.any(angles > high - margin):
            raise ValueError("Fixture pose cannot be reached within the actuator limits")
        return angles

    def engaged(self):
        return bool(self.data.qpos[self.q] < -HALF_STROKE)

    def reset(self, state, variant):
        self.variant = variant
        self.rng = np.random.default_rng(900 + variant)
        self.bias = self.rng.normal(0, 12, 5)
        self.drift = np.zeros(5)
        self.model.jnt_stiffness[self.model.joint("cover_side").id] = [48,43,54,50][variant]
        for side in ("left","right"):
            self.model.pair_friction[self.model.pair(f"{side}_tactile_pair").id,:2] = [.8,.73,.87,.77][variant]
        self.detent_gain = [2.8,2.55,3.05,2.7][variant]
        mj.mj_resetData(self.model,self.data)
        self.data.qpos[:5] = self.home
        self.data.qpos[self.q] = -.006 if state == "engaged" else 0
        self.command = np.r_[self.home,0.,0.]
        self.data.ctrl[:] = self.command
        self.last_state = self.engaged()
        self.transitions, self.samples = [], []
        mj.mj_forward(self.model,self.data)
        self.move(0, self.observation_dz, 0, .32, "reset")
        if self.transitions or self.engaged() != (state=="engaged"):
            raise RuntimeError("Initial detent unstable")

    def move(self, x, z, close, seconds, phase):
        target = np.r_[self.pose(x,z),close,close]
        start = self.command.copy()
        n = round(seconds/self.model.opt.timestep)
        for i in range(n):
            u=(i+1)/n
            self.data.ctrl[:] = start+(target-start)*u**3*(10-15*u+6*u*u)
            self.data.ctrl[:5] += self.data.qfrc_bias[:5]/170
            # Smooth double-well potential. The same conservative law is applied to both initial states.
            v=(self.data.qpos[self.q]+HALF_STROKE)/HALF_STROKE
            self.data.qfrc_applied[self.dof] = -self.detent_gain*v*(v*v-1)
            mj.mj_step(self.model,self.data)
            state = self.engaged()
            if self.last_state != state:
                self.transitions.append([round(self.data.time,3),phase,state])
                self.last_state=state
            tick=round(self.data.time*1000)
            if tick % 40 == 0:
                self.samples.append(self.sense(phase))
                if self.sink and tick%200==0:
                    self.sink(self,self.samples[-1])
        self.command=target

    def sense(self,phase):
        mj.mj_forward(self.model,self.data)
        raw,_,_=self._sample_tactile()
        net=raw.sum(axis=1)
        axes=self.data.site_xmat[self.sites].reshape(2,3,3)
        force=np.einsum("pij,pj->i",axes,net)
        self.drift=.8*self.drift+self.rng.normal(0,5,5)
        current=5*np.rint((1000*self.data.actuator_force[:5]/.85+self.bias+self.drift)/5)
        return {"t":round(self.data.time,3),"phase":phase,"current_mA":current.astype(int).tolist(),
                "tcp_xz_mm":(1000*self.hand(self.data.qpos[:5])[:2]).round(2).tolist(),
                "gap_mm":round(float(82.4137-1000*self.data.qpos[5:7].sum()),2),
                "pad_base_N":force.round(4).tolist(),
                "pads":(raw*COMPRESSION_POSITIVE).round(4).tolist()}

    def probe(self, a):
        x,z,dx,dz=np.array([a[k] for k in ("x_mm","z_mm","dx_mm","dz_mm")],dtype=float)/1000
        gap=a["gap_mm"]
        if not np.all(np.isfinite([x,z,dx,dz,gap])) or not (-.008<=x<=.008 and -.005<=z<=.005
                and abs(dx)<=.004 and -.009<=dz<=.004 and 23<=gap<=31):
            raise ValueError("Requested contact exceeds the published workspace")
        slide=(82.4137-gap)/2000
        initial=len(self.samples)
        if self.revised:
            self.pose(x, z)
            self.pose(x + dx, z + dz)
        self.move(x,z,0,.85 if self.revised else .55,"approach")
        self.move(x,z,slide,.55,"pinch")
        self.move(x+dx,z+dz,slide,.55,"probe")
        self.move(x+dx,z+dz,slide,.20,"hold")
        self.move(x,z,slide,.42,"return")
        self.move(x,z,0,.36,"open")
        self.move(0,self.observation_dz,0,.85 if self.revised else .55,"withdraw")
        return self.samples[initial:]

    def feeds(self, w=320,h=180):
        views=[self._render(n,(w,h)) for n in VIEWS[:3]]
        depth=self._render(VIEWS[3],(w,h),"depth")
        valid=np.isfinite(depth)&(depth>=SETUP["depth_min_m"])&(depth<=SETUP["depth_max_m"])
        frac=np.clip((depth-.195)/.5,0,1)[...,None]
        color=np.array([95,222,210])*(1-frac)+np.array([43,60,100])*frac
        color[~valid]=[20,23,34]
        return [*views,color.astype("uint8")]

    def view(self):
        l,r,w,d=self.feeds(480,270)
        return Image.fromarray(np.vstack((np.hstack((l,r)),np.hstack((w,d)))))
