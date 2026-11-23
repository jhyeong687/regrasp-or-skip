"""Check whether mirroring a hidden center of mass reverses a stable placement."""

import itertools
import json

import mujoco as mj
import numpy as np


def settle(ballast_mm, placement_mm):
    scene = f"""
    <mujoco model="Mirrored internal ballast on a ledge">
      <option timestep=".001" integrator="implicitfast" iterations="60"/>
      <worldbody>
        <geom type="plane" size="1 1 .01"/>
        <geom name="ledge" type="box" pos="0 0 .037" size=".007 .022 .037" friction=".85 .005 .0001"/>
        <body name="top" pos="{placement_mm / 1000} 0 .0745">
          <freejoint/>
          <inertial pos="{ballast_mm / 1000} 0 .015" mass=".085" diaginertia=".000017 .000045 .000049"/>
          <geom name="shell" type="box" pos="0 0 .015" size=".034 .020 .015" friction=".85 .005 .0001"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mj.MjModel.from_xml_string(scene)
    data = mj.MjData(model)
    for _ in range(2500):
        mj.mj_step(model, data)

    body = model.body("top").id
    tilt = np.degrees(np.arccos(np.clip(data.xmat[body, -1], -1, 1)))
    supported = {model.geom(name).id for name in ("ledge", "shell")}
    touches = any({contact.geom1, contact.geom2} == supported for contact in data.contact)
    return {"hidden_center_mm": ballast_mm, "placement_mm": placement_mm,
            "tilt_deg": round(float(tilt), 2),
            "stable_on_ledge": bool(touches and tilt < 5 and np.linalg.norm(data.qvel) < .03)}


if __name__ == "__main__":
    trials = [settle(ballast, placement)
              for ballast, placement in itertools.product((-12, 12), (-12, 0, 12))]
    print(json.dumps({"mujoco": mj.__version__, "trials": trials}, indent=2))
