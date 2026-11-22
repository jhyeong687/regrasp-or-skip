# Reproduce the pilot

The standalone kit contains the Python simulator and all required model meshes, four MuJoCo trial conditions, synchronized images, CSV and JSON logs, and camera mount print files. The system supports local simulation without a robot connection.

## 1. Open the dashboard

Download the [reproduction kit](https://github.com/jhyeong687/regrasp-or-skip/releases/download/v0.2.2/so101-tactile-recovery-kit.zip) and extract it. You can also use Git.

    git clone https://github.com/jhyeong687/regrasp-or-skip.git
    cd regrasp-or-skip

Create a Python environment from the extracted or cloned folder. The published build was verified with Python 3.12 and MuJoCo 3.17.

    python -m venv .venv
    source .venv/bin/activate
    python -m pip install -r simulation/requirements.txt

On Windows PowerShell, activate the environment with `.\.venv\Scripts\Activate.ps1`.

    python simulation/sim.py --verify --physics-only
    python simulation/sim.py --port 8770

Open [http://127.0.0.1:8770/](http://127.0.0.1:8770/). The browser preview requires working MuJoCo graphics support. Use `--verify --physics-only` and the included CSV/JSON to reproduce the measurements on a machine without a rendering context.

| Dashboard control | Meaning |
| --- | --- |
| **4개 비교 재생** | Synchronize and compare the four conditions |
| **집기 확대** | Switch the robot preview to a jaw close-up |
| **접촉면 추정 / 접촉점 기반** | Compare surface estimate and contact-point force |
| **센서 JSON** | Export the visible checkpoint and all available tactile features |

## 2. Replay the data or connect a local controller

    python scripts/record_trials.py --output replay-data

Compare `replay-data/trial-summary.json` with [the reference checkpoint](../data/trial-summary.json). The CSV contains the time series. The JSON contains two 77-point arrays with three force components per point. [Units and field definitions](../data/README.md)

With the dashboard running, request the current tactile observation.

    curl http://127.0.0.1:8770/agent_tactile

The response includes signed force vectors, sum force and torque on each side, and short-window tactile features. The browser controls the scripted trajectory. An external tool that handles robot actions would need its own motion interface and hardware calibration.

## 3. Inspect and print the CAD hardware

Import [mount-assembly.step](../cad/rgb-26cm/mount-assembly.step) into Fusion 360 to inspect the colored 26 cm mount. Open the [STL directory](../cad/rgb-26cm/STL) in a slicer with millimetre units. This release includes the two rails, four mast sections, camera interface parts, the [wrist D435i bridge](../cad/wrist-and-fit/D435i_Wrist_Bridge_R5.stl), and the [M3025 clearance coupon](../cad/wrist-and-fit/M3025_Screw_Fit_Test.stl). See [assembly and fasteners](../cad/README.md) and [the Korean guide](../cad/rgb-26cm/assembly.ko.md). The original rack-and-pinion hardware and applicable terms are linked in the [source credits](sources.md).

Download the [Fusion animation](https://github.com/jhyeong687/regrasp-or-skip/releases/download/v0.2.2/so101-fusion-assembly.mp4) for an overview of part placement. Check physical fit, wiring and limiting motor loads before running the arm.
