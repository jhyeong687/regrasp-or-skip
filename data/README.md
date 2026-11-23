# Pilot data

| File | Contents |
| --- | --- |
| [trial-traces.csv](trial-traces.csv) | Full scripted paths for four scenarios at 40 ms intervals, with per-pad summary features, arm state and simulated ground truth. |
| [trial-summary.json](trial-summary.json) | Shared 8.90 s checkpoint reported in the README. |
| [tactile-pilot.json](tactile-pilot.json) | 40 checkpoints with a force array of shape `[2, 77, 3]` and full features per case. |
| [cad-clearance.json](cad-clearance.json) | Limit dimensions, camera optical frames, FOV checks and sampled CAD intersection outcome. |

## Channels and units

- Pad force order is **Fx, Fy, Fz** in newtons. Viewed face-on, +X points right, +Y points up and reported Fz is positive in compression. Left and right are in gripper order.
- Pad torque is **Tx, Ty, Tz** in N·m, about each pad's frame origin. The `pad_axes_world` rotation columns are geometric +X, +Y, +Z. Convert reported compression to the geometric-frame sign before rotating a force into world coordinates.
- `paxini_force_N` is the simulated discrete point distribution. `estimated_contact_patch_N` is a separate geometry-based visualization. Their sums agree. Compare feature history using the discrete array.
- `tactile_features` includes bilateral contact, relative normal load, active points, peak normal force, pressure center, shear-to-normal ratio and changes over about 100 ms. Empty fields indicate no valid contact or insufficient history. The calibrated hardware thresholds are still unknown.
- Fields with names such as `mass_g_truth`, `friction_truth`, `target_depth_m_truth` and `lift_mm_truth` are simulation labels. Use them to score an episode. The agent-only endpoint omits them.
- Repeated samples within a trial are correlated. This release contains one deterministic episode per condition.

Rebuild the files with `python scripts/record_trials.py` from the repository root.

The [bounded recovery pilot](../docs/recovery-pilot.md) adds scored retry branches and paired model conditions. Its [compact summaries](recovery-pilot) are in this repository, and the [run bundle](https://github.com/jhyeong687/tactile-grasp-agent/releases/download/v0.3.0/tactile-agent-pilot-data.zip) contains the original RGB inputs, tactile streams, branch outcomes, and model responses.
