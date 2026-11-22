# References and asset credits

| Work | Relevance |
| --- | --- |
| [On Agentic Manipulation Policies](https://arxiv.org/abs/2611.09892v3) | Apple AMP1 motivated inspection of grasp failures and fixed left/right views. |
| [Training Reasoning Models in Simulation to Natively Use Real Robots](https://arxiv.org/abs/2611.18562) | PEAR informs the sim-to-real framing and controlled embodied actions. |
| [Democratizing Robot-Use Agents](https://arxiv.org/abs/2611.01560) | OpenRUA is a reference for holding the runtime/tool interface fixed during an ablation. |
| [Robotics Antigravity Plugin](https://github.com/google-deepmind/robotics-antigravity-plugin) | A tooling reference. This pilot asks what physical evidence should govern a recovery action. |
| [OmniVTLA and ObjTac](https://readerek.github.io/Objtac.github.io/) | Force-based tactile representation and tactile/vision/language pairs. ObjTac uses PaXini Gen2. |
| [Prior VTLA work](https://github.com/jhyeong687/VTLA) | Tactile observations on the SO-101 platform. |

## Asset credits

- The [SO-ARM100/SO-ARM101 CAD repository](https://github.com/TheRobotStudio/SO-ARM100/tree/main/STEP/SO101) supplies original arm and servo reference geometry under Apache-2.0. Meshes were tessellated and aligned to model coordinates. The [MuJoCo Menagerie SO-101 model](https://github.com/google-deepmind/mujoco_menagerie/tree/370c43965323c0944b433b202aac5058b5f907a9/robotstudio_so101) supplies the basis for kinematics and collision primitives. [Apache-2.0 terms](../licenses/MENAGERIE-LICENSE.txt)
- The [ggao50 parallel gripper](https://github.com/ggao50/SO101-Parallel-Gripper/tree/b0614b72c62e3891b1f7faa8b06c6eb3d490a9c2) is the mechanical base of the R4 film. That revision has no explicit redistribution license. Its original and modified rack/gear solids are not bundled. The public simulator provides visual proxies for six parts and retains the pad collision meshes and simulated dynamics.
- [PaXini PX-6AX GEN3](https://paxini.com/robot/ax/gen3) and the [M3025 model page](https://mall.paxini.com/product/itpu/6894488c74e45ce616705b80?skuId=68946e6a74e45ce61670752a) inform the reference geometry and reported units. The locations of the 77 points come from the [paxini-sdk registry](https://github.com/Jingyi-Z/paxini-sdk/blob/79c82145f88e158dd7706736e8c07f14f11cbd7a/paxini_sdk/data/pxsr_stdcp03b_points.json) under its [MIT license](../licenses/SDK-LICENSE.txt).
- The [RealSense D400 Series datasheet](https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf) and [RoboSEasy product page](https://roboseasy.ai/products/a-ba-max) inform the camera reference geometry. Measure actual intrinsics, extrinsics, connectors and near-field depth after mounting.
- The R4 film, synchronized figures, mounting accessories and experiment logs were produced for this project.
