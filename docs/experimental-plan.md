# Experimental plan

## Research question

Can a robot choose among **continue**, **regrasp**, and **skip** from the evidence produced by a short test lift? A weak first lift may follow from a poor contact location, low traction, a pose-dependent arm limit or excessive load. The useful label depends on which actions remain feasible for this arm, route and safety limit.

The four-case pilot fixes the scripted motion. It gives a reproducible comparison where a squeeze threshold cannot distinguish the standard block from the slippery block. Regrasp policies and physical tests remain future experiments.

## Protocol and labels

1. Start at a stationary pick station. Build identical-looking shells with interchangeable mass inserts, surfaces and contact offsets. Vary mass, friction, grasp location and arm reach independently. Hold out shapes and surfaces.
2. Record a first pinch and a short low-height lift. Use the same force and motion budget for every variant. Allow a bounded grip-force change, a small grip-position shift or a change in approach angle as candidate second probes.
3. Establish recoverability by replaying a small pre-registered set of safe alternate grasps. A condition is recoverable if a permitted alternate clears the lift-and-hold criterion. A skip label means none succeeds within the declared route, budget and robot limits. Label inconclusive episodes explicitly.
4. Use external measurements or simulator state for scoring. Predeclare lift height and hold duration. Keep hidden mass, friction, actual object pose and privileged geometry out of controller inputs.

## Baselines and ablations

| Policy | Observation | Decision |
| --- | --- | --- |
| Vision + arm state | Same camera feeds, joint state and commanded/measured motion | Continue, regrasp once or skip |
| Force-threshold rule | Normal force plus same arm state | Fixed thresholds and retry budget |
| Fixed tactile probe | Full per-pad signal history | Same second probe in uncertain trials |
| Adaptive tactile probe | Full per-pad signal history | Select one bounded probe and use the result |
| Shuffled tactile control | Matched observations with tactile windows permuted across episodes | Measure reliance on aligned contact evidence |

Keep the controller, task budget, camera hardware and model checkpoint identical where applicable. Isolate additional sensing, extra probe opportunities and adaptive selection. If a simple controller matches the agent within uncertainty at the same budget, report that outcome.

**Metrics.** Useful picks per minute, false skips of recoverable items, failed or unsafe retries, drops, time to verdict and number of extra probes. Report joint-limit/overload events separately. Compare matched trial IDs, stratify by object and arm pose, and report counts with uncertainty intervals. Introduce a conveyor queue after the action-level comparisons.

## Sim-to-real

1. Zero both pads and verify units, axis signs and taxel order using known loads at marked locations. Measure timestamp and output rates and confirm which torque channels the exact communication board exposes.
2. Align robot and tactile clocks from contact markers and calibrate the mounted cameras. Save raw channels for later feature extraction.
3. Collect matched normal and low-friction trials on small, known loads. Check force magnitude, timing, pressure center and transient changes. Fit filtering or quantization on calibration trials and retain held-out evaluation trials.
4. Collect weak grips and recoverable offsets. The 600 g condition belongs to the simulation pilot. Physical load bounds come from measured safe operation with the mounted equipment.

PaXini specifies force arrays, resultants and torque. Pressure-center change and shear-to-normal ratio are measurable slip cues whose predictive value must be checked on independent episodes. The simulator's estimated contact patch uses privileged geometry, so the agent endpoint excludes it. [PaXini](https://paxini.com/robot/ax/gen3) · [Research on 3-D tactile arrays and incipient slip](https://ieeexplore.ieee.org/document/10616162)
