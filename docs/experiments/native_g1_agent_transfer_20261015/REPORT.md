# Frozen Agent wrist skill: two-object transfer

Reference **34607b4**. Scope: **CODEX_IN_THE_LOOP / WRIST_RGBD_SAM3 / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION**. Experiments started 2026-10-15/16 Shanghai time; directory date follows preregistration.

Both registered objects completed one fresh, normally closed, continuous-scene episode: actual arm observation motion → fresh wrist RGB-D → automatic grasp → bilateral acquisition → original one-second hold → approximately +3° physical opening. No physical retries, released grasps, state stitching, object actuation or GT grasp fallback. These are **two specific transfer demonstrations, not a population success-rate estimate**, and do not change the original ten-door benchmark or establish 30° success.

## Registration and controlled inputs

[Preregistration](REGISTERED_CASES.json) was committed as `6470d37` before new grasp/IK planning or physical outcomes. Same [Agent instructions](AGENT_INSTRUCTIONS.md), SAM prompt (`cabinet door handle`), 0.25 threshold, simulated camera mount, fusion, robust grasp scoring, closure, one-second hold, Pink policy, actuators, physics and protections for both.

| Case | Registered handle | Closed geometry used for selection | Authored range | Prior-use boundary |
|---|---|---|---|---|
| 45936 | `l_2 / original-39` | 0.9 m cabinet; ~296 mm straight vertical bar; source −X handle side | 100° | No recorded experimental development/planning/physics use found |
| 46427 | `l_0 / original-42` | 0.25 m wall cabinet; ~63 mm central shaft with curved mounting supports; ~144 mm observed assembly; source +X side | 90° | Same boundary |

Selection used source metadata and closed visual geometry, not planner/contact scores. Both appeared in a prior raw inventory; no claim of being unseen to SAM3 training. An uncommitted provisional 45526/45916 draft was withdrawn **before any new planning or trial** after discovering prior development use. No failed evaluated case was replaced.

Both cases were preregistered at the **same** external station copied from 45600: `[0.1960537602, -0.5328198024, -0.1, -0.7953990038]` (x/y/z m, yaw degrees). This copies **base only**, not a view or grasp. Object installation remains x=0.5 m, y=−0.05 m, yaw=−90°, fixture=0.04 m. Thus this tests visual Skill transfer under a known station, not autonomous station selection.

## Actual outcomes

| Asset | Wrist observations / Agent moves | Automatic plan / bilateral acquisition | Acquisition angle | Verified hold | Final actual angle | Post-acquisition gain | ≥2° short outcome |
|---|---|---|---|---|---|---|---|
| 45936 | 3 / 2 | Yes / yes | 0.031180° | 1.0042 s | 3.029704° | **+2.998523°** | PASS |
| 46427 | 3 / 2 | Yes / yes | 0.003039° | 1.0042 s | 3.004496° | **+3.001456°** | PASS |

**45936:** Initial SAM score 0.455, valid non-border mask, but actual RGB showed the upper handle occluded by the robot. Codex requested a measured-cue-centered elevated view; visible cue extent grew from ~96 to ~296 mm, score 0.832. A lateral view added side/gap evidence, score 0.891. Three accepted observations produced 8,061 retained points / 24,994 open-patch triangles. The fused local extent remained ~105 mm despite the new full-length image: a recorded seed-propagation/retention limitation, not a new veto. Automatic selection: middle, depth 0, roll/pitch 0. Return and measured-state approach passed; original wrist refinement used 8,577 common-visible points.

**46427:** Initial score 0.922; whole handle visible near the image right side. Codex requested a centered higher view (0.898), then an opposing oblique. First oblique request was `NO_IK`; no motion occurred. A smaller request was feasible and physically executed (0.906). Three accepted views produced 8,031 points / 26,313 triangles, ~144 mm extent. Automatic selection: middle, +9.375 mm candidate-family depth, roll 180°, pitch 0; no manual pose offsets. Return and measured-state approach passed. Wrist refinement used 10,615 common points, 0.219 mm / 0.0187° correction. Predicted pad-tip contact was logged without adding a rejection gate.

Codex inspected each fresh RGB/mask/depth/cloud board, fused geometry and native-finger plan before commands. [45936 trace](45936/AGENT_TRACE.json) and [46427 trace](46427/AGENT_TRACE.json) include interpretations, missing information, requested targets, expected benefits, actual results and inspected-image hashes. Two executed adjustments each; one additional rejected view request on 46427. No saved successful view/grasp replay.

## Contact, tracking and protection evidence

| Metric | 45936 | 46427 |
|---|---:|---:|
| Acquisition L/R normal loads (N) | 0.483/0.510 | 0.571/0.420 |
| Peak recorded raw L/R pad load (N) | 1.067/1.163 | 0.815/1.054 |
| Peak filtered L/R pad load (N) | 1.060/1.131 | 0.790/1.049 |
| Maximum post-acquisition relative translation (mm) | 0.793 | 0.082 |
| Maximum post-acquisition relative rotation (deg) | 0.314 | 0.454 |
| Maximum all-phase TCP speed (mm/s) | 29.722 | 27.789 |
| Maximum TCP speed with positive allowed normal contact (mm/s) | 1.754 | 1.785 |
| Maximum post-acquisition TCP speed (mm/s) | 0.778 | 1.785 |
| Minimum all-phase joint margin (rad) | 0.403 | 0.868 |
| Maximum joint command tracking error (rad) | 0.00465 | 0.00471 |
| Maximum planned-TCP tracking error (mm) | 0.546 | 0.400 |
| Physical warning/stop events | 0 | 0 |

45936 had no post-acquisition positive-normal-report gaps. 46427 had one **left** zero-normal-report interval starting t=335.091684 s, duration **30.529168 s** (~0.50 s after opening started); right report remained positive. Both pads were established at acquisition, and the one-second hold preceded this interval. Door motion continued with small relative translation. This is not evidence of continuous bilateral normal loading, nor proof of true grasp loss. Relative TCP/moving-body change is a proxy, not direct material-slip measurement. All-phase maxima include unloaded observation/return; they must not be compared as if every phase had the loaded 5 mm/s limit. No original speed/force/collision/joint protection was relaxed.

### Independent contact forecast comparison

Fine geometric sweeps were performed **after each actor exited**, at its wrist-refined pose. No result entered execution. Values are L/R first-intersection apertures, mm:

| Asset | Observed + nominal uncertain rear | GT native geometry, offline only | Actual first positive normal contact | Actual first side |
|---|---|---|---|---|
| 45936 | 14.45/14.74 | 14.90/14.65 | 15.14/14.76 | left |
| 46427 | 9.76/9.81 | 9.15/10.20 | 9.68/11.29 | right |

45936 right contact and 46427 left contact occurred after original CENTER motion, so these are not fixed-pose sensor-error estimates. Execution tracking, contact offsets and centering also matter. Coarse planner/ranked apertures are different numerical approximations, not hard closure targets. Native normal contacts are partial wrench information; friction-anchor forces are unavailable. No measured pressure area, complete interaction-force estimate or hardware safety claim is made.

## Time and evidence integrity

| Recorded component | 45936 | 46427 |
|---|---:|---:|
| Simulation (s) | 329.4 | 365.6 |
| Actor total wall (s) | 2320.2 | 2446.4 |
| Sensing/SAM wall (s) | 98.1 | 56.1 |
| Observation robot motions wall (s) | 353.7 | 467.6 |
| Fusion wall (s) | 1.7 | 1.7 |
| Grasp planning wall (s) | 170.2 | 135.5 |
| Agent command wait wall (s) | 233.9 | 268.2 |

Measured actuator return/handoff wall times: **363.8 / 474.5 s**. Command wait includes image transfer/rendering, tools and scheduling, not pure LLM reasoning. Physics pauses during the frozen Agent inspection interface; the scene survives. Total actor time also includes initialization, approach, slow closure, opening and export. This is interactive Codex development, not a deployed autonomous latency result.

Physics-step and observation records: **79,059 / 79,059** and **87,749 / 87,749**, respectively, at native 240 Hz with uniform simulation intervals. Recorded raw contacts, raw/filtered pads, velocities, applied/measured efforts, reactions and native-gripper feedback are present in all observation records; presence does not certify physical calibration. Continuous videos: **329.4 s / 9,882 frames** and **365.6 s / 10,968 frames**, 1280×960 at 30 fps; both decoded without errors. Sensor definitions/conversions, raw points/normals/impulses/collider IDs, RGB-D, measured camera/q, commands and snapshots remain private. Camera/FK translation discrepancy maxima: **0.00131 / 0.00209 mm**, simulation diagnostics only.

## Integration amendment and frozen provenance

No Isaac initialization crash or physical retry occurred. One **new-input mapping error** was corrected before any new grasp planning/contact outcome: the initial preparation conflated active joint child with visual body. Final `moving_link` is `abstract_3_2` / `abstract_1_0`; `door_link` remains `l_2` / `l_0`. The first actor retained its already loaded visual frame, whose articulation delta was numerically identical (max matrix difference 0 over the short tested interval); fresh planners read the correct active axis. No scene reset, online pose correction, URDF/collider/physics/control change. Both cases received the same metadata correction before the first outcome. See [amendment](FRAME_INPUT_REPAIR.json) and [frame check](FRAME_EQUIVALENCE_CHECK.json). Do not describe this as an entirely unchanged-input experiment.

Existing generic importer retained per-visual convex hulls, source masses and prior box-COM/inertia approximation, with source-metadata scale. No concavity was silently fixed; compatibility is only established for these short interactions. Oracle handle identities label allowed contact/collision ownership; GT surfaces do not supply grasp/contact ranking.

Frozen runner `scripts/run_native_g1_agent_manipulation.py` SHA-256: `a161b0f92aa1f93ab0e1fb2bd51c7dbba8ab551ecde830978101cc0272840d62`. All **13 active source checks**, **22 frozen-baseline checks**, and supplied inspected-image checks match in each case. [Configuration comparison](CONTROL_CONFIG_COMPARISON.json) confirms unchanged visual/scoring/control settings. Thin input/launch changes only; frozen controllers were not duplicated or edited. Full source/config/generated-executable hashes: [45936](45936/PROVENANCE.json), [46427](46427/PROVENANCE.json).

Remaining online dependencies: common registered station, coarse cabinet envelope for initializer, known hinge and dynamic moving-body state, robot/camera calibration and GT scene collision checks. Sensor is a simulated rigid link7 mount without hardware validation. G1/arm actuators remain approximate simulation models. Legacy `oracle_handle` field names or 30° internal planning labels are not claims of oracle grasp generation or physical 30° execution.

## What transferred; next general capability

Detection, observation-driven wrist motion, metric fusion, automatic native-G1 planning, measured-state return/refinement, bilateral acquisition and short contact-driven opening transferred to both objects without controller tuning. 46427 additionally exercised a rejected viewpoint and an adequate reachable alternative. No end-to-end failure occurred in these two cases, so a population-dominant failure mode cannot be inferred.

Observed remaining limits are **coverage persistence** (45936's newly visible full length is not fully retained), viewpoint reachability at a fixed station, 46427's sustained asymmetric normal-contact report, and high wall cost of the frozen slow motion/closure. Recommend one next general-purpose capability: **coverage-aware, reachability-conditioned incremental wrist reconstruction**, preserving newly revealed surfaces beyond the initial seed and retaining explicit unknown regions. Keep the contact asymmetry as an unresolved diagnostic until additional wrench/retention evidence is available; do not infer a need to squeeze harder.

No new object, trial, controller, perception model, hinge estimator, collider change or full-range test was started. Both owned actors returned 0 and are absent after complete export: [cleanup](OWNED_PROCESS_CLEANUP.json). Raw evidence is indexed in [private evidence index](PRIVATE_EVIDENCE_INDEX.json); no raw archives/videos are published.
