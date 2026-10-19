# 10-object fixed-base revolute benchmark — 2026-10-11

**KNOWN_MODEL_DIAGNOSTIC. Full-task success: 0/10.** Planning produced executable candidates for 3/10 registered objects. Exactly one official normal-closed-start contact episode was launched for each of those three; the other seven remain planning failures in the denominator. No failed asset was replaced and no official physical trial was repeated.

Initial physical grasp confirmation succeeded for **2/3 attempted episodes (66.67%)**, or **2/10 registered cases (20%)**. Confirmation means the existing frozen physical closure/contact rule; 47686 established that contact but did not produce useful opening. Only 45621 produced substantial single-grasp opening, reaching **52.17298° / 90.72° (57.51%)**. No result is a full-range success.

Mean normalized completion across all ten: **5.81387%**. Median: **0%**. Normalized completion is maximum actual angle / authored target, capped at 100%; planning failures contribute zero completion without claiming a measured physical angle. The small 45091 opening during unsafe closure is retained as partial measured range, while grasp and full-task success remain false.

| Asset | Target | Max Angle | Target Reached | Grasp Success | Range Achieved | Failure Reason |
|---|---|---|---|---|---|---|
| 35059 | 162.00° | — | 否 | 未执行 | 0.00% | 规划：可解 IK 抓取仍与原碰撞几何相交 |
| 41004 | 90.00° | — | 否 | 未执行 | 0.00% | 规划：可解 IK 抓取仍与原碰撞几何相交 |
| 45091 | 90.00° | 0.57° | 否 | 否 | 0.63% | 闭合时 finger-back/root 接触保护 |
| 45166 | 97.20° | — | 否 | 未执行 | 0.00% | 规划：部分运动弧可行，但无可执行初始接近方案 |
| 45621 | 90.72° | 52.17° | 否 | 是 | 57.51% | 原有 0.05 rad 指令关节余量边界 |
| 46744 | 90.00° | — | 否 | 未执行 | 0.00% | 规划：可解 IK 抓取仍与原碰撞几何相交 |
| 47686 | 120.00° | <0.001° | 否 | 是 | 0.00% | 有效开门近零；最终 CONTACT_LOST |
| 7236 | 90.00° | — | 否 | 未执行 | 0.00% | 现有几何抓取器未生成接触截面 |
| 7306 | 90.00° | — | 否 | 未执行 | 0.00% | 现有几何抓取器未生成接触截面 |
| 103082 | 57.30° | — | 否 | 未执行 | 0.00% | 现有几何抓取器未生成接触截面 |

“— / 未执行” means no official contact episode, not a fabricated zero-angle physical attempt. 47686's raw maximum was 1.18782e-7 degrees, effectively no opening; its recorded value is preserved in the machine-readable report.

## Physical episode evidence

| Asset | Simulation s | Physical-run wall s | Peak pad loads L/R N | Existing stop |
|---|---|---|---|---|
| 45091 | 32.062 | 173.468 | 0.041 / 0.122 | RuntimeError('FINGER_BACK_OR_ROOT_HANDLE_LOAD:gripper_link1') |
| 45621 | 253.487 | 1131.317 | 0.594 / 1.098 | RuntimeError('EXISTING_COMMAND_JOINT_MARGIN') |
| 47686 | 146.908 | 796.946 | 0.601 / 2.061 | RuntimeError('CONTACT_LOST_PAUSE_IN_SAME_SCENE') |

Physical simulation durations sum to **432.458 s**. Physical-run wall durations sum to **2101.731 s**; they overlap because scenes ran on separate GPUs. Observed batch elapsed wall time, including setup, planning and the preparation repair, was **1583.902 s (26.40 min)**. Physical wall time is measured from the shared scene setup entry through raw export, before app closure; cleanup files separately measure full subprocess lifetime.

45621: one physical grasp, zero post-grasp base moves, zero releases/regrasps. It continued beyond the offline predicted 49.75° interval, and stopped at the inherited command margin boundary. Last recorded actual margin was approximately 0.05011 rad. Neither millimeter drift nor single-pad zero readings stopped its progress. Maximum loaded TCP velocity was 0.801 mm/s; maximum relative drift 3.008 mm / 1.118°. There was no interval without positive target-pad impulse after acquisition. Initial confirmation loads and contact events are in the per-case metrics. This is partial opening, not an authored-target or mechanical-limit result.

47686: existing bilateral grasp confirmation succeeded, but the authored hinge remained effectively closed. Relative drift reached 17.57 mm / 1.825°, followed by the existing `CONTACT_LOST_PAUSE_IN_SAME_SCENE` condition. Peak right-pad load was 2.061 N; that raw peak was not the stop reason. The post-grasp contact-impulse gap at the stopping interval was approximately 79.17 ms. No recovery/regrasp was performed because the official benchmark permits one grasp only.

45091: closure triggered `FINGER_BACK_OR_ROOT_HANDLE_LOAD:gripper_link1`. The small 0.566° excursion occurred before a successful grasp. Evidence is retained and no alternate physical closure was attempted.

## Failure categories

Exclusive report categories: **collision 4, IK/initial-path/joint-margin 2, contact loss 1, perception/grasp generation 3, speed protection 0, other official failures 0**. Three collision cases failed offline at reachable grasp candidates; one collision case failed during real closure. IK/path cases comprise 45166's initial approach planning failure and 45621's existing joint-margin boundary. Full blocker histograms are retained in `BENCHMARK_RESULTS.json`; large counts of unreachable base candidates do not by themselves prove an object is globally unreachable.

45166 had some feasible sampled articulation arc (up to 29.45°), but the bounded shared search did not return a collision-free home/pre-grasp/approach candidate. No physical manipulation was claimed for that offline arc. Three grasp-generation failures returned no contact sections from the existing dataset geometry ranking; its old screening reason labels were logged, never added as new execution gates.

## Registered sample and common mechanism

The fixed IDs were registered at 2026-10-11 13:29:06 +08:00, before planning/physical outcomes: 35059, 41004, 45091, 45166, 45621, 46744, 47686, 7236, 7306, 103082. The sample covers differing dimensions, left/right vertical hinges, horizontal hinges, a slanted hinge, and authored targets 57.30°–162°. It is a selected diverse available-asset sample, not a random population estimate. Named development assets 7320, 41529, 45007 and 7138 were excluded. Geometry, origins and source hashes are in `REGISTERED_OBJECTS.json`.

All ten cases used one common planning pipeline and per-object geometry inputs. The **three actual contact episodes used byte-identical shared execution code**; seven cases did not reach execution, so this report does not claim ten physical executions. `run_revolute10_fixed.py` reuses the previous Pinocchio/Pink opening controller literally from the archived runner and the unchanged frozen initial approach/closure/contact code. The minimal adapters supply the authored angular target and identify each original handle-region collider by its importer-preserved mesh/body identity. There are no object-ID controller branches or ten duplicated controllers.

Initial SE(2), wrist and IK-branch candidates came from the current object's geometric contact sections, the existing base candidate family, and the existing Pink solver. A shared 300 s planning search budget applied to each supported case. Grasp geometry came from current-asset meshes, not saved successful-object grasp transforms. No new handle proxy was constructed. The base was normally initialized at the selected station, fixed after grasp; pre-grasp navigation itself was not demonstrated.

Original dataset URDF/collision geometry, masses, friction/inertia, controller gains, clamp effort, contact thresholds, temporal force policy, 0.05 rad margin and 5 mm/s world TCP protection were unchanged. `plant: {}` retains the existing dataset plant rather than adding a new resistance setting. Known articulation/moving-body feedback, collision models and geometric planning are GT dependencies. No visual/unknown-model, autonomous reconstruction, hardware or physics-identification success is claimed.

## Protocol repair, preservation and evidence

Before any official grasp, four offline preparation attempts encountered missing relative mesh-directory links in the asset adapter. Only read-only original directory links were added; original files and physical semantics did not change. Their errors/exports are archived under `preparation_error_mesh_paths`. Planning was rerun with the same algorithm/budget and no physical outcome feedback; no official attempt was repeated. This infrastructure repair is reported separately from object failures. No manual continuation, scene restart/stitching, attachments, object-state commands during manipulation or artificial object forces were used in the three official episodes.

All owned setup/planning/physics processes were closed after export. Final verification found no changes among 488 preserved 7320 files, the two frozen drawer source/config sets, or any of the ten original asset URDF/manifest hashes.

Each executed case preserves `contact_baseline.mp4`, 240 Hz lossless observation/physics chunks, native raw contact impulses/points/normals/collider IDs, raw/filtered pad loads, finger/TCP/robot/base state, effort/reaction APIs, references, phases, events, sensor conversion/frame metadata and source/config/plan hashes. Optional telemetry remained passive. The three observation/native counts are 7695/7695 (45091), 60837/60837 (45621), 35258/35258 (47686).

Full raw evidence stays on the server at `results/revolute10_fixed_20261011` and locally under `engineering_artifacts_20261011/revolute10_fixed`. Source/configuration/report/metadata are published; raw videos and telemetry are not uploaded to GitHub. `evidence_hashes.json` and archive verification identify the retained evidence.
