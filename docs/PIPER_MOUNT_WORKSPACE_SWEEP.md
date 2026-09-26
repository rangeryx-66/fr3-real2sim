# Piper mounting-pose workspace sweep

This experiment changes only the rigid pose of the Piper base relative to the
frozen Arena scene. AnyGrasp outputs, candidate order, Piper kinematics,
gripper control, collision rules, and execution policy are unchanged.

## Frozen mount

The selected pose is stored in `config/piper_mount_arena_v1.json`:

| Parameter | Value |
|---|---:|
| base x | +0.1896875 m |
| base y | +0.0695679 m |
| base z | +0.08136 m |
| base yaw | -6.04373 deg |
| workspace shift | (0, 0, 0) m |

Coordinates are relative to the original Piper installation. A point in the
frozen nominal scene is expressed in the mounted base frame as
`T_base_nominal * p_nominal`, where `T_base_nominal` is implemented in
`src/workspace_mount.py`.

## Sweep protocol

- Inputs: the same 40 Arena scenes, seeds 1000--1039, and the exact same 40
  Piper AnyGrasp JSON files used by the original-mount run.
- Search variables: base x/y/z and yaw. The workspace shift remained zero.
- Coarse stage: 65 deterministic Halton samples.
- Refinement stage: 32 local samples around the four best coarse poses.
- Full stage: eight distinct finalists checked with the execution backend's
  IK, joint-limit, robot/world collision, pregrasp, Cartesian approach, global
  plan, attached-object micro-lift/lift, and frozen 4.5% hand geometry gate.
- Objective: maximize episodes with at least one executable candidate, then
  maximize total executable candidates.
- Physical validation: all 40 paired scenes, with success requiring a real
  lift of at least 8 cm and a hold of at least 2 seconds.

The full offline result for the frozen mount predicted 24/40 covered episodes
and 59/691 executable candidates. Physical validation produced 23/40 executed
episodes and 61/691 valid candidates.

## Results

All three runs use identical AnyGrasp grasp arrays, ranks, scores, and poses.
The original and mounted Piper JSON files are also byte-identical. Robot-frame
conversion is performed in memory.

| Robot/config | Pick success | Episodes executed | NO_IK | Executable candidates | Non-target contact | Disturbance | Mean planning |
|---|---:|---:|---:|---:|---:|---:|---:|
| FR3 paired baseline | 18/40 (45.0%) | 37/40 | 20/691 (2.9%) | 206/691 (29.8%) | 4/40 | 4/40 | 26.88 s |
| Piper original mount | 5/40 (12.5%) | 6/40 | 563/691 (81.5%) | 11/691 (1.6%) | 0/40 | 0/40 | 23.06 s |
| Piper frozen mount | 15/40 (37.5%) | 23/40 | 421/691 (60.9%) | 61/691 (8.8%) | 2/40 | 1/40 | 23.70 s |

The new mount reduces NO_IK by 142 candidates, or 20.6 percentage points
(25.2% relative). It increases executable-candidate coverage by 5.5x and
triples physical success. Piper finishes 3/40 trials, or 7.5 percentage points,
behind FR3.

| Object | FR3 | Piper original | Piper mounted |
|---|---:|---:|---:|
| mustard | 4/5 | 2/5 | 3/5 |
| raisin | 4/5 | 1/5 | 3/5 |
| hidden tuna | 0/5 | 0/5 | 0/5 |
| bowl | 0/5 | 0/5 | 0/5 |
| banana | 0/5 | 0/5 | 3/5 |
| sugar | 5/5 | 1/5 | 3/5 |
| soup | 5/5 | 0/5 | 3/5 |
| mug | 0/5 | 1/5 | 0/5 |

Mounted-Piper episode outcomes were: 15 `SUCCESS`, 11 `SCENE_COLLISION`,
7 `BAD_CONTACT`, 3 `INSUFFICIENT_PAD_OVERLAP`, 2 `NO_IK`, 1
`TABLE_COLLISION`, and 1 `APPROACH_FAIL`.

## Audit findings

Two harness defects were found before the formal run. Their outputs are not
included in the table above.

1. The first offline sweep omitted Piper's two physical finger state
   variables. MoveIt then evaluated a default finger configuration and
   under-counted finger/table collisions. The sweep now builds the same full
   nine-joint state returned by the simulator.
2. `ArenaBackend.reset_scene()` overwrote the mounted table with its nominal
   PlanningScene pose. This caused false online finger/table collisions. The
   table now uses the same rigid mount transform as the physical table,
   target, obstacles, and camera.

The first online smoke after both corrections selected rank 12 for mustard
seed 1000 and completed the physical lift/hold successfully.

## Interpretation

The original Piper deficit was largely, but not entirely, caused by poor
mounting geometry. A reasonable mount raises success from 12.5% to 37.5%,
close to the 45.0% FR3 baseline without changing AnyGrasp.

The remaining gap is morphology and orientation dependent. The mounted Piper
still has 60.9% NO_IK, compared with 2.9% for FR3. Hidden tuna candidates move
from mostly unreachable to table/scene collision, while mug candidates become
less reachable at the globally optimal mount. Bowl reaches executable poses
but fails physical contact. These patterns point to Piper's 6-DoF workspace,
wrist/gripper clearance, and grasp orientation compatibility rather than a
single remaining base-pose offset.

