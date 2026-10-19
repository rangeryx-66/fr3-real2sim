# R1-7a C-installation robustness and tabletop execution validation

## Scope and installation

The ten FR3 benchmark `trial_XX_grasps.json` files were reused byte for
byte. They contain the same 11 distinct frozen AnyGrasp poses repeated in
ten trials. This work changed only R1 installation, R1 scene/control
handling, and diagnostic scripts. No grasp pose, AnyGrasp, scanning,
payload, or articulated-object code changed.

The physical R1 base origin was set to world `(0.329, -0.175, 0.237 m)`
with yaw `+56.295°`. `R1A7_BASE_POSE` generates that fixed world-to-base
URDF joint for both Isaac and MoveIt. The table top is world Z = 0 and
the target box is centered at `(0.5, 0, 0.025 m)` with size
`0.045 × 0.045 × 0.050 m`. A **placeholder** fixed pedestal/support box
of `0.10 × 0.10 × 0.20 m` is centered at
`(0.329, -0.175, 0.10 m)`: it extends from the tabletop to Z = 0.20 m,
leaving 0.037 m to the model's base origin for an unmeasured adapter.
The dimensions must be replaced with measured hardware geometry before
physical deployment.

Isaac and MoveIt used the same generated C URDF. The live TCP cross-check
at home was below 1 µm position error and 6 µrad orientation error.
The importer cache is keyed by the generated URDF hash so that changing
the installation cannot silently reuse the earlier Isaac USD.

## Installation tolerance, before table and object collision

`scripts/r1a7_mount_tolerance_sweep.py` ran 31 conditions: nominal;
independent X/Y/Z errors of ±1 and ±2 cm; yaw ±3 and ±5°; local TCP
translation ±5 mm on each axis; and eight simultaneous extreme corners.
All conditions used exactly the frozen grasp list, MoveIt exact 6D IK,
2 cm sampled pregrasp/lift paths, ±2 cm postgrasp interactions, robot
self-collision, and J5/J6/J7 path margins. Each MoveIt case used 12
random seeds plus home and up to six IK branches. A complete path means
pregrasp→grasp, 10 cm lift, and all six interactions pass sampled IK;
it is not a proof of interpolated collision clearance or executability.

| Condition | Exact IK | Complete sampled path | Path margin >0.05 rad | Path margin >0.08 rad |
| --- | ---: | ---: | ---: | ---: |
| Nominal C | 10/11 | 10/11 | 8/11 | 6/11 |
| Single-axis errors, range over 22 conditions | 10/11 | 9–10/11 | 6–9/11 | 6–9/11 |
| Eight simultaneous extreme corners | 10/11 | 8–10/11 | 6–9/11 | 6–9/11 |

These results show a sampled **kinematic** neighbourhood around C:
±1–2 cm and ±3–5° do not erase exact IK in this target set. The
joint-margin count can shift by three of eleven candidates, and combined
corners can lose two complete sampled paths. TCP ±5 mm likewise kept
10/11 exact IK and 10/11 complete paths in the self-collision-only sweep.

## Full scene collision and MoveIt planning

The live R1 scene includes robot self collision, a 0.7 × 0.7 × 0.05 m
table with top at Z = 0, the parameterized pedestal, and the 45 × 45 ×
50 mm target box. Only the two Dex1 terminal finger links are allowed to
touch the target. An attached box can replace the world box and attach to
`r1a7_tcp` with those two touch links; the planning scene was verified to
switch world → attached → world. The target could not be physically
attached in this benchmark because every grasp was rejected before close.

The full-scene state audit found **10/11 kinematic IK**, **0/11
scene-collision-free grasp states**, **10/11 table collisions**, and **zero
pedestal collisions**. The contacts were the Dex1 second/third finger
links against the table. The maximum reported penetration for individual
solved candidates ranged from 7.5 to 21.1 mm. Removing the table box
made the representative rank-0 grasp state valid; removing the pedestal
or target afterward changed nothing. This isolates the blocker to hand
geometry versus tabletop at the frozen exact grasp poses.

The actual MoveIt OMPL request for rank 0 planned a collision-free
83-point home→pregrasp path with 0.057 rad minimum J5/J6/J7 margin.
The actual Cartesian approach service returned only **0.8148 fraction**
with collision avoidance enabled. A full pregrasp→grasp→lift plan was
therefore not submitted for execution. Because the grasp TCP pose and
Dex1 geometry relative to that pose are fixed, changing only the base
installation cannot remove the finger/table overlap.

All six local TCP translation ablations (±5 mm on X, Y, and Z) still
had 10/11 kinematic IK and **0/11 scene-collision-free grasp states**
with the table and pedestal enabled. A calibration error of this tested
size does not remove the current blocker. The adapter-to-Dex1 transform
remains mesh-derived and should still be measured on physical hardware.

## Isaac execution and 10-trial benchmark

One collision-free pregrasp plan was executed in Isaac through MoveIt's
`ExecuteTrajectory` action. Maximum arm tracking error in the plant's
retained execution samples was 0.0204 rad. During a 2.2 s hold at pregrasp, sampled TCP coordinate
ranges were 0.36, 0.41, and 0.22 µm, with maximum reported arm target
error 0.0129 rad. Dex1 close tracked both finger joints to about
0.0245 m at pregrasp; both contact forces remained zero because the
gripper was intentionally away from the object. No obvious drive
oscillation was observed in this safe test, and no gains were changed.

The full grasp approach was **not** executed into a known table
collision. Thus contact quality, loaded lift, slip, and drop were not
measured. The attached-object planning-scene API passed a state update
smoke test, but no attached-object trajectory could safely be attempted.

| Measure | FR3 + Franka Hand | R1-7a + Dex1 at C |
| --- | ---: | ---: |
| Frozen candidates evaluated by benchmark | First valid candidate per trial; all 110 were not individually tested | 110/110 |
| Candidate exact 6D kinematic IK | ≥10/10 selected first candidates; all 110 not evaluated | 100/110 |
| Candidate full-scene collision-free grasp | 10/10 selected first candidates; all 110 not evaluated | **0/110** |
| Candidate sampled path with >0.05 / >0.08 rad, self collision only | Not available | 80/110 / 60/110 |
| Candidate safe with full scene at either margin gate | First selected candidate in each of 10 trials | **0/110** |
| Complete grasp plan admitted per trial | 10/10 | **0/10**, blocked by scene collision before planning |
| Complete grasp execution attempted | 10/10 | **0/10**; conditional execution success is unmeasured |
| Final lift ≥8 cm, hold ≥2 s, no drop | **10/10** | **0/10** |

All R1 ten episodes ended `NO_EXECUTABLE_CANDIDATE`. Their 110
candidate outcomes were `COLLISION` × 100 and `NO_IK` × 10; there were
no `LOW_JOINT_MARGIN`, `NO_PLAN`, `BAD_CONTACT`, `CONTACT_LOSS`,
`DROP`, or `SUCCESS` candidate outcomes in the completed benchmark.
The latter categories were not reached; zero counts are not evidence
that contact or control would succeed after resolving scene collision.
The ten frozen files, scene, target object, and lift/hold criterion match
the earlier FR3 benchmark. The R1-specific pedestal is the additional
support collision geometry.

## Answers

1. **Is C suitable for actual grasping?** No under the current exact
   frozen poses and Dex1 geometry. It is a useful kinematic placement,
   but all ten IK-feasible unique grasps intersect the table.
2. **Do ±1–2 cm / ±3–5° mounting errors matter?** They do not remove
   exact IK in the 31 sampled self-collision conditions, but they change
   path and joint-margin counts. The measured full-scene success rate
   remains zero because the frozen TCP/hand pose still intersects the
   table. This is a sampled tolerance study, not a worst-case guarantee.
3. **What is the present bottleneck?** Full-scene collision and approach
   planning, specifically Dex1 finger/table overlap. Safe pregrasp
   execution and hold were stable; gripper contact and loaded Isaac
   control remain untested because no collision-free grasp is available.

No articulated-object or hinge work was started.

## Reproduce and evidence

Export the same variables in the Isaac, MoveIt, bridge, and trial shells:

```bash
export R1A7_BASE_POSE=0.329,-0.175,0.237,56.295
export R1A7_PEDESTAL_SIZE=0.10,0.10,0.20
export R1A7_MIN_JOINT_MARGIN_RAD=0.05
export R1A7_ROS_DOMAIN_ID=202
./run.sh r1a7-sim --gpu 1 --port 18775
./run.sh r1a7-moveit
./run.sh r1a7-bridge
./run.sh r1a7-trial --trials 10 --grasps-dir /path/to/frozen/FR3/run
```

The diagnostic MoveIt sweep used a separate domain with the original
`R1A7_BASE_POSE=0,0,0,90` model and transformed unchanged world
targets by each hypothetical base pose. This is mathematically
equivalent to changing the world mount for IK. The actual C Isaac and
MoveIt stack used the C mount directly. See
`results/r1a7_installation_validation/` for the sweep summary,
full-scene audit, planning probe, attached-scene smoke test, drive hold,
and all ten R1 trial records.

- `tolerance_summary.json`: all 31 perturbations and both margin gates.
- `scene_collision_audit.json` and `scene_ik.json`: per-grasp collision
  contacts and full-scene IK totals.
- `scene_tcp_summary.json`: six ±5 mm TCP checks with table/support collision.
- `planning_probe.json`, `drive_hold.json`, and
  `attached_scene_smoke.json`: planner, safe Isaac motion, and attachment
  interface evidence.
- `r1_benchmark10/` and `fr3_summary.json`: trial-level R1 data and
  the earlier FR3 baseline summary.
