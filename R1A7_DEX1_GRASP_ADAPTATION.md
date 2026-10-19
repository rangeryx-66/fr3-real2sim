# Dex1 local grasp adaptation: R1-7a tabletop benchmark

## Scope and implementation

The AnyGrasp model, inference, top-K output, scores, and raw camera-frame
poses are unchanged. `src/r1a7_grasp_adaptation.py` generates bounded Dex1
TCP alternatives near each raw pose. It searches vertical offsets of
5–25 mm, insertion-depth shifts of ±10–20 mm, lateral shifts of ±5 mm,
and local roll/pitch/yaw changes up to 20°. The raw transform and candidate
metadata are retained in each trial JSON. The best valid variant is stored
for every raw candidate before the candidates are ranked for execution.

The official Dex1 terminal pad mesh spans TCP-local X = -28.4–0 mm and
Z = -2–47.5 mm. The contact proxy requires the target centre to lie inside
that pad region, between the jaws, and within the Dex1 opening. This proxy
is a necessary geometric check; loaded Isaac lifting is the contact proof.
Variant ranking imposes hard gates for geometry, exact 6D MoveIt IK,
J5/J6/J7 margin >0.05 rad, robot/table/pedestal/target collision, and
Cartesian pregrasp→grasp plus micro-lift/lift paths. Among survivors it
favours joint margin and pad alignment while penalizing displacement and
rotation from the raw AnyGrasp pose. MoveIt OMPL then plans home→pregrasp.
The R1 backend executes approach, Dex1 close, 2 cm micro-lift, 10 cm lift,
and 2.2 s hold in Isaac. The box is attached in the MoveIt scene after close.

Only `src/r1a7_backend.py` and the new R1-specific adaptation module were
edited. FR3Backend, AnyGrasp, scan, PayloadID, COM/inertia, and hinge code
were untouched. The C installation is `(0.329, -0.175, 0.237 m;
yaw=56.295°)`, with a parameterized 0.10 × 0.10 × 0.20 m pedestal box.
The same official Unitree R1-7a + Dex1 URDF is loaded by MoveIt and Isaac.

## Reproduction

Prepare the R1 description with `python scripts/prepare_r1a7_description.py`
and launch `src/r1a7_sim_server.py`, `src/r1a7_moveit.launch.py`, and
`src/r1a7_ros_bridge.py` as in `R1A7_INSTALLATION_VALIDATION.md`. Set
`R1A7_BASE_POSE=0.329,-0.175,0.237,56.295` and
`R1A7_PEDESTAL_SIZE=0.10,0.10,0.20`. With the existing ten frozen FR3
benchmark output files, run:

```bash
python src/r1a7_backend.py --mode raw --trials 10 --grasps-dir /data1/home/rangeryx/fr3_moveit_grasp/results/run_1788943864166403373
python src/r1a7_backend.py --mode adapted --trials 10 --grasps-dir /data1/home/rangeryx/fr3_moveit_grasp/results/run_1788943864166403373
```

The same 11 grasp poses occur in each of the ten files, with the same
tabletop scene, target box, reset seed, and lift/hold success criterion as
the FR3 benchmark. For identical grasp arrays and reset box position,
the variant search is cached in memory; each episode still replans and
physically executes independently. This is repeated execution in one
frozen scene, not evidence of generalization to new objects or grasps.

## Results

| Measure | Raw AnyGrasp + Dex1 | Dex1-adapted AnyGrasp |
| --- | ---: | ---: |
| Raw candidates processed | 110 | 110 |
| Candidates passing geometry, exact IK, margin, collision and complete Cartesian path | 0 | 20 (2 distinct / 11) |
| Collision-free complete-path candidates | 0 | 20 |
| Trials with MoveIt pregrasp plan | 0/10 | 10/10 |
| Trials executing the approach in Isaac | 0/10 | 10/10 |
| Final ≥8 cm lift, ≥2 s hold, no visible slip/drop | 0/10 | **10/10** |

Raw mode ended `NO_EXECUTABLE_CANDIDATE` in all ten trials. With the new
Dex1 pad-geometry prefilter, its 110 candidate outcomes were 40 table
`COLLISION` and 70 `BAD_GRASP_GEOMETRY`. The earlier full-scene audit
without that prefilter found 100/110 table collisions, 10/110 NO_IK, and
0/110 collision-free raw grasp states. Thus the raw mode's geometry labels
must not be read as 70 newly discovered collision-free poses.

Adapted mode selected raw rank 9 in all ten trials. Its chosen variant
shifted the TCP roughly 25 mm from the raw pose with zero rotation change.
The selected grasp and checked Cartesian paths had 0.358 rad minimum
J5/J6/J7 margin; the ten independent OMPL pregrasp plans had at least
0.104 rad margin. All ten lifts were 99.88–100.36 mm. The maximum recorded
arm tracking error was 0.011 rad. Across the 2.2 s hold, the largest
box-relative-to-TCP coordinate range was 0.54 µm. No `NO_PLAN`,
`BAD_CONTACT`, `CONTACT_LOSS`, or `DROP` final outcomes occurred in the
ten adapted episodes. Candidate-level rejection was dominated by
pad geometry and table collision; some IK attempts also hit the margin
gate or failed to solve.

The imported Dex1 terminal-link force sensors reported zero throughout
these loaded grasps. Both finger prismatic joints stalled before their
unloaded close target, and the box followed the micro-lift and 10 cm lift
with a stable hold. The pass result therefore relies on measured object
motion and hold stability, not on the zero force readout. Finger force
instrumentation should be repaired before using force thresholds as a
hardware acceptance criterion. Physical R1/Dex1 hardware was not tested.

Raw trial data: `results/r1a7_grasp_adaptation/raw_benchmark10/`.
Adapted trial data: `results/r1a7_grasp_adaptation/adapted_benchmark10/`.
