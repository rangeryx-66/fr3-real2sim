# R1-7a base placement: path and joint-margin study

This study continues the frozen AnyGrasp IK diagnosis. The 11 unique
grasp poses are the byte-identical inputs used in ten benchmark repeats
(110 candidate occurrences). No inference, grasp pose, motion planning,
simulation, contact, or robot execution was run.

## Evaluation contract

- Search variables: base world X/Y/Z and yaw. The official Unitree arm and
  Dex1 joint limits, hand model, grasp transforms, and TCP stay unchanged.
- Exact 6D grasp IK: MoveIt KDL with multiple seeds; kinematic success and
  self-collision-free success are recorded separately.
- Path reachability: grasp to pregrasp is sampled backward 8 cm along TCP
  approach, then grasp to lift 10 cm upward in world Z. Waypoints are 2 cm
  apart with frozen orientation. Traversing backward from grasp tests the
  same pregrasp→grasp segment in reverse.
- Interaction reserve: from grasp, world ±X/±Y/±Z, each to 1 and 2 cm,
  with unchanged orientation. All six directions must be feasible.
- Continuity: each waypoint uses the previous joint solution as its IK
  seed; a step is rejected if any joint jumps by more than 0.45 rad over
  one 2 cm Cartesian segment. Multiple grasp branches are tested to
  prefer a continuous solution with J5/J6/J7 clearance.
- Collision: MoveIt `check_state_validity` checks robot self collision
  at grasp and every sampled waypoint. `scripts/verify_r1a7_placement_moveit.py`
  also accepts `--scene-json` to add table and mount collision boxes in
  world coordinates after transforming them to the selected base frame.
  `config/r1a7_collision_scene.example.json` is the schema. Table and
  mount boxes are disabled for the ranked results because the actual
  mount geometry is unknown. The scene API was smoke-tested with the
  benchmark table box enabled on an isolated MoveIt instance.

The previous single-pose IK recommendation `(0.326, −0.201, 0.147 m;
83.753°)` is the baseline: MoveIt grasp IK 11/11, pregrasp/lift 9/11,
all six interactions plus the path 8/11. Although all grasp targets are
reachable, some J5/J6/J7 path margins are zero.

## Three installation candidates

| Choice | Base world `(x, y, z, yaw)` | Exact grasp IK, self collision free | Pregrasp + lift | Complete path + all ±XYZ interactions | J5/6/7 grasp margin median / worst | J5/6/7 complete path margin median / worst |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| A, maximum point reach | `(0.378, −0.146, 0.187 m, 38.056°)` | 11/11 (110/110) | 9/11 | 8/11 | 0.233 / 0.023 rad | 0.243 / 0 rad |
| B, maximum safe margin | `(0.338, −0.114, 0.141 m, 54.801°)` | 10/11 (100/110) | 8/11 | 7/11 | 0.289 / 0.106 rad | 0.247 / 0.111 rad |
| C, recommended balance | `(0.329, −0.175, 0.237 m, 56.295°)` | 10/11 (100/110) | 10/11 | 10/11 | 0.376 / 0.101 rad | 0.222 / 0 rad |

Margins are the minimum across J5/J6/J7 for each sampled state. The
"worst" path statistic is over complete paths only. A has 6/8 complete
paths with at least 0.05 rad clearance, B has 7/7, and C has 8/10.
Thus C has the best tested complete-path coverage, but two of its ten
complete paths still come within 0.05 rad of a J5/J6/J7 limit. At C,
one path touches the J6 limit and another has a 0.025 rad J5 margin.
A and C are not blanket installation approvals: a joint-limit reserve
threshold should be enforced during subsequent candidate selection.
B is the margin-priority choice because its worst complete-path clearance
is 0.111 rad, the largest minimum among the MoveIt-verified finalists
that reach at least 10/11 frozen grasps; it sacrifices three complete
paths compared with C.

| Choice | Worst J5 on complete paths | Worst J6 | Worst J7 |
| --- | ---: | ---: | ---: |
| A | 0.005 rad | 0 rad | 0.966 rad |
| B | 0.111 rad | 0.184 rad | 0.341 rad |
| C | 0.025 rad | 0 rad | 0.754 rad |

The comparison uses 11 unique frozen poses, each repeated unchanged ten
times in the benchmark. Counts out of 110 therefore multiply each unique
pose by ten; they are not 110 independent grasps.

The benchmark table top is world Z = 0. The object center is approximately
world `(0.5, 0, 0.025 m)`. Base Z is therefore the proposed mounting
height above the tabletop. The planar base-to-object distance is
`sqrt((0.5 − base_x)^2 + base_y^2)`. Yaw is measured counterclockwise
from world +X when viewed from above. These coordinates are kinematic
recommendations; a measured pedestal/adapter, table clearance, and
attached-object collision model are still needed before motion planning.

For the recommended C placement, mount the arm base origin about **23.7 cm
above the tabletop**, **17.1 cm back in X** and **17.5 cm toward negative Y**
from the nominal object center (24.5 cm horizontal separation), at **+56.3°
yaw** in the benchmark world frame. A is 18.7 cm high, 19.0 cm horizontally
from the object, yaw +38.1°; B is 14.1 cm high, 19.8 cm horizontally,
yaw +54.8°. These are base-frame origins, not the top of a pedestal; the
mount adapter thickness must be accounted for when setting hardware height.

## Result files

The deterministic numerical screen contains 113 coarse and 49 local
placements; 34 and 18 respectively received numerical path evaluation.
The coarse samples were anchored at the previous diagnostic finalists and
perturbed in all four base variables (clipped to X 0.15–0.45 m, Y
−0.35–0.15 m, Z −0.03–0.32 m, yaw 20–145°); the local set perturbed
the former 10/11 complete-path candidate. This is a sampled search, not a
global optimum certificate.
The three published placements then received independent MoveIt IK and
self-collision evaluation. The numeric screen is useful for ranking, but
the MoveIt results in the table above are authoritative. A follow-up C
run with 24 random seeds and 12 grasp branches reproduced 10/11 complete
paths and 8/10 paths above 0.05 rad margin; it did not remove the J6
limit contact.

- `results/r1a7_placement_robustness/r1a7_robust_search.json`: coarse
  placement search, complete per-target numerical records.
- `results/r1a7_placement_robustness/r1a7_robust_refine.json`: local search.
- `results/r1a7_placement_robustness/r1a7_robust_moveit_refine2.json`:
  A, full MoveIt record.
- `results/r1a7_placement_robustness/r1a7_robust_moveit_candidate18.json`:
  B, full MoveIt record.
- `results/r1a7_placement_robustness/r1a7_robust_moveit_refine7.json`:
  C, full MoveIt record; `_deep.json` is its expanded-seed repeat.
- `results/r1a7_placement_robustness/r1a7_robust_moveit_A_full.json`:
  previous placement's full MoveIt baseline.
- `results/r1a7_placement_robustness/r1a7_scene_table_hook.json`:
  isolated planning-scene API smoke test, with the table box enabled and
  no motion command.

To repeat the search on the ROS/MoveIt machine after preparing the model:

```bash
python scripts/r1a7_placement_robustness.py --samples 100 --max-shortlist 35 \
  --output results/r1a7_placement_robustness/search.json
python scripts/r1a7_placement_robustness.py --samples 48 --max-shortlist 18 \
  --refine-center 0.318 -0.179 0.204 50.002 \
  --output results/r1a7_placement_robustness/refine.json
R1A7_ROS_DOMAIN_ID=197 ./run.sh r1a7-moveit
ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 python scripts/verify_r1a7_placement_moveit.py \
  --base 0.329 -0.175 0.237 56.295 --seeds 12 --branches 6 \
  --output results/r1a7_placement_robustness/C_moveit.json
```

The evaluated paths are sampled IK/state-validity chains, not MoveIt
planned trajectories. A 2 cm sample interval and 0.45 rad maximum joint
step do not prove all interpolated states collision free. Table and mount
boxes are deliberately excluded from A/B/C scores until their physical
geometry is measured; pass `--scene-json` with those boxes enabled to
check them once known. No Isaac Sim or robot execution was performed.
