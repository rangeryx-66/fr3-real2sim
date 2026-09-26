# AgileX Piper backend validation

## Frozen implementation

This validation changes only the robot execution layer. AnyGrasp inference,
weights, top-K output, score, rank, and pose are unchanged. The FR3 backend is
still available and remains the default. Scan, PayloadID, COM/inertia, CoACD,
and USD packaging were not changed.

The Piper backend uses the official URDF, collision meshes, joint limits,
gripper mimic definition, and MoveIt configuration. Exact upstream revisions
are recorded in `third_party/agilex_piper/COMMITS.txt`. The frozen Piper grasp
configuration is:

- six official arm joints `joint1..joint6`;
- total jaw opening `gripper` in `[0, 100]` mm;
- contact links `gripper_link1` and `gripper_link2`;
- TCP at the official distal finger plane, `gripper_base z=138 mm`;
- 20 mm grasp insertion, derived from the official 76.5 mm axial finger span;
- Piper pad bounds derived from the official collision geometry;
- the common 1 mm scene margin and unchanged physical success criteria.

The generated MoveIt TCP and the measured Isaac TCP agreed to about 0.2 micrometres
and `4.7e-7` radians. A simple-box smoke benchmark completed 10/10 successful
picks with 9.98--10.37 cm true lift, bilateral contact, and a two-second hold.

## Paired Arena protocol

The formal comparison used eight objects and seeds 1000--1039, five episodes
per object. Each FR3 episode copied the Piper episode's saved AnyGrasp JSON.
SHA-256 verification passed for all 40 pairs. Both robots used the same scene,
target pose, clutter, camera data, candidate order, success criteria, and
failure taxonomy.

One interrupted Piper result and five FR3 results affected by duplicate ROS
action servers were archived as invalid infrastructure runs and rerun. They are
not included below. The replacement runs used isolated ROS domains and ports.

| Robot | Pick success | Non-target contact | Non-target disturbance | Executable candidates | Episodes with a selected pose | Mean planning time |
|---|---:|---:|---:|---:|---:|---:|
| FR3 | 18/40 (45.0%) | 4/40 | 4/40 | 206/691 (29.81%) | 37/40 | 26.88 s |
| Piper | 5/40 (12.5%) | 0/40 | 0/40 | 11/691 (1.59%) | 6/40 | 23.06 s |

| Object | FR3 | Piper |
|---|---:|---:|
| mustard | 4/5 | 2/5 |
| raisin | 4/5 | 1/5 |
| hidden tuna | 0/5 | 0/5 |
| bowl | 0/5 | 0/5 |
| banana | 0/5 | 0/5 |
| sugar | 5/5 | 1/5 |
| soup | 5/5 | 0/5 |
| mug | 0/5 | 1/5 |

The Piper episode outcomes were 5 `SUCCESS`, 21 `SCENE_COLLISION`, 7 `NO_IK`,
5 `INSUFFICIENT_PAD_OVERLAP`, 1 `TABLE_COLLISION`, and 1 `BAD_CONTACT`.
Thirty-four episodes therefore exhausted the complete candidate set without a
selected executable pose. Candidate-level rejection was more diagnostic:

| Candidate status | Count | Share of all 691 candidates |
|---|---:|---:|
| `NO_IK` | 563 | 81.48% |
| `SCENE_COLLISION` | 49 | 7.09% |
| `TABLE_COLLISION` | 39 | 5.64% |
| `APPROACH_FAIL` | 18 | 2.60% |
| `INSUFFICIENT_PAD_OVERLAP` | 11 | 1.59% |
| `VALID` | 11 | 1.59% |

Of the six Piper episodes that reached physical execution, five succeeded.
The remaining one was a bowl `BAD_CONTACT`. This separates the present failure
mode from controller instability: the backend executes a feasible Piper pose
reliably, but the shared candidate set is rarely feasible for Piper from the
same mounting point.

## Conclusion

The clean, reusable Piper backend is operational, collision-aware, and preserves
the AnyGrasp output exactly. Its complex-scene success rate is not comparable to
FR3 yet: it falls from 45.0% to 12.5%. The 81.48% candidate-level `NO_IK` rate
shows that Piper reachability and six-DOF kinematics are the dominant bottleneck.
Table/scene clearance is secondary. Piper finger geometry rejects relatively few
candidates, and planning/execution is not the primary limitation.

The fair conclusion for this frozen comparison is therefore: AnyGrasp itself
does not need to be modified to operate Piper, but the current same-base scene
layout cannot maintain FR3 success. The next robot-level experiment should test
a declared Piper mounting transform or reachable workspace layout, then rerun
the same frozen JSON benchmark. That is an environment/robot placement change,
not an AnyGrasp score or pose change.

## Reproduction

Prepare and start the backend as documented in `docs/PIPER_BACKEND.md`:

```bash
python3 scripts/prepare_piper_description.py
bash run.sh piper-sim
bash run.sh piper-bridge
bash run.sh piper-moveit
bash run.sh piper-trial 10
```

For server-side downloads, first run `source ./proxyon.sh`. Use distinct
`ROS_DOMAIN_ID` and `PLANT_PORT` values for parallel jobs. The paired report is
generated with `scripts/summarize_robot_ab.py`; it rejects the paired-input claim
unless all saved grasp hashes match.
