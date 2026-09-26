# AgileX Piper grasp backend

## Scope

This backend replaces only the robot execution layer. AnyGrasp inference, its
weights, score, rank and 6-DoF pose are unchanged. Scan, reconstruction,
PayloadID and USD code remain FR3-only until a separate dynamics calibration is
performed for Piper.

`src/backend.py` owns the common MoveIt sequence. `src/robot_profile.py` supplies
robot names, actions, joint limits, opening convention and the grasp-frame
offset. `src/fr3_backend.py` and `src/piper_backend.py` are explicit entry
points. The default profile remains FR3.

## Upstream model

The official Piper URDF, visual/collision meshes and gripper mimic joints are
vendored under `third_party/agilex_piper/description`. The reference MoveIt 2
configuration is under `third_party/agilex_piper/agx_arm_moveit`. Exact upstream
commits are in `third_party/agilex_piper/COMMITS.txt`. Run
`scripts/prepare_piper_description.py` after cloning; it produces host-local
URDF/SRDF files with absolute mesh paths.

The arm is the official six-joint chain `joint1..joint6`. The gripper command
joint is `gripper` (0..100 mm total opening); the imported physical fingers are
`gripper_joint1=width/2` and `gripper_joint2=-width/2`. Contact links are
`gripper_link1` and `gripper_link2`. No Franka joint gain, finger name or TCP
offset is reused.

## Frames

Transforms are active column-vector transforms: `T_A_B` maps B coordinates to
A, in metres.

1. AnyGrasp reports `T_camera_grasp`, with grasp +x as approach, +y as jaw
   separation and +z as grasp height.
2. `T_base_camera` is saved with the point cloud.
3. The common robot TCP uses +z approach and +y jaw separation. The fixed
   orientation is

   ```text
   R_grasp_tcp = [[ 0, 0, 1],
                  [ 0, 1, 0],
                  [-1, 0, 0]]
   ```

4. Piper `tcp_link` is fixed at `gripper_base z=138 mm`, the official distal
   finger plane. The official finger collision mesh spans 76.5 mm axially. A
   20 mm insertion places the target within that span, so
   `x_grasp_tcp = AnyGrasp.depth + 20 mm`. This is represented by the Piper
   profile's `grasp_tip_offset_m=-0.020`; it is independent of FR3's +9.5 mm
   correction.

Therefore:

```text
T_base_piper_tcp = T_base_camera * T_camera_grasp * T_grasp_piper_tcp
```

MoveIt FK and Isaac's measured `tcp_link` must agree within 3 mm / 0.02 rad;
the validated simple run was about 0.2 micrometres / 4.7e-7 rad.

## Candidate and execution policy

Candidates retain AnyGrasp order. Each must pass width, IK, official joint
limits, self/table/scene collision, pregrasp IK, Cartesian approach, global
pregrasp planning and Cartesian vertical-lift reachability. The Arena adapter
also checks the target mesh, 1 mm non-target boxes and the Piper pad rectangle.
The first valid candidate is executed.

Execution is:

```text
current -> planned pregrasp -> Cartesian grasp -> physical close
        -> 5 mm support-aware micro-lift -> Cartesian lift -> 2 s hold
```

The target/table ACM pair is enabled only during the 5 mm support departure.
Target contact is not allowed during pregrasp or approach. After close, only the
two actual finger links are touch links for attached-object collision checks.
Success requires bilateral physical contact, at least 8 cm true target lift and
a stable two-second hold. Trajectory success alone is never counted.

## Run

Use four terminals after loading the documented environments:

```bash
python3 scripts/prepare_piper_description.py
bash run.sh piper-sim
bash run.sh piper-bridge
bash run.sh piper-moveit
bash run.sh piper-trial 10
```

If downloads are needed on the lab server, first run `source ./proxyon.sh`.
For isolated parallel tests, set `PLANT_PORT`, pass the same `--port` to the
simulator, and use a distinct `ROS_DOMAIN_ID`.

## Paired A/B requirements

Use the same Arena target/seed and copy the Piper run's `inputs/seed_*_grasps.json`
into the FR3 output's `inputs/` directory. The summarizer verifies each
`grasp_sha256`; a comparison is not paired if a hash differs. Run
`scripts/summarize_robot_ab.py` only after both directories are complete.

Failure categories retain the existing meanings: `NO_IK`, `JOINT_LIMIT`,
`SELF_COLLISION`, `TABLE_COLLISION`, `SCENE_COLLISION`, `NO_PLAN`,
`NO_EXECUTABLE_CANDIDATE`, `BAD_CONTACT`, `CONTACT_LOSS`, `SLIP`, `DROP`,
`LIFT_FAIL`, and `SUCCESS`.
