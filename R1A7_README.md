# Unitree R1-7a + Dex1 grasp backend

This branch adds an independent `R1A7Backend`. The FR3 backend, AnyGrasp
model/inference/top-K scores and poses, scanning, PayloadID, and inertial
estimation are unchanged.

## Model and frames

The source arm and hand URDF/meshes are the unmodified files from
[`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros)
at commit `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee`:
`robots/r1_7a_description` and
`robots/dexterous_hand_description/dex1_1`. Their BSD-3 license is copied
under `third_party/unitree_ros/LICENSE`.

Run `python3 scripts/prepare_r1a7_description.py` on each machine to make
`config/r1a7_dex1.urdf` and `.srdf` with local absolute mesh paths. The script
preserves official joint axes, limits, inertias, and collision meshes. It adds
the following fixed joints because Unitree publishes the arm and Dex1 as
separate descriptions:

| Joint | Parent → child | xyz (m) | rpy (rad) |
| --- | --- | --- | --- |
| `r1a7_world_mount` | world → `base_link` | Set by `R1A7_BASE_POSE` | yaw from `R1A7_BASE_POSE` |
| `r1a7_dex1_mount` | `Link7` → `dex1_base_link` | 0, −0.047736, 0 | 0, 0, π |
| `r1a7_tcp_joint` | `dex1_base_link` → `r1a7_tcp` | 0, 0.097343, 0.0142 | 0, −π/2, −π/2 |

The default world mount reproduces the earlier benchmark placement. The arm-to-Dex1 adapter
transform and finger-center TCP are derived from the published meshes/URDF,
not an official combined assembly. Verify the adapter against real hardware
before sending physical commands. Run `python3 scripts/audit_r1a7_model.py`
to compare the combined model with the upstream source.

The generated world mount defaults to the earlier `(0, 0, 0; +90°)`
diagnostic placement. Set `R1A7_BASE_POSE=x,y,z,yaw_deg` consistently for
Isaac, MoveIt, bridge, and trial to evaluate another installation; the
model generator only changes this fixed world mount, not official Unitree
arm geometry or joint limits. `R1A7_PEDESTAL_SIZE` sets a world-frame
support box size (X,Y,Z metres) in Isaac and MoveIt; the default is
`0.10,0.10,0.20`. `R1A7_MIN_JOINT_MARGIN_RAD` sets the executor's
J5/J6/J7 threshold (default `0.05`).

The MoveIt group is `r1a7_arm`, a chain from `r1a7_world` to `r1a7_tcp`.
It contains J1–J7 and uses KDL 7DoF redundancy with multiple seeds. The
Dex1 width command maps 0–90 mm opening onto the two official finger joints;
the bridge publishes measured joint states. The Isaac importer checks nine
joint axes/limits and sets explicit drives because the upstream URDF has no
drive gains. Isaac internal self collision is disabled for overlapping
adjacent Dex1 meshes; MoveIt checks arm/hand self collision and the table,
scene, and attached object with the generated SRDF.

## Start on the Isaac/ROS2 host

Set `ISAAC_PYTHON` to the Isaac Sim Python executable and `CONDA_SH` to the
Conda activation script. Set `ROS_ENV` to the ROS2/MoveIt environment if it is
outside this checkout. Keep the same `R1A7_ROS_DOMAIN_ID` and
`R1A7_PLANT_PORT` in every terminal. Then run, in separate terminals:

```bash
./run.sh r1a7-prepare
./run.sh r1a7-sim
./run.sh r1a7-moveit
./run.sh r1a7-bridge
./run.sh r1a7-trial --trials 10
```

For the C-installation validation, export
`R1A7_BASE_POSE=0.329,-0.175,0.237,56.295` and
`R1A7_PEDESTAL_SIZE=0.10,0.10,0.20` in every terminal before these
commands. The measured C-placement benchmark is documented in
`R1A7_INSTALLATION_VALIDATION.md`: exact frozen grasps currently collide
with the table, so do not attempt grasp execution on hardware at C.

To replay the exact AnyGrasp JSONs of a previous FR3 run, pass
`--grasps-dir /absolute/path/to/fr3/results/run_...` to `r1a7-trial`.
The R1 backend still captures the same scene and checks the Isaac/MoveIt TCP
alignment before candidate evaluation.

## Comparable IK diagnostic

`scripts/compare_tabletop_ik.py` runs the frozen
`config/ik_tabletop_81.json` world-frame targets against an isolated MoveIt
service for each robot. For each target it records exact 6D, discrete ±10°
and ±20° orientation perturbations, position-only solutions, five seeds,
collision status, solver codes, and per-joint limit margins. The relaxed
orientations are sampled directions inside each tolerance, so these success
counts are lower bounds on continuous-tolerance reachability. Run each robot
in a separate ROS domain. For the R1-7a service:

```bash
python3 scripts/compare_tabletop_ik.py \
  --robot r1a7 --urdf config/r1a7_dex1.urdf \
  --base r1a7_world --tip r1a7_tcp --group r1a7_arm \
  --joints J1,J2,J3,J4,J5,J6,J7 --home 0,1.3,1,-1.3,0,0,0 \
  --fingers dex1_Joint1_1,dex1_Joint2_1 \
  --finger-positions=-.02,-.02 --output results/r1a7_ik_81.json
```

`R1A7_MODEL_VARIANT=j7_fixed ./run.sh r1a7-moveit` locks only J7 at zero
for a controlled 6DoF ablation. Run the diagnostic with
`config/r1a7_dex1_j7_fixed.urdf` and J1–J6 to compare it with the full arm.
