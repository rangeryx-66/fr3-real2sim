# R1-7a + Dex1 official ecosystem audit

## Result

The R1-A7 and Dex1 source URDFs and meshes remain pinned to Unitree `unitree_ros`
commit `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee`. The combined R1 + Dex1
mount and grasp TCP are **not** published in those source descriptions. Their
existing values have been moved to `config/end_effector_calibration.yaml` and
explicitly marked provisional. This preserves the earlier simulation geometry
while making its hardware uncertainty visible and configurable.

| Definition | Evidence and status | Action |
| --- | --- | --- |
| R1-A7 J1–J7 axes, limits, inertia, meshes | Direct Unitree R1-A7 URDF | Preserved verbatim |
| Dex1 finger origin, opposite X axes, limits `[-0.020, 0.0245] m`, terminal collision STL | Direct Unitree Dex1 URDF | Preserved verbatim; sim width map now derived from URDF origins and limits |
| Dex1 zero-width convention | URDF gives nominal terminal-link separation `0.050067 - 2q` m; this is geometric, not measured pad contact gap | Sim bridge uses derived formula; true pad gap requires measurement |
| Link7 → Dex1 mounting | No official R1-A7 + Dex1 assembly found; G1 + Dex1 assembly is a different wrist | Existing mesh-fit transform moved to calibration YAML; hardware unconfirmed |
| Dex1 base → grasp TCP | No official Dex1 grasp TCP frame in the source URDF | Existing pad-center frame moved to calibration YAML; hardware unconfirmed |
| Finger pad contact surface and friction | Terminal STL is official; pad footprint, friction and contact thresholds were simulation inferences | Contact proxy bounds/thresholds moved to calibration YAML; measure hardware |
| Joint zero | URDF prismatic `q` is metres; `dex1_1_service` sets motor zero after manually closing the gripper | Do not equate URDF `q` with DDS motor `q`; mapping is unset in calibration YAML |
| Real gripper command/state | `rt/dex1/{left,right}/{cmd,state}` with `MotorCmds_`/`MotorStates_`, single motor per gripper; simulation uses ROS `GripperCommand` and two prismatic joints | Documented official interface and kept simulation bridge separate; no unverified hardware bridge added |
| Calibration | Unitree service `-c`: manually close each gripper, calibrate, then verify motor state near zero | Required before hardware control |

Official sources: [Unitree Dex1 URDF](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/dexterous_hand_description/dex1_1), [Unitree R1 description](https://github.com/unitreerobotics/unitree_ros), [G1 + Dex1 variant](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/README.md), [XR R1-A7 and Dex1 selection](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/teleop_hand_and_arm.py), [XR Dex1 DDS and motor mapping](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_hand_unitree.py), [Dex1 service calibration](https://github.com/unitreerobotics/dex1_1_service/blob/main/README.md).

## Validation in this audit

- `prepare_r1a7_description.py` generated the combined URDF/SRDF using the calibration file.
- `audit_r1a7_model.py` passed: 13 official joint definitions preserved, 15 links checked, zero missing meshes.
- `smoke_r1a7_official_calibration.py` passed static checks on all 40 sealed scene inputs, eight object classes, TCP/mount config, and simulated opening limits.
- **Isaac/MoveIt startup, live FK/TCP agreement, physical open/close, and benchmark execution were not run.** `labserver_inschool` port 10061 timed out twice during this audit. The static smoke is not an execution regression result.

## Hardware calibration required

1. Identify the installed left/right Dex1 and motor ID; record adapter dimensions, then measure `Link7 → dex1_base_link` with a fixture or CAD and refine with a wrist pivot calibration.
2. Measure a tool/grasp TCP from the actual finger pads and verify it by touching a known target from several arm configurations. Compare MoveIt FK, external measurement and robot reported joint states.
3. With the service's documented closed-jaw zero calibration, measure motor radians versus pad gap at several openings and fit the map in `end_effector_calibration.yaml`; confirm direction and travel limit before enabling a real control bridge.
4. Measure actual contact patch, pad friction, contact sensing/stall behavior, payload and drive response. Replace simulation proxy thresholds with measured values.

The earlier Isaac grasp success numbers remain evidence for this simulated assembly only. The
unknown R1 adapter, TCP, motor-to-gap map and pad contact properties can change IK, collision,
contact and drop outcomes on hardware. No grasp success tuning or articulated-object work was
performed in this audit.
