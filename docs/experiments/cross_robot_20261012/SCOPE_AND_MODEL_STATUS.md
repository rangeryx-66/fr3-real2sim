# Cross-robot study scope and model status

Reference: `81ab4f5`. Label: **ORACLE_HANDLE / KNOWN_MODEL / CROSS_ROBOT_ABLATION**.

Four registered robots × four fixed assets: PiPER, NERO, xArm7, FR3;
45134, 45443, 45176, 45415. No failed object or robot is replaced. Existing physical baselines are references, not new episodes.

The same geometry/contact score, frozen object colliders and PiPER hand are reused. Pink task weights, posture cost, inherited 0.05 rad limit reserve, velocity budget and numerical solve settings remain unchanged. The adapter supplies joint count/order, official URDF geometry/limits, home configuration and fixed tool transform. Two explicit IK seeds replace the six-joint-specific wrist flip; an initial engineering query that accidentally repeated the home seed is preserved and excluded from the registered final comparison. No physical episode occurred during that correction. A second input-only correction resolves legacy cases through their saved collision-export paths; a new final 16-case batch uses the same corrected adapter across all cases. Engineering batch metadata/partial outputs remain separate.

Three offline modes distinguish common grasp/common PiPER station, common grasp/optimized station, and independently selected geometry-ranked grasp/station. Station and own-grasp searches each have the same 300 s ceiling across robots; candidate order, two-branch budget and deterministic seeds are shared. Full seven-joint solutions are compared against joint-3 locks at predefined -0.6/0/+0.6 rad and the full-seven solution's initial joint-3 value, using the same tool, collision meshes, base and grasp. The matched lock is a universal rule, not an asset-specific setting. Unreachable fixed values remain failures. The locked models are not called PiPER substitutes.

Arm collision checks use official URDF collision meshes as convex hulls; unchanged native object and frozen finger solids remain. These foreign-arm proxies are offline estimates, not claimed native-contact measurements. PiPER uses its exact frozen native collision export. FK/Jacobians are checked independently against the URDF-chain implementation, including reduced models. Isaac model-only imports inspect joint name-to-USD paths, native limits and drive values without an object scene, World or physics integration. Tensor index order and physical tracking are not claimed from that structural inspection.

## Physical admissibility required by the user

The user explicitly says not to launch an invalid physical trial if a robot lacks a credible simulator model. The retrieved seven-joint models do not yet provide a validated arm actuator plus same-gripper installation in this workspace:

- NERO official URDF has uniform nominal effort/velocity fields and zero imported servo gains. SDK torque command ranges depend on firmware; an encoding range is not a calibrated motor saturation/dynamics model. Exact PiPER and current AgileX gripper meshes are identical, but current official finger conventions differ from the frozen hand. No silent gain transfer is made.
- xArm7 has official mass/collision/kinematic data. Manufacturer ROS PID configurations are simulation examples, and importing the URDF gives no position-drive stiffness. A mechanically validated PiPER tool adapter is absent; its coaxial tool transform is conditional offline geometry.
- FR3 has official inertia/kinematics/limits and transmission dynamics metadata. The importer does not turn its motor inertia, gear ratio, K/D and friction metadata into a complete actuator model. The old local FR3 simulator uses arm gain overrides 10000/400 and cannot silently stand in for the requested manufacturer model. A validated PiPER tool adapter is absent; its coaxial transform is conditional offline geometry. No Panda model or gripper is substituted.

Accordingly, physical results for these three arms are **NOT RUN / MODEL OR MOUNT LIMITATION**, not task failures or successes. New physical episodes: zero, within the maximum of three. This is a limitation of the requested controlled physical study, not a new manipulation safety gate. The offline study proceeds; it cannot establish physical contact stability.

Primary sources: [NERO official URDF](https://github.com/agilexrobotics/agx_arm_urdf/blob/983788b58d7eae76511177f768d85877534917fe/nero/urdf/nero_description.urdf), [NERO firmware/torque interface](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/nero/firmware_reference.md), [UFACTORY controller scope](https://github.com/xArm-Developer/xarm_ros/blob/aad7e1611c9c46eb719045414394bfdd42dcb0f8/ReadMe.md), [FR3 dynamics](https://github.com/frankarobotics/franka_description/blob/7aeeddc449edf8d62b594f9e36a81da53e7796f9/robots/fr3/dynamics.yaml).
