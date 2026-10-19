# xArm wrist F/T V0 — commissioning design

Reference: `136b6e0`, Recovery V1.1. No baseline source is edited.

## Measurement boundary

The pinned official xArm/G1 URDF has `link7 -> link_eef -> gripper_fix -> xarm_gripper_base_link`. The new, separately generated assembly renames only `gripper_fix` to `wrist_ft_interface` and retains the fixed joint during import. Its child-joint origin and axes define the virtual sensor. It measures the load transmitted across the real simulated flange-to-tool joint through the native articulation spatial-reaction API. No contact sum or projected motor effort supplies its wrench.

This is an **idealized zero-thickness, infinite-stiffness virtual six-axis load cell**, not a CAD/dynamic replica of the purchased sensor. The pinned official robot description contains no sensor mechanical model. Tool position, existing masses/inertias/colliders and link7-mounted simulated camera remain unchanged. In particular, the manufacturer's sensor/adapter mass and thickness are **not** modeled. Hardware fit, camera clearance and actuator loading with an actual sensor remain unverified.

## Official references and conventions

- [UFACTORY installation](https://docs.accessories.ufactory.cc/6_Axis_Force_Torque_Sensor/2.Installation.html): sensor mounted to end flange through mounting flange; downstream tool interface must be checked against actual supplied kit.
- [UFACTORY specifications](https://docs.accessories.ufactory.cc/6_Axis_Force_Torque_Sensor/5.Technical_Specifications.html): AI1500 listed capacity 400 N / 20 Nm, resolution 0.4 N / 0.01 Nm, 600 g including adapter, 200 Hz. These specifications are documented **without claiming they were reproduced**. No sensor noise, clipping, crosstalk or precision is invented for the ideal interface.
- [UFACTORY SDK](https://docs.api.ufactory.cc/xarm_python_sdk_docs/7.-Force-Torque-Sensor.html): raw and gravity-compensated/filtered values are separate; `[Fx,Fy,Fz,Tx,Ty,Tz]` uses N and Nm. A single-posture zero is not general payload compensation. V0 remains non-force-control.
- [NVIDIA joint sensors](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_articulation_force.html): incoming fixed-joint spatial force is appropriate for mimicking an F/T sensor. The installed version is `6.0.1-rc.7+release.42383.32955d8d.gl`, so installed source is authoritative for API details.

Installed compatibility API `SingleArticulation.get_measured_joint_forces()` returns child incoming-joint local force first and torque last. Map by full joint metadata **plus one**, including fixed joints, rather than actuated DOF index. In the commissioning scene the fixed joint's metadata index is 7, reaction row 8. This must be determined at runtime in later scenes too.

## Calibration and compensation

Keep raw 240 Hz readings unchanged. Compare orientation-dependent tool gravity using the official downstream G1 masses/COMs and measured link poses. Fit constant sensor bias only from an unloaded static acquisition; validate on other orientations. Never zero under contact. Independently apply small known world forces and pure moments to the tool in the no-object calibration scene to verify all axes, action/reaction sign and nonzero force lever arm. Such externally applied calibration loads are **not permitted in door identification episodes**.

Tool inertial wrench is the sum of `m*a_COM`, `I*alpha + omega x I*omega` and moment arms. Derive accelerations from measured body trajectories, retaining native velocities separately; the baseline has a known native-velocity/readback discrepancy. The commissioning analyzer uses an explicitly labeled centered offline derivative filter, not an alleged zero-latency online estimate. No compensation is applied twice, and all compensation terms are exported separately.

For a parent-on-tool reaction `W_R`, Newton–Euler balance gives robot-on-object wrench at the sensor origin:

`W_robot_on_object = W_R - bias + W_tool_gravity - W_tool_inertial`.

The calibration must substantiate this convention before use. Transform both force and moment to world and shift moment to the explicit prior joint origin. Revolute effort is `axis dot moment_at_hinge`; prismatic effort is `axis dot force`. This interface accepts a kinematic axis prior, never hidden dynamics or GT object torque.

## Bounded protocol

One no-object commissioning scene; raw records and video private. Start door identification only if this primary sensor and compensation can be validated. Otherwise preserve the concrete sensor failure and stop before consuming any of the three identification configurations. The frozen grasp, closure, opening, parking and safety implementations are unchanged.
