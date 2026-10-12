# xArm7 shared door backend

Scope: **XARM7 / ORACLE_HANDLE / KNOWN_MODEL / APPROXIMATE_ACTUATOR_SIMULATION**.

The xArm7-specific runtime adapter loads the pinned official UFACTORY arm, restores the official mass tensors explicitly in the native stage, enforces sourced finite effort/speed limits and provides implicit PhysX position/velocity drives. P/D references come from the pinned manufacturer's simulation examples; integral action is not implemented and gravity compensation is a drive equilibrium offset. This is not a hardware-calibrated servo.

The entry point is `scripts/run_xarm7_shared_door.py`. It reuses the frozen common closure-continuity repair, smooth pre-contact trajectories, Pink differential IK, collision/contact protections, temporal force policy, 0.05 rad joint margin and 5 mm/s loaded-TCP guard. Only robot geometry, joint count and low-level drive installation are adapted; no object-specific controller is created.

The gripper remains the frozen PiPER hand with a **virtual coaxial tool adapter**. No validated real adapter, electrical compatibility or real xArm grasp performance is claimed. Intended future standard UFACTORY gripper requires its own tool/mass/collision/contact mapping and validation; it is not silently substituted here.

Registered contact cases are **45134** and **45443**, one fresh closed-start attempt each, fixed base, no releases or regrasps. Existing automatically generated xArm7 plans from `4b73580` are reused. Source/configuration is frozen before physical outcomes; no case replacement or tuning is permitted.

Raw native velocities and position-difference velocities are both retained. They differ under the native friction/constraint solve; this diagnostic is not a new execution gate. Original measured-TCP and joint speed protections remain unchanged.

See `REPORT.md` for actual results and limitations when the registered batch completes. Full evidence and source hashes accompany the report. No door outcome is inferred from offline IK coverage.
