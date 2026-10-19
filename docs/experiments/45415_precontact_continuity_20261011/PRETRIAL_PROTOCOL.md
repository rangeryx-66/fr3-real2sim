# 45415 pre-contact continuity — one registered physical trial

Classification: **ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT**.
Frozen reference: `e1e534c`, verified 30.000082° physical opening on handle `original-40`. The old runner, closure implementation, Pink controller, plans, station, geometry, gains, effort, physics and safety protections stay unchanged.

The archived 441.113 mm/s PREGRASP maximum is a broad fast section of a cubic joint-space move. Jacobian prediction gives 439.095 mm/s, so it is not an isolated sensor or handover artifact. Retiming the same joint-space segments with a rest-to-rest quintic and Cartesian-tangent-derived duration addresses this cause. Nominal reference speeds are 30 mm/s for free-space pregrasp and 3 mm/s for approach. These generate commands; they are not new physical acceptance gates. Existing loaded 5 mm/s protection remains active.

The separate 100.785 mm/s SLOW_CLOSE entry occurs when measured-state IK replaces the gravity-deflected position-command equilibrium, converting its offset into one-step velocity feedforward. Retain the command-minus-measurement offset acquired at this handover, while continuing to seed IK from actual feedback. The JawCenteredClosure aperture repair and Pink opening excerpt are unchanged.

Offline checks compare independent URDF and Pinocchio FK, finite-difference and analytic Jacobians, simulator FK, exact trajectory endpoints, boundary velocities, the recorded closure-entry commands and explicit runner/helper source hashes. Cooperative-stop and SIGINT export fixture tests consume zero physical attempts. Offline results are not physical validation.

Exactly one new normal-closed-start 45415 attempt uses the exact saved plan and fixed station. No retries, other assets or physical parameter changes. Save continuous video and complete 240 Hz streams; close the owned finished actor after export. Report all-phase and loaded speed independently, grasp, angle, slip, margin and contact loads, including failures. Raw archives stay on the server and local workspace; source and compact report/evidence metadata are published to GitHub.
