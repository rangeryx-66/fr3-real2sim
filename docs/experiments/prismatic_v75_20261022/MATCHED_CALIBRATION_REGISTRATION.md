# Additional matched unloaded calibration — before physical outcome

The original four sensor-fixture records stay intact (including the initial
infrastructure failure and the abrupt-load velocity stop). They verify signs,
small known loads and original protection, but their unloaded joint-six motion
is **not** the actual drawer-probing motion. One additional calibration fixture
is justified to address that remaining measurement-chain ambiguity.

- At most one completed matched-unloaded fixture, after the four fresh drawer
  engineering scenes finish. No extra grasp/opening or benchmark episode.
- Replay the first two complete measured FIT TCP motions of
  `v75_19179_fresh_01_R2`, with identical timing/orientation, translated 40 mm
  away along its **observed** panel normal. Cached robot motion is explicitly
  diagnostic-only and can never be an online grasp or camera fallback.
- Use median recorded jaw width in free space via the original bounded jaw
  release-speed command, original xArm/G1 actuators and all safety checks.
- Offline native IK/collision preflight passed 443 configurations with **no
  intended-contact waiver**. Revalidate routes and paths from measured state
  again after initialization. If unavailable, preserve the limitation; do not
  relocate the object, change colliders or force a path.
- No external tool load in this fixture, no object force/joint command. Confirm
  zero pad load and stationary object. Finish by original guarded full parking.
- Register all motion as `CALIBRATION_NOT_FIT`. Preserve raw six-axis F/T and
  actual body/robot states at 240 Hz. Compare raw, gravity and complete inertia
  compensation offline; do not change production bias, filters or coefficients.
- Report actual tracking mismatch and that free-space dynamics cannot reproduce
  loaded contact compliance. This is an ideal virtual-sensor check, not a new
  hardware-calibrated noise or confidence bound.
- Finite 720 s wall /300 s simulation fixture budget, with 120 s wall /60 s
  simulation exit reserve; launcher timeout includes a separate safe export
  margin. Manage only the owned actor under the existing lock.

The original V7.2/V7.4 results and the 71+22 revolute freeze remain unchanged.
