# 7320 continuous-contact mobile manipulation experiment

**KNOWN_MODEL_DIAGNOSTIC. Primary milestone PASS; continuous-contact90° NOT achieved.**

The successful physical episode demonstrated closed → physical grasp →19.85° → retain jaw effort and physical contact → actual29.999mm driven base translation with arm compensation →30.43° (+10.63° after translation). No intentional release or regrasp, attachment, direct object force, runtime object-joint command, base teleport or scene stitching.

| Episode | Maximum actual | Completed coordinated base moves | Regrasps | Stop |
|---|---:|---:|---:|---|
| Fixed-base control03 |56.252°|0|0|Current IK branch exhausted; not a mechanical limit|
| World-TCP prototype03 |61.659°|2 plus a partial third|0|Original contact-loss protection; scene paused/alive|
| Moving-body-following variant04 |19.947°|0|0|Original5mm/s world-TCP protection; scene paused/alive|

The world-TCP prototype's basic segment passed. Its extension ended at60.890° after a short unloaded contact interval during the third translation. The fingers remained closed and were closing; uncontrolled detachment was not established. The initial-grasp-relative motion proxy reached approximately10mm, although world-target tracking error remained below20µm. The subsequent moving-body-following attempt did not pass the primary milestone and is retained as a failed variant. Neither stop is claimed as the authored/mechanical limit. No physical protection or clamp effort was relaxed.

The old visual-state90° pipeline at815fd0c remains frozen, with success and failure evidence separately preserved. The new code is on codex/piper-continuous-contact. The scene adapter adds physical driven XY prismatic joints; it does not write the base root pose during motion. It converts the existing static pedestal to a100kg dynamic support and adds 1 kg carriage/anchor bodies. This is a simulation-stage model, not a wheeled-base or hardware performance demonstration. Arm/object geometry, masses, friction, gains, force/contact thresholds,0.05rad margin and5mm/s world-TCP protection remain unchanged.

Known grasp geometry, hinge geometry, actual articulation/moving-body state and collision models are used. This experiment is not GT-free visual execution.

## Published scope

This branch publishes source code, operational configuration and the experiment report only. Videos, raw telemetry, contact-point dumps and other experiment evidence are retained locally and on labserver_inschool; they are not included in this publication.

The source/config hashes below identify the exact tested snapshots. The world-TCP prototype passed the basic20°→3cm→+10° segment; its90° extension failed. The body-following runner is a failed experimental variant, not a frozen successful skill. All physical scenes remain paused, and no further physical action has been taken during publication.
