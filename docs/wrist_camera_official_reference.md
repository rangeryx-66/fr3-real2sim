# PiPER camera reference

Verified 2026-10-06 against the AgileX Robotics published example:
https://www.hackster.io/agilexrobotics/control-piper-arm-with-hand-gestures-7c2779

The example lists two optional RGB-D cameras:

- RealSense D435, aligned RGB/depth, 640×480 at 30 fps.
- Orbbec Petrel, aligned RGB/depth, 640×400 at 30 fps.

This establishes compatibility, not a universal PiPER wrist camera or a
factory camera-to-TCP transform. The example uses camera intrinsics for depth
projection; its gesture base-frame calibration is not wrist hand-eye calibration.

The experiment uses `configs/wrist_camera_d435_nominal.json`: D435-class nominal
1280×720 RGB intrinsics and an explicit simulated mount. It does **not** claim
the example's capture configuration or a measured per-camera calibration.
Real deployment requires aligned-depth CameraInfo, optical-frame convention,
hand-eye calibration and mount clearance verification. Existing runs retain
their frozen nominal configuration; this documentation does not change images,
grasp targets or physical safety settings.

## Rendered simulation mount check (2026-10-06)

The original nominal TCP optical translation `[0, -0.075, -0.100] m`
produced 17.7076% gripper pixels at two physically reached scan poses.
A separate render diagnostic at recorded robot posture compared offsets
`y=-0.075/-0.100/-0.125 m`: robot pixel fractions were
`0.1770757/0/0`. This diagnostic does not count as robot capture.
Official visual/cooked-envelope checks also found the original housing
overlapping `gripper_base`; the two larger lateral offsets cleared it.

The new `wrist_camera_d435_clear_mount_sim.json` uses `[0,-0.100,-0.100] m`,
with the same nominal intrinsics. It is a simulation mount design, not a
manufacturer mounting transform or measured hand-eye calibration.
Scan housing checks now include all robot colliders, including the wrist
cluster. No robot geometry, contact safety, or image QA threshold changed.

Old cropped/occluded images and interrupted runs remain in their original
output directories. The new run inherits the original total start time
and elapsed capture budget, and reexecutes physical initial actions.
It does not restore object joint states or copy contact impulses.

## Paused capture verification

A separate three-view render check using the installed Replicator standalone
`step(rt_subframes=8, delta_time=0.0, pause_timeline=True)` returned valid
RGB-D while `world.current_time` remained exactly 1.8833334315568209 s
before and after every view. Plain paused `world.render()` had returned
no RGB for a newly initialized sensor. ORACLE capture now uses the tested
zero-time annotator scheduling call and records its implementation hash.
This neither commands nor locks the object joint, and remains diagnostic.
Evidence: `results/wrist_mobile_20261006/mount_paused_qa/mount_qa.json`.
