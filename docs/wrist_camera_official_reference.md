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
