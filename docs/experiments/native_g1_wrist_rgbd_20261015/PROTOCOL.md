# xArm7 rigid wrist RGB-D — bounded protocol

Reference: `dba947d`; branch: `codex/xarm7-wrist-rgbd-views`.
Label: RGBD_SAM3 / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION.

1. Offline view reachability/visibility: 45600, 45606. Archived observed RGB-D can diagnose visibility; it supplies no live grasp pose.
2. Exactly one observation-only 45600 scene: native arm actuators execute survey and up to two observed-geometry views, then automatic visual grasp planning. No handle grasp in this scene.
3. Only if observation/plan are usable: freeze source/configuration, run at most one fresh 45600 manipulation scene with wrist acquisition, unchanged closure, one-second hold, approximately +3° opening. Existing ≥2° post-acquisition outcome is reported separately from full task success.
4. No manipulation retries, scene-camera depth fallback, GT handle surface scoring, saved grasp transform, parameter tuning or additional cases. Pre-physics initialization repairs are infrastructure repairs.

Camera is an explicitly assumed simulation design, a USD child of the measured link7 rigid body. Its local pose is set once; no camera world-pose setters occur in acquisition. Camera housing/cable mass and collision are not added to the frozen robot and remain a deployment limitation. It is not a calibrated hardware mount.

Initial survey uses only the coarse closed-cabinet collision-envelope midpoint/front and fixed quarter-height sweep, not a handle bounding box. Subsequent views use current wrist metric points/panel normal. Native G1 geometry, known station, hinge/state, scene collisions and robot calibration remain dependencies. Scene cameras provide video only.

All observations save RGB, optical-Z depth, K, measured joint state, actual camera world pose, FK and rigid-transform diagnostics, self-exclusion mask and time. Robot motion/contact telemetry stays at the original 240 Hz. Video/raw observations remain private. No new manipulation quality or telemetry gate.

Frozen baseline source files are reused without edits. Only new wrist sensor, view-route and source-adapter files are introduced. Own Isaac actors run sequentially on a GPU with sufficient free memory and close after export. Public deliverables contain compact reports/metrics/provenance only.
