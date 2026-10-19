# Agent-in-the-loop wrist perception — preregistration

Reference `d2ad962`. One normal-closed 45600 Isaac scene, registered station,
native xArm7/G1, same fixed simulated wrist-camera calibration. No contact
grasp or door opening. At most four Agent-selected viewpoint adjustments.

The initial pose is the existing generic coarse-cabinet survey initializer,
recomputed from the current model/robot, not a saved successful observation.
After this one initial pose, no fixed survey runs. Codex must inspect actual
RGB, masks including clipped candidates, colorized metric depth and local
point cloud before choosing each next camera eye/look target. Commands retain
brief interpretation, missing information, expected benefit, inspected file
identities and measured outcome. No GT handle decision input.

A small JSON command inbox exposes observe, inspect, evaluate_view,
move_and_observe, build_geometry and finish. It is an experiment interface,
not an autonomous fixed-rule policy. Physics pauses while Codex examines data;
the same scene and finite-effort actuator state survive. Normal actuator
commands execute all motion. Existing IK/collisions/limits are unchanged.

Frozen SAM3 confidence and valid-mask filters are reused literally. Rejected
mask files are retained only as partial observation cues. Only accepted masks
can seed the frozen measured-patch fusion/grasp rank. Unknown rear geometry
stays separate. No grasp-score, force, actuator, physics or controller changes.

Known station, coarse cabinet survey envelope, robot calibration, hinge/state
and scene collision models remain dependencies. Scene cameras provide video
only. GT handle surfaces may enter a separate evaluator after decisions end.

No other object, retry episode, manual grasp pose, VGGT or unknown-axis module.
All raw data/video private; publish code, compact trace and report. Close own
actor after export. Shared GPU capacity is allowed; never touch other users.
