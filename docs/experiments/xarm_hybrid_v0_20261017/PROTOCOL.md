# Hybrid pull–push–planned-regrasp V0

One development episode on 46427; simulated wrist RGB-D, known hinge/state and registered station, approximate xArm/G1 actuators, ideal virtual wrist load cell. Frozen recovery 136b6e0 and sensor pipeline d6e690f remain unchanged.

Planned contact modes: handle pull toward 10°; guarded release and safe unloaded park; fresh wrist observation; compact G1 in free space; observed moving-panel push toward a smaller nonzero angle; separation and safe park; fresh visual handle planning and one planned regrasp; pull toward 15°; guarded final park. Targets are exploratory and do not override original protections. At most two handle acquisitions, one panel-push attempt, no unexpected recovery acquisition or valid-episode retry.

Panel contact uses only actual native distal fingertip geometry. One nominated moving-panel collider may contact those surfaces only in explicit PANEL_PUSH mode; frame, handle, other robot and environment restrictions remain. The point is selected from inspected wrist RGB-D, not snapped to native geometry. The central visible panel's missing physical collider is retained and documented; visually observed moving door rails are legitimate alternative surfaces for offline testing.

The panel wrist envelope uses 2 N and a moment equivalent to 2 N at the measured sensor-to-tip lever, 50 ms windows and three sustained windows, with 10 N equivalent simulation emergency protection. These are conservative development limits, not hardware certification. Existing protections are unchanged. Unfiltered wrist reactions, gravity compensation and residual inertial loads are retained; complete inertial compensation is applied separately for measurement processing.

Each segment is saved independently before further modes. Numerical updates use wrist F/T and known motion only, strict segment-local preprocessing, and complete validation segments. Model failure is reported without replacing unknown inertia with hidden truth. Slow Agent decisions and estimator processing occur only at verified unloaded boundaries.

Raw RGB-D, video, native 240 Hz streams, hidden randomized dynamics and detailed evidence remain private. A planned regrasp is not an unexpected recovery and does not erase any failed first interaction. No population-level claim or complete physical twin will be inferred from this case.
