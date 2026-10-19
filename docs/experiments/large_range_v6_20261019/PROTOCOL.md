# V6 preregistration — large-range interaction and passive RGB-D

Reference fe69627; preserve all previous physics/manipulation results. One blind
45936 episode, fresh wrist observations and automatic grasps. No archived grasp
or camera coordinates enter online inputs. Known hinge/state and registered base
remain; simulated wrist mount, ideal load cell, approximate actuator limitations.

Aim: 30 degrees actual opening and meaningful reverse motion; 45 only conditional
on current-state collision/IK, contact, resources, and complete safe-exit checks.
Engineering fixtures reached 45 in IK but both bounded parking searches failed
there; 20/30 exits passed. Those fixtures are eligibility evidence, not live poses.

Up to four intentional acquisitions, four Agent viewpoint adjustments, 32 protocol
operations. Each acquisition permits up to four predeclared finite segments under
unchanged safety. 5400 seconds wall / 3600 simulation, exit reserves 300/90;
launcher 5700 seconds with cooperative stop/export before last-resort termination.
These are bounds, not instructions to exhaust them. No valid-episode rerun.

At unloaded boundaries Codex inspects fresh wrist RGB/masks/depth/cloud and numeric
contact/model feedback. Targets/profiles chosen from current measurements, not
fixed object rules. Obtain varied acceleration/speed/direction FIT in overlapping
regions, expand as feasible, and complete a distinct-profile VALIDATION once the
expanded-range FIT is supported, before optional stretch exploration. FIT windows
are declared before each segment; interrupted transmitting prefixes retain original
failed-action status. Entire VALIDATION is never fitted or used for selection.

Estimator: V3.1 SG121 and tool compensation unchanged. Nonnegative local cubic
B-splines with 5/10/15-degree knot spacing and fixed regularization alternatives;
whole-FIT-trajectory cross-validation, >=3 trajectories and >=10% improvement
before selecting an extension. Unobserved resistance is unknown. Conditional
HAC intervals exclude model-selection/systematic contact/sensor uncertainty.

Two stationary 640x480 optical-Z RGB-D sensors are separate write-only owners.
Placement uses coarse cabinet envelope, not handle geometry. Sparse synchronized
pairs at actual motion checkpoints / reversal / exit; maximum 28 pairs. No fixed
camera image/depth/geometry is returned through Agent operations or estimator.
Geometry package contains only images, calibration, simulation time, sequence,
direction/phase and diagnostics. Privileged q/moving transforms live in separate
evaluation-only records. No hinge reconstruction in this round.

Compact 240 Hz interaction numeric data + wrist checkpoints + passive pairs;
target total <200 MB, no continuous RGB-D/video or redundant point-cloud archives.
Raw remains private. Hidden seed/parameters stay unread by Agent/estimator until
actor exit, all online decisions and estimates finish. Freeze commitment and code
before research execution; post-hoc evaluation must not mutate online models.
