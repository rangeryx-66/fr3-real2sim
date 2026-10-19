# V4 independent physical validation — registered before outcomes

References `0305095`, `b75d5b8`, `1c98f9b`. One fresh continuous normally closed
45936 episode, xArm7/native G1, registered station, known hinge/state,
simulated wrist RGB-D, ideal wrist load cell and approximate actuators.

One new private random seed draws the existing constant-resistance family:
mass scale [0.75,1.25], positive affine-density gradient each [-0.30,0.30],
Coulomb [0.015,0.045] Nm, viscous [0.10,0.35] Nm s/rad. Mass/COM/inertia
are generated together by the existing positive-density quadrature.
No robot/gripper parameters change. Commit private configuration by SHA256
before launch. Agent and estimators do not read its values/seed until actor
exit, online decisions and exports are complete. No replacement draw.

Frozen V3.1 production runner and worker, SG121 fixed, no coefficient/filter
tuning. Original phase-aware runtime and contact recovery remain unchanged.
At most two acquisitions (one original, one existing recovery after safe exit).
One valid episode only; no physical retry or saved grasp/viewpoint input.

Codex inspects fresh wrist RGB/mask/depth/point clouds, chooses reachable
observations (up to four adjustments), inspects automatic geometry/plan and
explicitly selects a finite probing block before loaded execution. No slow
Agent wait under unstable load. Online models update after complete segments.

Available unchanged motion menu: acceleration-rich internal speed plateaus,
velocity-rich steady upper-speed FIT, independent sine-speed VALIDATION.
Targets around 10–15 deg opening and 2–5 deg closing. A possible block is
FIT open plateaus -> FIT close plateaus -> FIT faster reopen -> complete
VALIDATION sine close. Actual targets/contact mode remain Agent choices
from current observations/contact. Profiles use existing 12 mm/s and
40 mm/s² ceilings. No required sequence/switch angle or stronger grip.

Target 30–40 min, <100 MB. Do not begin new loaded work after 35 min;
owner limit45 min allows a10 min release/park reserve, plus cooperative
export. Existing120 s unloaded Agent wait remains. Shared GPU capacity is
allowed; serialize only owned actors. No full continuous high-res video,
only compact240Hz essential channels and checkpoint RGB-D/event clips.

After actor exit, independently reveal authored parameters; preserve online
V3.1 packets unchanged. Run frozen V3 on read-only links to the SAME data,
same completed FIT/VALIDATION partitions and compare predictions.
Truth is post-hoc only; authored inertia is not independently read effective
PhysX inertia. Preserve failure, unsupported parameters and safe-exit status.
No estimator/control changes after seeing this episode's outcome.
