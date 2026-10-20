# Prismatic contact-transmission investigation (in progress)

Revolute source/configuration and external robot/camera/closure files are frozen
in REVOLUTE_FREEZE.json and REVOLUTE_EXTERNAL_FREEZE.json. No original source,
physical controller, safety threshold or six completed research outcome is edited.
The registered benchmark remains paused and hidden dynamics remain unread.

Status: **engineering in progress, not PASS**. The optional prismatic-only
contact-retention implementation is disabled by default and has only offline
validation at this revision. It is not loaded by the ongoing 45290 comparison.
Its predeclared conditional native test is in PLAN.md. No original revolute
source or G1 closure source is edited.

## What is established before new outcomes

The 0.3 N rail resistance is really applied. Independent known-force fixtures on
27044 and 45290 resolved opening and reverse breakaway in (0.300,0.325] N and
showed actual travel. This is engineering evidence, not robot identification.

The previous 27044 repair enabled 7.3046 mm actual pulling with a 6.238 mm interval
of nearly constant TCP/drawer relation. Its later stall and 0.7056 mm incomplete
reverse remain failures. 45290 obtained bilateral contact but produced only
0.0169 / 0.0352 mm travel and about 1.9 mm actual TCP/drawer relative displacement.
Command-to-measured TCP deficits were separately 0.737 / 0.541 mm, not that slip.

At 2–5 s in the archived 45290 first pull, mean friction force **on the object**
per finger was approximately [0.1951,-0.0892,-0.0004] and
[-0.0001,-0.0806,0.0000] N in world XYZ, with rail direction -Y. Thus substantial
traction acted along the bar rather than the rail. The next grasp's corresponding
vectors were [0.1568,-0.1574,-0.0004] and [0.0510,-0.0336,-0.0001] N. Normal rail
projection was negligible. Neither positive bilateral normal reports nor total
force magnitude certifies axial pulling. Signed friction is the native separate
friction-anchor getter; normal impulses alone are not complete forces.
Fully compensated wrist and native total rail effort agreed to about 0.0164 N
RMS for the first pull, much smaller than its sustained breakaway deficit.
This does not eliminate compensation uncertainty or identify a true coefficient.

Actual native reported contact points cluster near a lateral pad edge (~0.573 mm
margin) while geometric predictions often prefer more interior support. These
are discrete manifold points, **not pressure areas**, so their edge clustering
alone cannot prove that the whole physical contact patch was edge-supported.
No material coefficient is inferred solely from force/normal ratios.

Archived measured finger-body poses also constrain the coupling hypothesis:
actual inner-plane gap minus master-angle aperture was approximately
-0.034 to +0.057 mm / +0.049 to +0.146 mm in the two 27044 probes, and
-0.017 to +0.075 mm / +0.014 to +0.081 mm in 45290 (10 Hz diagnostic samples
from the original 240 Hz archive). Opposing-normal orientation errors stayed
below 0.055 degrees. This does not rule out small contact-load redistribution,
but it does not support a millimetre-scale master/mimic mapping or gross finger
frame error as the explanation for ~1.9 mm TCP/drawer relative motion.

## PiPER versus native G1

The previous `run_45746_regrasp_clean.py` uses actual-state +2 mm reference steps,
actual-joint-seeded IK and a retained loaded command bias. It switches PiPER to
explicit per-finger effort hold. The xArm native G1 correctly uses finite master
position control with five native mimic constraints, translating the unchanged
legacy preload into a bounded position target. It does not have independent
PiPER-style effort-controlled fingers.

Current `Runtime.begin_reference` **already preserves loaded command bias**.
A missing-bias claim is therefore ruled out. However, subsequent differential
IK is seeded from the prior nominal command, rather than each measured arm state.
This is a plausible tracking adaptation to test, not an established unique cause.
The isolated prismatic prototype reuses the same QP, gains, loaded bias and all
velocity/acceleration/collision checks, with measured seeding during loaded motion.
Its measured-state seeding alone is **not a demonstrated repair**: the new
27044 engineering episode acquired and held both fresh grasps, then obtained
2.2181 mm / 0.01748 mm pulling before the original finite transmission detector
stopped. Both were released and collision-checked to native-rest parking. Neither
reverse motion nor independent validation executed. The fresh grasp differs from
the archived one, so this is not a controlled same-pose controller A/B result.

The old PiPER 45746 scene used a semantic bar box plus two support boxes, a
station [0.5,-0.55,-0.1,150deg], and allowed unloaded mobile-base repositioning.
The current native-G1 comparison retains the existing per-source-piece convex
colliders, registered xArm station and fixed base. Both select the same named
prismatic joint and original-46 handle, but URDF geometry hashes differ. The old
selected joint has no explicit SI Coulomb/viscous law in its URDF; the old passive
resistance author only supports revolute joints. Consequently old success is not
an isolated same-physics robot comparison, and its effective breakaway cannot be
assumed equal to V7's independently verified 0.3 N. No legacy proxy, force gain,
station or resistance is silently copied into the new experiment.

## Isolated candidate change under evaluation

The frozen task feature uses the most optimistic pad's opening cone score.
The new **prismatic-only** adapter instead evaluates both rail directions and
uses the weaker direction's mean two-pad capacity. It retains existing native
pad geometry, observed surfaces / explicit hidden hypotheses, initial contact
accessibility, retention perturbations, candidate families, shortlist budget,
weights and all native execution checks. Equal unit-normal pad loading and mu=0.5
remain declared ranking assumptions, not measured traction, a success probability
or a hard rejection threshold. Archive replay still ranks an already failed
45290 family highly; scoring alone is not demonstrated to repair it.

32 deterministic tests (including all 26 original V7 tests), the unchanged
fast-runtime safety/protocol fault-injection regression and adaptive-segment /
validation-isolation regression pass locally. These are not native-contact PASS.


## Current commissioning readbacks

The new engineering entry passively queried the installed native tensor API
`get_material_properties`: both G1 fingers report static friction 0.5, dynamic
friction 0.5 and restitution 0. The installed API docstring confirms this field
order. These readbacks are preserved privately and were not modified. The handle
pair's combine mode and the active contact-anchor dynamics are not inferred from
the finger values alone. Insufficient observed axial force therefore must not be
presented as a confirmed low-material-friction initialization defect.

## New 27044 contact evidence

The first pull's commanded TCP gain was 4.9990 mm, measured TCP gain 3.6825 mm,
and drawer gain 2.2181 mm. Incremental command-to-TCP deficit (1.3164 mm) and
actual TCP-to-drawer rail motion (1.4645 mm) are distinct quantities. Transverse
relative motion reached 0.1432 mm; no gross transverse displacement is invented
to explain this result.

The Agent inspected the actual feedback image, safely released, captured fresh
wrist RGB-D from the opposite side, and accepted a different automatically
generated roll-180 candidate after complete current-state return/approach checks.
That recovery also failed to transmit the 0.3 N native breakaway load. It was not
an unchanged candidate retry or a scene reset.

Alignment to GRASP_VERIFIED shows a difference between the two holds. First:
L/R 0.434/0.561 N became 0.503/0.504 N after one second; TCP Z changed only
0.00357 mm and aperture 11.2488 -> 11.2584 mm. Recovery: 0.475/0.512 N became
0.126/0.813 N; measured TCP Z changed +0.03512 mm, aperture 11.2763 ->
11.2413 mm with an unchanged 10.6827 mm command. Unloading begins before rail
demand in the recovery. A fixed-command hold, small settling and contact-load
redistribution remain relevant; larger aperture opening is not a prerequisite
for this failure. This does not establish one unique causal mechanism.

Force-weighted native reported contact centroids during both holds also remain
approximately 0.573 mm from a lateral pad edge, with longitudinal margins
9.37–10.69 mm. Thus insufficient longitudinal insertion alone is not established.
Native reported manifold points/centroids are not measured pressure areas.
Missing positive-normal reports alone do not prove physical separation.

An independent **robot-only** cooked-geometry audit further explains that
0.573 mm number. The actual G1 inner contact plane is a five-vertex polygon
(1017.49 mm²), whereas the scoring rectangle has area 1106.25 mm². Its positive
lateral boundary is 14.1771 mm, versus the official rectangle's 14.7500 mm.
Native reported points are effectively on that cooked boundary (median boundary
distance below 0.00001 mm), not 0.573 mm inside a physical rectangular pad.
The cooked plane's Z coordinate matches the official plane. No shape or contact
offset is changed. This bounds a representation discrepancy (~8.0% planar area),
but does not prove it causes the entire traction deficit or recover the unknown
pressure/friction distribution. See NATIVE_PAD_GEOMETRY.png/.json.

One malformed profile serialization was rejected by the protocol with no robot
motion; the corrected command used the already inspected plan. The episode ended
at verified unloaded parking after two physical acquisitions, wall time 3143.04 s.
The remaining finite resources did not support another observation/planning/
acquisition cycle. No physical result was retried or erased.
