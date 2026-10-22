# Two bounded prospective checks — not a new benchmark

Freeze: unchanged R2 physical implementation `bc81d50`, unchanged production
V7.3/V3.1 qualification and filtering, and original 71+22 revolute dependencies.
No post-engineering control, score, load-band, actuator or collider tuning.

Four fresh R2 objects completed local FIT/VALIDATION/release/park. R1's valid
physical failure and 45290's zero-acquisition orchestration failure stay in the
record. The additional matched unloaded calibration completed, with zero object
motion/contact and verified parking. These establish **local engineering**
readiness; they do not establish population reliability or long-range coverage.

Preregister exactly two continuous prospective scenes, in this order:

1. `v75_prospective_45290_00`: known development geometry, **new blind dynamics**.
2. `v75_prospective_19855_00`: previously untuned ordinary single-horizontal-joint
   table drawer, new blind dynamics. Its **round knob** is outside the validated
   long-bar geometry domain. This is an explicit applicability-boundary check,
   not a claim of same-distribution unseen-bar transfer. Preserve a perception
   or planning failure if the frozen pipeline cannot support it; no replacement.

19855 was the first remaining ordinary single-joint table in the original
metadata order after engineering objects and earlier documented exclusions.
Offline geometry review first marked it outside the bar-only domain. Two
additional bounded geometry alternatives (20453,22241) did not establish a clear
standard-bar target. Rather than keep searching for an easy geometry, the
prospective boundary case is fixed to 19855 before any physical result.
Its original geometry has one horizontal rail, 412 mm authored travel and a
standard exposed knob; the original station passed coarse IK and native
home-collision checks. This does not prove a grasp. No executable grasp/view is
exported from preflight. A missing legacy manifest limit alias was repaired
before any Isaac scene: metre limits copied without conversion; URDF and native
collision geometry unchanged. The failed preparation record is retained.

Both use independent seeds and physically consistent positive-density
mass/inertia scaling from the existing prior: mass scale [0.75,1.25], density
gradient [-0.3,0.3], damping [1,6] N s/m, Coulomb [0.2,0.6] N. One constant and
one localized passive resistance law are secretly shuffled. Existing amplitude,
centre and width ranges remain unchanged. Commit the private configuration
digest, per-job digests and loaded-source hashes before launch. Neither seed nor
parameters/family assignment enters Agent packets or fitting. Reveal truth only
after both actors, decisions and online models finish.

Shared Agent instructions: inspect actual fresh wrist RGB/masks/depth/cloud;
resolve the intended moving-part handle from observations and safety feedback;
select feasible candidates and initial 1.0 N **requested** preload. At verified
unloaded boundaries the original 0.5/0.75/1/1.5 N menu remains available based on
feedback. Use no cached successful grasp or camera coordinate. Native collision
identities validate safety, never snap an observed pose or supply contact scores.
On ordinary failure inspect concise current evidence, safely reobserve and
regenerate alternatives in the same scene; hard stops are terminal.

The initial local menu offers 20/6/17/10 mm endpoints: acceleration-rich FIT,
slower reverse FIT, varied-speed FIT, then a predeclared distinct-speed complete
VALIDATION. The Agent may select a smaller supported menu **before execution**;
minimum full validation/exit admission is required before acquisition. Initial
profile speeds/accelerations are the already tested R2 values. Do not equate
this local task with the old 80–150 mm goal or infer unobserved resistance peaks.
Three distinct eligible FIT trajectories are required even to consider the
position-dependent extension; FIT-only selection and complete VAL isolation
remain unchanged. Stop after useful complete validation and parking, or a real
terminal blocker; no optional range expansion.

Per scene: original finite 3600 s wall /1800 s simulation, 300 s wall /120 s
simulation exit reserve, 360 s unloaded command window, at most 3 acquisitions
and 40 operations. No physical rerun or replacement after a valid result.
Fresh observations and safe native actuator motion throughout. FAST_DEV,
essential 240 Hz numeric evidence and sparse RGB-D, target <100 MB/scene.

Report every first-attempt/planning/recovery failure, real pull/reverse, complete
physical VAL versus retained prediction horizon, mass/damping/resistance error,
uncertainty/identifiability and verified final state. Conditional covariance is
not calibrated physical accuracy. Authored/readback mass remains distinct from
an independently measured effective PhysX mass. This is a two-case applicability
check, **not** an unbiased benchmark success rate. No further batch follows.
