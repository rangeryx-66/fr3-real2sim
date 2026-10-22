# V7.5 bounded engineering protocol — not a new benchmark

Baseline: d72ee9160b5b97036208208aa905f27989bba279. Historical V7.2–V7.4 results are immutable.
Work is confined to new prismatic V7.5 modules; 71+22 revolute manifests, call paths,
configs, G1 authority, object resistance/colliders and existing emergency stops stay unchanged.
FAST_DEV: essential 240 Hz numeric contact/F/T, sparse wrist RGB-D, no continuous video.

## Order and finite scope

1. Read-only archive diagnosis: every V7.2/V7.4 prismatic slot, V7.3 engineering controls;
   primary mechanisms 19179 torque/reversal, 27044 traction/cost, 45746 exit. Preserve
   raw inputs and existing model hashes. Use native contacts only for diagnostic checks.
2. Software/synthetic checks: current-attempt evidence, timing admission, periodic joint
   winding and whole-chain collision preflight. Estimator comparison: old versus V7.3
   eligibility, equal-count/time-stratified control, independent sliding momentum estimate;
   same model family LOFO and contiguous-block bootstrap, complete held-out isolation.
3. Native calibration (only after server/archive availability): one short unloaded matching
   motion + known externally applied tool load fixture; one bounded normal-contact versus
   blocked-motion contrast. At most four short fixture runs including an exact-issue repair.
   No new task-load limit is activated without measured normal/abnormal separation.
4. Fresh Agent regressions: at most one continuous engineering scene for each of
   19179,27044,45746,45290, with <=3 actual closure attempts/scene and finite measured-cost
   admission. Fresh wrist observations and automatic current-state candidates; original
   one-second hold, bidirectional informative FIT then complete distinct VALIDATION,
   release and actual verified park. Safe ordinary recovery stays in the same scene;
   hard latches are terminal. All outcomes retained. No object reset or cached/GT pose.
   Any necessary second development revision requires a written root cause and separate
   attempt ID; it is never a replacement for the first result.
5. Only after those readiness checks: freeze repaired sources/configs and preregister
   two prospective slots (one fresh-dynamics existing object, one ordinary drawer not
   used for tuning, chosen by geometry/eligibility before outcomes). Do not choose IDs
   or seeds from successful contact scores. No eight-slot batch or automatic expansion.

## Load and estimation assumptions, frozen before testing

No increase from the current 4 N pad / 8 N wrist / 0.6 N m sustained bands is proposed
from archived peaks. These are simulator engineering limits, not manufacturer ratings.
Keep all six wrench components and original emergency, joint, collision and actuator
checks. A causal protective signal may use present/past samples only. Offline SG121
and centered full-tool compensation are identification/diagnostic operators only.

The identified interface is the official seven-axis xArm model plus pinned official
G1 (position/speed, 84 mm, nominal maximum clamp 30 N) and ideal virtual flange/G1
cut. Hardware serial-specific calibration and independent motor drive force are
unavailable. AI1500 (400 N /20 N m, stated resolution 0.4 N /0.01 N m, 200 Hz) is a
reference, not the implemented sensor's calibrated accuracy. No payload-to-pull-force
conversion or hardware safety certification is made.

Perturbation scenarios are applied to raw six-axis sensor/robot samples BEFORE
compensation/filtering, separately in each FIT/VALIDATION block:
- constant bias +/-0.01 N and +/-0.001 N m (assumed calibration scenarios);
- uniform quantization 0.4 N/0.01 N m (official stated resolution, NOT Gaussian SD);
- independent Gaussian force noise SD 0.001/0.01/0.05 N and torque SD 0.0001/0.001/
  0.005 N m (explicit scenarios, NOT official noise specifications);
- F/T delay 1/2 native samples (assumed 4.17/8.33 ms), no cross-segment borrowing;
- downstream tool mass +/-1%, COM +/-1 mm (assumed calibration error, recompute gravity
  and inertia consistently; do not compensate twice);
- joint-position quantization 0.1 mm and Gaussian 10 micrometre SD (assumed state
  estimation scenarios, NOT a conversion of robot repeatability to sensor accuracy),
  with velocity estimated from those positions rather than ideal native velocity only.
Use deterministic scenario seeds, independent of object hidden parameters. Do not
select filters, priors, coefficients or trust labels against hidden truth.

Bootstrap: 32 replicates, fixed 1 s contiguous FIT blocks within original trajectories;
LOFO holds out whole FIT trajectories. Keep selected model family/basis/smoothness,
compensation and VALIDATION unchanged. Matched eligibility control uses the original
eligible samples with the qualified count in fixed 1 s time strata, selected uniformly
by index, not by torque, acceleration or parameter error. Report unmatched/empty strata.
Momentum estimator uses nonoverlapping 0.5 s windows wholly within qualified sliding,
with integrated measured force, endpoint velocity, displacement and the SAME friction
basis. It is an independent diagnostic, not a replacement tuned to authored mass.

Synthetic non-model-matching plants include distinct static/kinetic resistance and
opening/closing asymmetry. Preserve unsupported/unknown results and disagreement;
small fitted residual is not an acceptance criterion.

## Acceptance and publication

Report first contact, actual pull/reverse, independent physical VAL, retained prediction
horizon, release and park separately. Report model availability, excitation retained,
parameter/forward prediction errors and assumptions. Error distributions include N/A
counts. Successful engineering scenes do not form an unbiased benchmark rate.
No prospective test or reliability PASS until native-contact and whole-lifecycle
readiness is demonstrated. Preserve failures, source hashes, protocol changes and
owned-process cleanup. Raw measurements stay private; publish concise derived evidence.

## Pre-contact implementation registration

No load band is increased. The isolated runtime forecasts the existing trailing
150 ms sustained-load window at the measured stopping horizon using its past
samples and a linear continuation of present raw load norms. This requests the
original bounded deceleration and guarded release, returning SAFE_PARTIAL_PROGRESS;
it never resets or supersedes the original hard guard. The interrupted action is
not target success and is not automatically FIT-eligible. This is a conditional
forecast, not proof that a hard stop can always be avoided. Replay all eight V7.4
episodes before first fresh contact; report every early advisory, including any
on an originally complete action.

Candidate feasibility extends the original CENTER envelope and original preload
mapping through the Agent's declared minimum pull/reverse/validation chain and
release/exit. Native checks remain authoritative while moving. Scores/weights
and observed contact surfaces are unchanged. A nominal preflight cannot prove
loaded contact success: the archived 19179 reverse-collision case also passes
nominal full-chain checks and remains a counterexample to that inference.

Parking adds only legal 2*pi representations of the original rest regions,
derived from actual URDF joint types. Every complete path and measured endpoint
still passes the original collision/joint/unloaded checks. No joint teleportation.

Recovery admission counts the current return as actually executed waypoint by
waypoint, timed approach and exit, the original finite closure+hold bound, and
each planned FIT/VALIDATION's ProgressClock duration plus observed additive
execution excess. Native-step wall cost is derived from all eight V7.4 runs;
current measured observation cost covers refinement. These are forecasts, not
new deadlines or contact-success guarantees. Original exit reserves stay intact.

The first sensor fixture had zero valid load samples: its recorder action omitted
coordinate_unit and cleanup masked the initial error. Preserve the failed folder.
The repaired interface passed preflight and completed 11.733 sim s /39.65 wall s
of unloaded motions and known six-axis small loads. The abrupt 9 N tool-load
contrast correctly hit PHASE_TCP_VELOCITY first; preserve that result. A new
predeclared 1 N/s tool-load ramp then triggered original SUSTAINED_OVERLOAD over
the original 150 ms window, without changing any guard. These fixtures are TOOL
calibration/protection checks, not drawer-contact or physics-identification PASS.
