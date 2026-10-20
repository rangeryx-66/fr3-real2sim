# V7.1 preload hypothesis — preregistered engineering comparison

Reference 780b1ab. Frozen revolute 71+22 hashes remain authoritative; no blind
benchmark launch or hidden configuration access. No integral IK or transverse
retention prototype. Keep the existing measured-state/fixed-bias prismatic arm
backend identical across conditions, with original guards and G1 drive caps.

First compare requested preload 0.5, 0.75, 1.0 N, in that order, on engineering
27044. These numbers are original JawCenteredClosure normal-load setpoints, not
measured force commands or motor caps. The original closure feedback and finite
nonlinear master-position mapping are reused. No stiffness, damping, friction,
30 N aperture-Jacobian authority, safety or object physics changes.

Each condition uses a fresh normal-closed scene and identical fixed geometry,
station, approach and pulling profiles. Freeze the first failed engineering
27044 grasp's saved pregrasp RGB-D refinement T_after as the controlled pose;
source run 27044_measured_feedback_initfix01, rgbd/pregrasp/visual_correction.json.
This is explicitly a matched engineering fixture, not an online saved-grasp
fallback or a fresh Agent perception success. The pose was observation-generated,
not GT. Freeze its source hashes. Recompute native current-state IK/route, execute
through actuators, and capture fresh pregrasp wrist RGB-D. Suppress per-condition
visual refinement ONLY inside this fixture, because it would change the pose.
Original closure centering remains live; its actual displacement is a measured
preload response, not a manually changed initial pose.

Same C=0.3 N, b=3 N s/m known engineering plant, native colliders/materials and
0.5 s initialization. Request 12 mm opening FIT, 4 mm reverse FIT, 10 mm
variable-speed FIT, then complete 6 mm reverse VALIDATION. Speeds .003/.0015/
.003/.0021 m/s, acceleration .008 m/s2, sine period3.2s low fraction.35.
Original adaptive startup/transmission/force/collision checks cancel invalid
successors. No action is relabeled success; incomplete VALIDATION excluded.
These are finite comparisons, not another full Agent research episode.

Agent inspects actual RGB-D, native pad traction and F/T/motion results before
approving the next preload condition. Stop a condition on original hard stop;
do not increase preload further through an unsafe candidate. Matched conditions
are separate fresh scenes, not claimed as same-scene recovery. After the matched
comparison, use its measured outcome to select a legal preload/contact strategy
for fresh Agent-driven tests on 27044 and45290. In those tests, recover at verified
unloaded boundaries in the same scene: fresh wrist observations, existing
candidate families and full native executable-candidate fallback. No saved grasp
or GT fallback. Do not make 100um tracking a prerequisite for this hypothesis.

Record native240Hz normal/tangential per-pad force, signed rail traction,
contact points/normals, master/mimic q/dq, actual/command jaw aperture, finite
master caps, actual/command TCP, drawer state, raw wrist F/T and compensation
body states. Explicit drive zero effort/projected reactions are not motor output.
Sparse checkpoint RGB-D, no continuous video. Report static hold separately from
sustained pulling/reversal and complete independent ID. Preserve all failures.

Each matched run: wall600s (180s exit reserve), simulation240s (60s exit reserve).
No stale actor/source deployment; owned actors sequential, shared VRAM allowed.
Only promote/resume benchmark after BOTH drawers demonstrate actual sustained
pull, meaningful reverse, safe exit and complete independent FIT/VALIDATION.

## Conditional millimetre-scale arm-reference comparison

The completed0.75N contact record has a remaining recorded clock lead of3.183mm
while actual drawer motion has stopped. A one-QP offline diagnostic using its
measured joints, initial command bias and reconstructed Cartesian clock goal
requests only~0.000285mm further command FK with the measured-seed prototype,
versus~0.332mm with the unchanged frozen nominal-seed path. This is a diagnostic,
not native proof, and not based on the unloaded100um test.

If1.0N likewise increases load/traction but fails sustained target completion,
run ONE additional matched1.0N condition with
`prismatic_measured_ik_feedback=false`. Every other fixture/job/control/physics
setting remains identical. This reuses the original frozen arm-reference code;
no integral, new drive gains, new authority, timeout or reference-lead increase.
Do not switch the backend under load. Original motion/collision/force checks
remain authoritative. Compare actual command/measurement deficits, relative
motion and drawer travel before attributing the remaining failure to tracking.
This engineering A/B cannot substitute for subsequent fresh Agent tests.

## Matched result and fresh Agent validation freeze

All four original segments completed at1.0N: +12.098/-8.133/+6.096/-4.121mm,
with safe parking. The conditional original-reference comparison is therefore
NOT triggered; no fourth matched run and no arm-controller change. The same
measured-seed backend remains active. The successful same-pose preload contrast
is separate from fresh visual/Agent transfer.

Use one fresh Agent scene per27044/45290, same shared default request1.0N,
unchanged frozen robust ranking (bidirectional experimental re-ranking OFF),
original closure, one-second hold and the same arm/safety/physics. No fixture,
old grasp or successful camera-view input. Codex inspects actual fresh wrist
RGB/SAM/depth/cloud, chooses views and plans, then finite trajectories. Existing
four view adjustments/four acquisitions are maximum resources, not required
retries. After failure: original safe exit, fresh feedback/image inspection,
new automatic candidates/current-state checks and Agent-chosen legal preload.
Never clear a hard latch or wait for Agent while unstable/loaded.

Bound each scene to1800s wall (180s exit reserve),900s simulation (90s reserve),
40 Agent operations and original360s unloaded wait. Prefer the proven12–16mm
initial pull, useful reverse, overlapping variable-speed FIT and distinct reverse
VALIDATION before any larger exploration. Declare exact targets before execution
from fresh geometry/current reachability. Do not count a complete held-out action
as full-duration forward prediction: report the estimator's retained interior
explicitly and retain uncertainty/parameter biases.
