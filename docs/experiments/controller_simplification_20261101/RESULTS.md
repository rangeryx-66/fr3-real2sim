# Controller simplification: implementation and bounded native verification

The code replacement is implemented, and the final unloaded native checks pass.
**Loaded short-chain validation is NOT PASS:** the final bounded contact trial
has not entered B before its work budget; its exit result is recorded below.
The old matched reference reached12.1476mm, while no new contact trial has yet
completed a B segment. There is no demonstrated task-performance improvement.
This is an engineering replacement and short-stroke comparison, not a completed
180mm experiment, hardware certification, or Physics-Aware benefit. All negative
iterations are retained separately.

## Source and scope

- App checkout at intake: `2bdae2ad1318c2ed466ca1096f021cda162ce3fe` (unrelated dirty work preserved).
- Development parent: `22e652417f5afc9683fdfcca15fc0690d58e34dc`.
- Old executed reference: `23c73e6e7b05893162f11865ea036a9f86935839`.
- Post-native budget-routing fix (unit tested; no further native run): `3d7b8479d0420d07f9dc864ce2bf76c88170b12f`.
- Final native verification source: `1138b892d416cbae2b6319f12540081d33f17f86`.
- Branch: `codex/simple-coordinated-controller`, repository `rangeryx-66/fr3-real2sim`.
- HiddenKinematics audited source: `d0318abec626fc8416f194619f21fbe7145e2ebf`;
  MIT attribution retained in `third_party_licenses/HiddenKinematics.txt`.

Native execution was isolated in `/tmp/agentic_simple_controller_20261101`; the
final owned worktree is retained at `/data1/home/rangeryx/agentic_controller_v89_20261102`. The original
app checkout and its unrelated dirty files were preserved; both old reference
checkouts and all previous safety holds were preserved. No frozen live actor
was edited. New code is opt-in; old execution remains available in its separate
checkout. Source freezing is now enforced before instance reservation and
immediately before actor creation, including staged/untracked executable code.

The executed flow is task progress -> bounded world TCP reference -> weighted
XY+7-arm DLS -> transactional CommandOwner -> one synchronous native drive
submission -> independent measured supervision. DLS uses a time-based 6/s
feedback gain, explicit seconds, world-frame rotation log and xyzw pose boundary;
it does not copy a 60 Hz position increment into a 240 Hz velocity loop.

The new path removes online bounded nonlinear IK, the old command governor,
iterative stopping projection and duplicate Cartesian reference filter together.
The established offline geometric planner remains for grasp/path feasibility;
it is not a command owner. Native C2 position paths pass their exact timed
samples through the same final owner without a second retiming filter.

A rejected proposal changes no committed state. A finite all-nine-coordinate
scalar brake preserves base/arm cancellation; it uses the identical discrete
integration for preview and execution and never invokes a failed motion solve
again. Strict joint/actuator authority, candidate geometry/dangerous-speed
guards and original independent measured supervision remain. Braking geometry
is sampled, not a continuous collision/contact proof. Exact TCP design-rate
ratios are telemetry and continuous demand regulation, not new hard hazards.

Command zero and measured rest are separate. The executed existing SI
PhysicalMotionDwell is reused, with additional actual XY rest confirmation.
Typed pending rest returns to protective C; all subsequent exit dwells handle
it. Unexpected accepted-stop/ledger uncertainty retains a finite diagnostic
World as CONTROL_BLOCKED without inventing a measured hard latch. That state
does not authorize automatic movement or claim rest. Native hard stops keep
their original latches. Full decisions/thresholds appear in `LIMITS.md`.

Fresh vision/SAM/grasp planning, ABC, native G1/XY/xArm contact and object
dynamics, F/T compensation, Physics-ID fitting and FIT/VALIDATION separation
remain. Physics-Aware preload/speed/acceleration reach the executed action
interfaces and are covered by tests. This is interface preservation, not proof
that an updated model improves task performance. No hidden dynamics were read
for online decisions; known articulation geometry remains an allowed prior.


## Actual module assembly

`agentic_interaction_v81.entry` chooses the native RuntimeV7/V81 scene and XY
interface plus V89 replacement, and explicitly bypasses V87 governance and V88
online coordination for this profile. `interaction_runtime.runtime.step` calls
`submit_drive` once before the native step. V89 forwards one XY+arm+G1 target
to `NativeDrive.apply_coordinated`; the separate legacy XY hook is passive.

| Audited module | Role in the new execution |
|---|---|
| `agentic_interaction_v88/kinematics.py` | Existing offline grasp/path feasibility only; no online owner |
| `agentic_interaction_v88/runtime.py` | Bypassed online |
| `agentic_interaction_v87/command_governor.py` | Bypassed online; preserved for old checkout/replay |
| `agentic_interaction_v88/stopping.py` | Iterative projection bypassed; common-ray owner brake replaces it |
| `scripts/xarm7_native_gripper.py` | Same finite native drives/gravity mapping; single synchronized submission |
| `agentic_interaction_v87/staged_exit.py` | Current-state staged release, retreat and alternative parking; consumes the new measured stop |

The committed9D reference is a float64 nominal kinematic target. NativeDrive
retains the original arm gravity-bias `reference + g/kp`; that raw implicit
drive target and bias remain separately reported in native drive evidence.
Neither the nominal reference nor requested jaw preload is a force measurement.
Compatibility `qtarget`/phase carriers and existing scene/planning wrappers
remain; they are not additional online proposal owners or native writers.

## Non-physics checks

144 focused tests passed in19.961 s after the final budget-routing fix. The
last native source had140 passing tests; the additional4 tests cover the
observed typed-budget routing error and hard-latch/error precedence. These cover quaternion/frame/dt semantics,
singularity/joint margins/reversal/lag, rejected/stale transactions, cancellation
braking, command-zero versus measured SI/XY rest, paused World handling without
latch bypass, exact timed position paths, exit pending/outer diagnostic
lifecycle, grasp memory, real versus forecast resource authority, Physics-Aware
command inputs and tracked/staged/untracked-code provenance.

Actual robot URDF world Jacobian finite differences: maximum absolute error
`8.027167819335546e-11`. Perfect-state 240-step integration completed; a native
C2 92-sample path retained its endpoint with zero numerical position error.
Final-source median solve+owner 6.708 ms / p95 12.170 ms in that CPU check; median stop
certificate 6.066 ms. These include certification but no native physics/contact
and do not establish a 240 Hz real-time rate.

Closed K replay accepted 4/4 sampled new command brakes where the last old
projection failed. Historical arm values require approximate float32
reconstruction: this is not actual collision checking, measured stopping or
same-World incident recovery. D sampled stop states were already zero; D's
material-slip dynamics were not reproduced. A separate real old-controller
reverse segment below reproduced TCP_PROJECTION_INFEASIBLE.

## Native runs and negative iterations

All trials are independent and are never concatenated into one successful
episode. The engineering pilots use a 3,600 s wall / 1,200 s simulation ceiling
with existing exit reserves; they are not the previously requested 4 h Goal
benchmark. A controlled 3 mm interruption is a registered recoverable fixture,
not a collision or claimed physical slip. Object state is preserved during
recovery inside each pilot. Explicit owner finalization of these bounded
engineering pilots is recorded as incomplete, never as physical impossibility.

| Trial | Source identity | Actual result | Terminal evidence |
|---|---|---|---|
| free01 | local b904dc5 | Float32 nominal integration caused a proposal while braking. Fixed float64 committed integration. | EXIT_UNVERIFIED, closed |
| free02 | local d67c628 | Command stopped; projected native dq disagreed with stationary configurations. Added actual-configuration witness, retained raw dq hard supervision. | EXIT_UNVERIFIED, closed |
| free03 | local e0cd6b3 | Move/stop/restart completed; parking design-bound rejection. | EXIT_UNVERIFIED, closed |
| free04 | local f97048b | Parking candidate 0.080001358 m/s versus 0.08 design speed rejected (1.36 micrometres/s excess). Fixed authority; physical limits unchanged. | EXIT_UNVERIFIED, closed |
| free05 | local 0596a0f / public 61ecb68 | Two free movements, measured stop/restart, release, retreat and parking completed. | SAFE_PARK_VERIFIED, closed |
| contact01 | local 0596a0f | Paused World stop sampling triggered clock mismatch before grasp. Corrected resume after latch check. | HARD_STOP_UNPARKED, latch retained, closed |
| contact02 | local 007ac02 / public 89669a4 | Fresh grasp failed; low-load CENTER compensation frozen by unsupported tracking-deficit veto. Veto removed. Same-World observation/recovery boundary preserved. | SAFE_PARK_VERIFIED, closed |
| contact03 | local c18ab0a / public 7796ae5 | Two acquisitions/one regrasp, never FORCE_HOLD; 0.50246 mm preload/stop creep, no B/FIT/VALIDATION. Later typed pending escaped outer C. Exit lifecycle/witness corrected. | EXIT_UNVERIFIED, closed |
| old_matched01 | 23c73e6 | Acquired, 3 mm fixture, recovered in same World, forward reached 12.1476 mm; reverse projection failed after 0.22335 mm. Later recovery planning rejected by forecast cost before IK. | SAFE_PARK_VERIFIED, closed; full short chain NOT PASS |
| free06 | HEAD 891ada1 / staged tree identical public 8597fd4 | Agent launch sequencing error allowed launch after metadata alignment failed. Immediate export requested. Excluded from frozen validation; added enforced clean-source launch admission. | UNVERIFIED_MISSING_EVIDENCE, closed |
| free07 | 40870bee | Free move/stop/restart succeeded; open-loop cancellation failed: 1.001 mm XY, 1.090 mm TCP drift. Kept as negative; no threshold change. | SAFE_PARK_VERIFIED, closed; cancellation NOT PASS |
| free08 | fd17163f | Same cancellation input plus normal measured-TCP DLS arm feedback: 1.001 mm XY, 0.519 mm TCP peak drift; move/stop/restart and measured stopping passed. | SAFE_PARK_VERIFIED, closed |
| contact04 | fd17163f | Failed bilateral acquisition; no B, fixture or FIT/VALIDATION. Qualified stop windows were lost to duplicate downstream hold; same World retained in protective C. Existing release and finish could not complete; explicit engineering export. | EXIT_UNVERIFIED, closed |
| free09 | 1ea27a3 | Native unloaded constructor admission called hold before the replacement owner existed. Failed before any physics step; corrected initialization-only dispatch, added regression. | EXIT_UNVERIFIED, closed |
| free10 | ba997153 | Move/stop/restart, coordinated cancellation, release/retreat/park passed;146.250 s wall /12.9625 s simulation. | SAFE_PARK_VERIFIED, closed |
| contact05 | ba997153 | First acquisition failed, exited/parked and re-observed in same World; new oblique view and disjoint second IK batch found deeper grasp. Second acquisition GRASP_VERIFIED, then restart from SETTLING failed before B with MOTION_FORBIDDEN_IN_STOP_STATE. Both action exits reached parking, but final control-fault lifecycle conservatively unverified. | EXIT_UNVERIFIED, closed; no B/FIT/VALIDATION |
| free11 | 1138b892 | Readiness-loss supervision-input injection rebuilt real dwell and restarted through actual runtime;148 physics steps for requalification/re-stop. Original moves, cancellation and final park passed. | SAFE_PARK_VERIFIED, closed |
| contact06 | 1138b892 | First acquisition failed bilateral hold and exited safely. Same-World oblique view and unflipped replan succeeded; second acquisition reached work deadline during jaw closure. No B. Both action exits reached measured parking, then typed-budget normalization incorrectly entered control diagnosis. | EXIT_UNVERIFIED, closed; NOT PASS |

Early new pilots temporarily used a different offline feasibility solver; the
final source restores the legacy offline planner. Initial state/plant/action
declarations are matched, but different actual view/grasp choices, manual Agent
wait and recovery paths confound a simple wall-time speedup claim. No causal
performance advantage follows from these small engineering samples.

Old matched pilot: 3,055.037 s wall / 311.779 s simulation, actual maximum
12.147595 mm; reverse segment 12.147155 -> 11.923803 mm (0.223352 mm), not the
requested approximately 8 mm meaningful reverse. After release the drawer was
11.831799 mm. The next plan was denied by an 806.288 s empirical forecast with
616.087 s actual work budget remaining, before native IK. New code retains that
forecast as evidence but removes its veto; exact work/exit budgets and native
transition/current-chain checks remain binding. Actual stall still ends an
action; an ETA warning during healthy measured co-motion is advisory.

Free05 (earlier source): final TCP tracking errors 0.07155 / 0.01746 mm,
252.143 s wall / 17.092 s simulation. Fixed geometry/certification costs remain
substantial. Stop-preview timing is inclusive of its own FK/geometry work;
exclusive collision and command timers have different scopes and cannot be
added. New final run timings must be read separately below.

Final free08: 149.001 s process wall / 14.1625 s native simulation. The bounded
check itself took 116.816 s, excluding setup/export. Actual TCP cancellation is
partial, not zero (0.519 mm peak for 1.001 mm XY); all stopping witnesses and
parking completed. For 2,572 unique changed RUNNING samples the recorded solver total was0.404 s,
stop-certificate total27.681 s / p95 30.260 ms. Exclusive collision work11.510 s
and command/stop work2.539 s have overlapping scope with certificate evidence;
do not add them. Native profile has50.148 s unaccounted wall time.
Command TCP excursion reached 0.964 mm as feedback overcame
native drive lag. These distinct quantities are retained, not relabelled as
perfect cancellation. The unchanged pass cell is <1 mm excursion for this
small unloaded engineering input; it does not qualify loaded tracking.

Contact04:1,576.542 s wall /272.642 s simulation; maximum rail2.944 micrometres
(0.002944 mm), zero verified grasps /one attempted acquisition, no B segment.
Protocol wait440.896 s includes repeated supervised C. It retained World and
real physics rather than throwing out of C, but that is not successful recovery.
Live sealed150–171.53 s support contained a0.5625 s combined quiet window and
owner-measured-stationary samples, despite repeated exit-pending responses.
Removing the redundant post-witness300ms hold is a code fix, not proof that
this closed instance subsequently exited. Its unverified state is preserved.
The explicit replacement hold also removes implicit legacy retention restarts
for verified loaded contact; failed acquisition did not execute that branch.

Contact05:3,306.551 s wall /305.8875 s simulation,2 acquisition attempts /1
verified grasp /1 regrasp. The maximum0.062948 mm was preload disturbance, not B
operation. Protocol wait646.470 s includes Agent engineering work. First IK
shortlist failure did not terminate World; the disjoint second batch found
family[0,+9.375mm,0,0] from the new current observation. GRASP_VERIFIED at283.7375 s
with filtered resultant pad loads1.204/.848 N (not pure pad normals). No B
segment started, so the registered3mm interruption did not fire and no FIT or
VALIDATION ran. Action exits confirmed parking at165.4125 and305.8875 s, but
CONTROL_STATE_UNTRUSTED at the final software fault did not provide a new
post-fault controller acknowledgement: the final safety record remains
EXIT_UNVERIFIED. These two scopes are not conflated.

Shared restart fix: an explicit request from SETTLING must first wait through
the unchanged measured-stop protocol; a SETTLING proposal is rejected without
committed-state mutation. Running motion is unaffected by a stationary witness
being false. Free11 conservatively invalidates only readiness input, resets no
physical state and clears no latch. It rebuilds REAL native dwell and accepts a
new explicit request, then stops again. This proves the unloaded restart path,
not recovery from a physical disturbance under load. New staged snapshots also
label the authoritative9D command separately from legacy qtarget XY/G1 slots.


Contact06:3,509.853 s wall /258.133 s simulation,2 acquisition attempts /0
verified grasps /1 regrasp. Protocol Agent wait1,088.823 s includes engineering
analysis;8 high-level commands are not8 LLM calls. Maximum measured rail
0.0225464mm was acquisition/preload disturbance; final rail0.000000232mm.
**No B segment, registered3mm interruption, FIT or VALIDATION ran.** The first
failed acquisition completed release/retreat/park. The same World was then
re-observed from an actual-map-derived oblique view, and a different current
automatic orientation passed native planning. The work deadline occurred
during the second jaw closure, before contact (its recorded pad normals were
zero), rather than a mechanical travel limit.

The second protected exit also recorded release, retreat and measured parking,
but `complete()` wrapped the separately typed `operation_status=BUDGET_EXHAUSTED`
into an unknown RuntimeError. The diagnostic lifecycle consequently discarded
that exit for terminal classification. Its immutable terminal remains
EXIT_UNVERIFIED, no native hard latch, actor closed; it is not rewritten as a
success. This exposes a remaining software classification bug in the executed
source, not evidence of dangerous load or failed mechanical parking.

The follow-up3d7b847 fixes this shared routing after closure: typed operation
budgets remain ordinary incomplete operations. Only trusted current state,
explicit command-zero/actual-rest and complete native exit evidence permit
reuse of that exit. Missing evidence still invokes the original guarded
recovery; unknown errors and native latches cannot use this path. Work-budget
end remains an honest task timeout, while a subbudget with overall resources
left returns to Agent. Four regressions and the full144-test suite pass.
**This final routing fix has not had a new native run.**

For contact06,57,915 unique changed RUNNING command samples recorded DLS
solve14.123 s total versus stop certification765.772 s /p95 40.204ms. This
measurement confirms the remaining bottleneck; a small DLS solve alone does
not make the complete control chain inexpensive. It cannot be compared as a
causal wall-time improvement against the different old grasp/recovery trace.

## Metrics, semantics and evidence

Closed-only analysis preserves commanded, measured and proposed state scopes.
Requested preload is distinct from measured per-pad normal forces, resultant
pad loads and six-axis wrist wrench. Curves labelled compensated wrist apply
the original bias and tool-gravity correction at wrist origin; dynamic tool
inertia remains in those diagnostic curves. The original Physics-ID estimator
retains its full compensation, fitting and independent-validation workflow.

TCP/handle relative translation is reported as drift; it is not automatically
material contact slip. Contact diagnosis uses the original physical records.
Incomplete fixture segments, missing FORCE_HOLD and incomplete VALIDATION are
not fitted or promoted into identification success. Unsupported mass/friction
parameters and GT errors remain unknown where the actual data cannot support
them. No A/B/C updated-model efficacy study was run in this controller check.

Agent wait is protocol wait wall time and includes engineering analysis; command
counts are not LLM-call counts. Phase aggregates may span repeated phase names
and gaps. Motion acceleration/jerk are rail-DOF, original segment-local filter
and edge exclusions; no smoothness advantage is claimed at unmatched speeds.
Generic telemetry stop-marker counts are not classified native hard-stop counts.
Terminal safety records determine whether measured stopping/parking completed.

Raw continuous video, telemetry, actual source manifests, current-view/plan
boards and terminal states are retained under
`/data1/home/rangeryx/agentic_v86_private/controller_simplification_20261101`.
Analysis packets include source SHA, profile scopes, six-dimensional wrist and
motion/load curves, and per-file SHA256 seals. A return code 0 is not success.
Every completed/abandoned owned actor has exited; owned simulation resources
are released. Historical unverified outcomes and safety holds remain unchanged.

## Remaining limitations

The contact short-chain has not passed. The shared restart and stop-witness
fixes have meaningful unit/unloaded evidence, but no successful final loaded
B segment. There is no basis to assert reliable long-stroke operation, reduced
regrasp rate or improved grasping.
The sampled finite brake is bounded and transparent but still costly; measured
scene changes can block its certificate. CONTROL_BLOCKED preserves diagnosis,
not a qualified new controller or permission to restart. Exact nonlinear TCP
design-acceleration bounds and continuous swept-volume/contact guarantees are
not claimed. Long-run controller validation, material-slip replay, matched
repeat statistics and formal Physics-Aware causal comparisons remain future
work after these engineering results, rather than inferred passes.

## Reproduce and inspect the evidence

`NUMERIC_RESULTS.json` contains per-source measured quantities and timer scopes;
`checks/` contains the144-test packet, final-source robot-Jacobian check and
K/D command replays. `curves/` publishes selected motion/load and six-axis
wrist plots; pad normal, resultant load, compensated wrist and material slip
remain distinct. All earlier failed iterations remain in the raw private
archive and table, not aggregated as repeats of one final controller.

```bash
PYTHONPATH=.:scripts OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  "$ISAAC_PYTHON" -m agentic_interaction_v89.analyze_closed \
  --source "$PWD" --episode /absolute/closed/episode --output /absolute/analysis
```

The analyzer refuses a live or non-prismatic episode. Its no-B plot uses an
explicitly labelled projected-arm-velocity trace; diagnostic wrist curves use
bias/gravity compensation, not a full dynamic-inertia estimate. Per-file raw
SHA256 manifests are stored with private analysis. `SOURCE_DIFF.patch` in the
private delivery archive compares the development parent against the final
branch. The original app checkout remains at2bdae2a with its unrelated work;
old reference worktrees remain clean at23c73e6 and22e6524.

## Launch and rollback

See `agentic_interaction_v89/README.md`. Use a clean checkout of the native
verified SHA, the explicit private resource/environment, and the existing V81
engineering spec with `controller: coordinated_dls_v1`. Free-space-only smoke
adds `controller_validation: free_stop_restart`; normal ABC omits it and still
requires actual current image/plan inspection and explicit finite execution.
Never edit a live actor or use a previous successful pregrasp as current vision.

After exporting/closing the new actor, old execution is available in a separate
checkout of `23c73e6e7b05893162f11865ea036a9f86935839` with the controller field
omitted. Do not hot-load the old version or clear any historical latch.
