# Prismatic transmission repair — engineering protocol

Revolute implementation is frozen at `94dab46` / runtime `2a91dbf`; see
REVOLUTE_FREEZE.json (71 exact source/config hashes). Existing six door outcomes
and all blind configuration commitments remain unchanged. No truth reveal.

Engineering comparisons, not official randomized benchmark:

1. 45746: fresh wrist perception, unchanged V7/native-G1 stack and current native
   asset colliders, finite 8 mm pull / reverse / qualified FIT and complete held-out
   trajectory only if physical transmission supports continuation. Legacy PiPER
   used a different gripper, a primitive handle proxy and a different station;
   it is not a controlled same-physics robot comparison.
2. 27044: fixed automatically obtained grasp, compare archived/current finite
   startup with any evidence-supported measured-progress adaptation, without
   changing object parameters or contact geometry. Retain each attempt and actual
   measured starting state; no reset or object joint drive.
3. 45290: fresh wrist observations and genuinely different feasible candidate
   chosen from existing families after inspecting failed contact. Same-scene
   guarded release/reobserve/regrasp, up to the existing four acquisitions and
   four actual Agent view adjustments per scene. No unchanged failed retry.

No clamp/actuator/collision/friction/safety changes. Native 240 Hz essential
numeric/contact evidence, sparse actual RGB-D; optional event clips only.
Ordinary loading/traction failures return to the Agent at a verified unloaded
boundary. Original hard stops remain authoritative; no loaded clearing.
No blind benchmark resume until both representative drawers demonstrate real
pulling, reversal, safe termination and complete independent dynamics validation.
Insufficient evidence is not PASS. New engineering results never overwrite the
previous benchmark failures.

## Conditional retention regression, declared before its outcomes

The measured-state IK and bidirectional ranking tests have not established
reliable transmission. Archived/native feedback shows pad-load redistribution
despite an unchanged jaw target, including degradation during stationary hold.
If the currently frozen 45290 comparison also fails after its meaningful
same-scene alternatives, test the isolated `prismatic_contact_retention` option
on 27044 and 45290. This is engineering, not the registered benchmark.

The original G1 closure and actuator are unchanged. During the original hold
and rail loading, the option can translate the TCP only perpendicular to the
known rail, toward unloading the more-loaded finger. It reuses the existing
5 micrometre centering step, 2 mm total centering bound (including closure's
already-used displacement), and .8*preload stable band. It takes at most one
step per existing 19-sample force-filter window, not repeated integration of
the same delayed imbalance at 240 Hz. Jaw targets and force authority do not
change. This is a bounded contact-retention motion feature, not a symmetry gate.
The original force/collision/slip/actuator and finite breakaway guards remain on.

Before native execution: sign/frame, zero rail command, global displacement
accounting, handover continuity, one-second live hold and quiet-load noninterference
must pass deterministic tests. Surrogate contact-response tests are not physical
success evidence. Use fresh wrist observations and automatic candidates in each
scene; permit Agent-selected safe alternatives within the existing four-acquisition,
four-view, 3600 s wall / 300 s exit-reserve budget. Stop if correction only moves
the contact problem or the original protections stop motion. Never claim repair
or restart the benchmark without actual pulling, reversal and independent validation.

### No-contact delivery check before another contact episode

The first robot-only test used an unloaded, current-collision-checked pregrasp
configuration from 45290's fresh automatic (unexecuted) third plan. No grasp or
object motion was requested. Native API readback confirmed arm static/dynamic
friction efforts of 1 N m per joint. The measured-seed, fixed-bias prototype
commanded ~75 micrometres for a 100 micrometre closing-axis request but delivered
less than 0.2 micrometres; tangential reversal also failed to follow.

Before testing retention, use `prismatic_integrated_ik_feedback` to integrate
measured-state IK velocity into the existing command instead of replacing it
with a new fixed-bias position target. Scale Pink's per-solve pose correction
to the existing 0.5 s measured-progress window (2/s outer rate), without changing
Pink costs/gain, robot/gripper gains, friction, effort or speed protections.
Conditional integration stops growing a saturated incremental command/TCP
tracking bias at the existing 1.85 mm stopping-distance allowance; original
acceleration bounds still decelerate it continuously. This is not an extra abort
or an extended loading timeout. Record physical Cartesian reference, nominal
command FK and actual implicit drive target separately.

The approximate coupled implicit-drive linearization is a preflight, not proof
of nonlinear contact stability. Repeat only the short robot-only tracking check
before proceeding. Include a declared tracking dwell of 4.7 s, derived from the
existing finite loading allowance/rate plus its two measured settling windows;
report that extra dwell explicitly rather than claiming a time-matched result.
If useful small-motion tracking remains unavailable, do not spend another full
observation/grasp episode on the contact-retention option.
