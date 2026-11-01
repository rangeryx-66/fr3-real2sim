# Controller replacement audit

Parent development:22e6524; last executed legacy owner:23c73e6. The old app
checkout and both of these checkouts are preserved. Only a new engineering
controller selection uses the replacement. No old physical record is changed.

| Condition | Decision and new coverage |
|---|---|
| Native xArm/G1/XY effort, speed, drive and joint authority | KEEP, unchanged. XY +/-1m, .08m/s; original effort/gains. Request preload is not measured finger load. |
| Measured collision, invalid state/F/T/clock, joint margin, hard velocity, emergency wrench | KEEP, original native independent supervisor and latch. No reset interface added. |
| Original measured joint margin .05rad | KEEP including command and sampled stop. Not reinterpreted as a manufacturer-certified safety rating. |
| .012m/s loaded design speed and .03m/s measured TCP envelope | ADJUST AUTHORITY, not values. Desired speed remains .012 or lower action request. Exact-FK design excess is logged and continuously regulated, cannot veto an action/brake. Unchanged .03 measured hard supervision also guards candidate/brake speed. No 90%-of-hard extra interruption. |
| Loaded .04m/s² design acceleration and action-selected lower acceleration | MERGE in one owner. Task-selected acceleration changes coordinated command and brake timing. Nonlinear exact-FK design deviations are recorded, not a hard numerical veto. No new strict TCP design-acceleration guarantee. |
| .75 pose error per solve (.75/dt velocity) | REPLACE with time-based gain6/s at60/240Hz, bounded task demand, damping .03. This is an explicit control design change, not a raised safety threshold. |
| SciPy bounded nine-variable LS + nonlinear final governor/SLSQP | REMOVE from new execution. One weighted 6x6 DLS and one transactional boundary/geometry owner. |
| New offline120-iteration DLS feasibility solver | REMOVE; reuse established offline coupled geometric continuation. It is a planner only, never the online command or stopping owner. Early pilots changed available grasp families; final matched comparison must distinguish this. |
| Pink online velocity_step substitution via scoped patch | REMOVE from new execution. Offline grasp/approach Pink remains as an existing geometric planner. |
| Native pre-timed joint paths retimed again at final owner | REMOVE second filtering. Validate each exact discrete joint-position sample and its brake through the same owner; reject rather than silently lag/drop its endpoint. DLS velocity demand retains one rate stage. |
| CartesianReference, nominal seed bias, integrated measured QP, final governor all modifying trajectory | MERGE: measured feedback + command integration in one owner. Progress clock remains the task generator. Bounded contact centering is explicit before IK. |
| Stopping-distance command-deficit anti-windup | DELETE. Contact02 showed ordinary ~0.65mm gravity/closure offset freezing low-load CENTER compensation. Actual task progress/closure references, finite actuator/joint authority, F/T regulation and hard supervision remain. Tracking deficit is telemetry, not an extra veto. |
| Duplicate fixed-station candidate collision checks | MERGE at original30Hz for acquisition/prechecked unloaded paths. Loaded/base-active commands and actual braking remain every240Hz step; native actual contact240Hz and measured geometry30Hz unchanged. |
| Ordinary load warning, provisional sustained band, predicted load | KEEP continuous demand adaptation through the existing classified force monitor. Emergency precedence unchanged. They cannot alone destroy World. |
| Original .3/.6/1Nm wrist and other load bands | UNCHANGED. Provisional sustained band remains classified regulation; report actual safety semantics, not certified limits. |
| Every8tick iterative governor stopping preview, <=5s, optimizer per step | REPLACE with finite scalar-ray brake built without IK/optimizer; same discrete command path used for actual stop. Cache consecutive FK poses. Geometry samples all9 DOFs at0.5mm XY/.005rad arm resolution plus first/end; actual brake geometry and contact supervision remain every step. Strict actuator and dangerous-speed bounds retained; TCP design ratios are diagnostics. |
| Independent per-joint clipping or instant freeze as braking | NOT USED. All9 velocities share the same decreasing scalar. FK curvature and strict bounds still may reject an unsafe candidate. |
| Command dq=0 treated as completed stop | REPLACE with distinct SETTLING and the existing executed V87 PhysicalMotionDwell SI witness: arm<.01rad/s,TCP/object path<.001m/s,TCP/object rotation<.01rad/s,300ms; add measured XY<=1e-4m/s for300ms. Remove the replacement's duplicate raw object/TCP cutoffs entirely. Object rad/s is never treated as m/s. Projected dq remains separately recorded/hard-supervised. |
| Failed measurement dwell | ADJUST to typed pending zero-command hold; staged exit cannot release or restart from it. All subsequent settle() calls catch the same typed state; no exception may escape to close World. Existing finite C supervision/exit budget still applies. |
| Additional hold(.3) after an already qualified300ms owner stop | DELETE for the new path. Consume the same fresh SI/XY witness immediately with native geometry. Contact04 had qualifying quiet windows but the extra wait repeatedly lost them. Thresholds and required dwell are unchanged. |
| Legacy contact hold implicitly stops, starts Cartesian retention micro-motion, then stops again | REMOVE from the new runtime. Explicit hold repeats the stopped owner's command under native feedback/supervision; contact balancing remains an explicit A/B reference. The failed-acquisition contact04 had no verified grasp, so this additional legacy branch is not claimed as its cause. |
| Acquisition bilateral/FORCE_HOLD confirmation | KEEP as action-level contact quality evidence, not hard danger or permission to destroy World. Failed acquisition remains incomplete and transitions to supervised stopping/re-observation; it is never relabelled transmitting FIT. |
| IK numerical failure/candidate collision | Reject proposal, execute previous accepted brake, return action-level recoverable failure if measurements trusted. No numeric exception is relabelled a measured hard hazard. |
| Unexpected accepted-stop/ledger failure at outer C boundary | Retain finite World in explicit CONTROL_BLOCKED read-only diagnosis, not an invented measured hard latch or process teardown. No motion or success is authorized; inspect/owner finish only until a qualified control owner exists. Ordinary IK/candidate failures still use the prior certificate and recover normally. Actual hard SafetyStop keeps its real latch. |
| Future90deg/180mm complete parking route as acquisition prerequisite | Still removed by existing staged exit. Current action route and actual-state release/retreat/parking remain checked. |
| Past measured attempt/recovery cost as a hard plan/acquisition gate | REMOVE authority for new controller. Preserve forecast values; actual finite work/exit budgets and current native geometry remain authoritative. Old pilot rejected remaining planning from806s forecast vs616s remaining work, before IK. Staged admission keeps native transition/current-chain checks and skips stale cost recomputation. |
| ETA forecast interrupts healthy co-motion after two windows | ADVISORY for new controller; measured stall/low transmission and actual budget exhaustion still end action safely. Keep forecast evidence, do not wake Agent for this ordinary estimate. |
| Planning and recovery subbudgets | Existing finite resource accounting retained; no new recovery-count or per-asset caps added. This is a short controller check, not a4h task success claim. |
| Nominal pacing advisory invalidates whole FIT | REMOVE for the new profile. Actual sample safety/finite/sliding/direction/filter support and complete VALIDATION requirements remain. No estimator compensation/fitting mathematics changed. |
| Grasp-family blacklist after any RuntimeError | ADJUST for new profile. Verified acquisition plus known numerical/candidate/brake or registered engineering interruption does not disprove the grasp; preserve error/incomplete action/FIT status and allow fresh native-checked reconsideration. Real transmission/contact loss, unknown faults and hard latches remain failures. Observed legacy3mm fixture incorrectly marked its successful grasp failed. |
| Historical special stops/latches | Preserve original evidence and owner locks. No incident is resumed or rewritten. |

Remaining planning/constructor/ABC wrappers still use some scoped bindings;
they are not online motion solvers or drive writers. This is a controller-sized
replacement, not a rewrite of perception, grasp ranking or native physics.

Validation totals and physical outcomes are recorded in RESULTS.md. CPU tests,
perfect-state integration and command replay do not prove contact dynamics.

K closed-command replay now admits4/4 sampled brakes after removing design-speed
hard-stop authority; the last old projection fails. Arm float32 reconstruction
is approximate and no collision/physical stop is reproduced. D sampled states
were already stationary: its slip dynamics remain unreplicated.
