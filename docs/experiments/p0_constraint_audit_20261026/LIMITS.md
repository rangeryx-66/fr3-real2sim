# P0 execution-limit authority audit (2026-10-26)

Status: v2 engineering candidate; v1 physical qualification NOT PASS (see [PHYSICAL_01.md](PHYSICAL_01.md)); complete 90-degree / 180-mm physical acceptance PENDING. No inherited incident is reset. The previous R02 World is closed and cannot be recovered. This audit and any subsequent qualification are distinct from that episode.

## Scope and trace

Baseline for this change: `d65f0d8cfd557cd3c7ecb3c3898d98492f609003`. The actual R02 loaded-source ledger is bound by SHA in [CONDITIONS.json](CONDITIONS.json). Its complete source list is expanded with current V81/V84/V87 and runtime modules to include adapter paths absent from the older actor. Each condition, explicit exception, catch owner, loop break and status-bearing return is indexed with file, function, line and source hash. This is an overinclusive static inventory, **not a claim of dynamic branch coverage**. Constructor-only and inactive legacy branches are included. Native PhysX/driver internals are outside Python source coverage.

Execution ownership:

1. `agentic_interaction_v81.launch`: strict registration, resource/incident integrity, canonical actor mutex, new instance identity, finite wall owner. It spawns `entry`; no direct-entry bypass.
2. `entry` wraps frozen revolute RuntimeV7 / rail PrismaticRuntimeV751 with MobileRuntime and V87 GovernedRuntime, once at construction. Native closure, implicit drive caps, body physics and estimator remain frozen.
3. `agentic_interaction_v84.session/operations`: A fresh wrist/whole-chain planning and approval; B finite predeclared action chain; C actual result, exact contact generation and optional continuation. No per-step Agent call.
4. `interaction_runtime.operations.rotate_segment`: original bounded reference lead, progress/transmission/startup, finite resources; calls V87 Cartesian reference -> Pink IK -> final-command FK governor -> command collision check -> native step.
5. Native step applies the bounded command, measures 240 Hz state/F/T/contact, checks actual collision/joints/speed/force, records telemetry. Guarded braking uses last accepted phase before a lower release ceiling.
6. Recoverable action outcomes -> existing recovery_supervisor -> stop / release geometry / unload confirmation / local clearance / fresh wrist / measured native park or verified released hold. Original object state, goal and records remain.
7. Hard or unconfirmed control state -> finite read-only `lifecycle.retain`; no automatic reset/actuation; owner finish/deadline -> export/finalize/actor close. Session owns World lifetime, not a task warning.

## Authority before and after

H = hard stop, latch/diagnostic owner; R = stop only the action and supervised recovery; A = continuous adjustment/recording; P = reject an unexecuted plan; E = finite resource/owner finalization. An empirical value is not relabeled as a manufacturer safety rating.

| Limit / owner | Basis, duplication and former consequence | Current authority / action |
|---|---|---|
| Actual forbidden contacts; arm/hand/support/scene collision (`policy`, `collision`) | Physical geometry/contact ownership; static 30 Hz plus native 240 Hz, complementary | H retained. Never disabled to reach a target. |
| Command path collision (`V87.cartesian_step`, frozen command guard) | Prediction before command; used to propagate SafetyStop as if measured collision | P/R. Reject command; only trusted current control may brake/unload. Actual collision remains H. |
| URDF joint range and finite continuous-joint workspace | Official geometry plus declared workspace | H and planning rejection retained. |
| 0.05-rad joint margin | Conservative engineering reserve, not a hard mechanical stop; repeated in planner/final command/measured checks | P on predicted violation; measured violation remains fail-closed H pending separately proven stopping-domain replacement. No closer joint operation is authorized here. |
| URDF velocity/effort and native implicit drive caps | Official source caps; one-third working-speed margin is engineering, not a rating | Retained unchanged in command governor and measured guard. A before H; no additional actuator authority. |
| Phase TCP command speed/angular speed/acceleration; joint acceleration | Provisional engineering envelopes; virtual reference alone did not bound final IK command | A with original nonlinear final-command FK governor. Independent original-limit verification; optimizer headroom/success flag is not a new gate. |
| Measured TCP speed (loaded 0.03 m/s; phase-specific otherwise) | Conservative engineering operating envelope, not manufacturer certification. 45134 exposed actual final-command overspeed | H retained; continuous reference regulation starts at declared command speed before reaching this boundary. |
| 90% of measured TCP hard bound (`V87.cartesian_step`) | Additional heuristic action interruption, duplicates governor/real speed guard | Removed for explicit classified policy; A using measured speed feedback, original hard bound unchanged. Legacy profile unchanged. |
| Old 1.5x command-speed feedback heuristic | Additional feedback rule overlapping new regulator | Replaced by shared measured-envelope regulation for classified policy; no competing ad hoc multiplier. |
| Force warning: pad2 N / wrist4 N / torque0.3 N m | Configuration explicitly says informational / provisional engineering values; raw warnings never needed latch | A. Original trailing history continuously reduces desired reference speed between existing warning and sustained bands. Original bounded deceleration/final-command checks remain. No instant speed snap. |
| Sustained 0.15s: pad4 N / wrist8 N / torque0.6 N m | `configs/interaction_runtime_v1.json.provenance.force`: provisional short-test settings, not manufacturer safety ratings. Previously direct H, including 0.600200 event | A only in explicitly registered loaded action with original trusted-state qualification: desired reference speed brakes continuously to zero at the old band, without an automatic action exception for tiny/noise-scale crossing. Existing emergency priority remains. Actual loss of transmission or two completed resource forecasts returns only the action to the Agent; other phases/untrusted state keep the original conservative guard. No threshold is raised. |
| Emergency pad10 N / wrist10 N / torque1 N m | Original conservative emergency envelope, not newly claimed hardware certification | H retained with original precedence, including during brake/unload. Original history is never reset at mode/direction changes or adapter installation. |
| Invalid contact/wrench/robot/command; invalid physics period | State integrity essential for any controlled recovery | H/uncertain-state diagnosis retained. Never normalized into ordinary warning. |
| Both pad reports absent >0.15s (`runtime.loss_s`) | Normal-only report absence is not independently proven separation; formerly SafetyStop -> lifecycle | R if current original trusted-state checks pass; original stop/unload owner handles it. During an already active guarded brake, record the absence without aborting that brake. Collision/speed/emergency/state guards remain. |
| Single missing pad / finite stable slip | Not evidence of separation by itself | Existing advisory semantics retained. No new symmetry veto. |
| Closure slow speed, centering2mm /5um steps, bilateral preload/0.2s hold, acquisition deadline | Finite native closure domain; failure does not prove actuator hazard | Native closure unchanged. Failure -> existing bounded R. Request force is not measured clamp force. |
| Relative slip change / aperture stability in C | Empirical contact-trust test for retaining load, distinct from hard safety | Ends retained C and invokes original unload; does not destroy World or erase goal. |
| Increasing slip + consumed lead; two low-transmission windows | Empirical physical-transmission diagnosis; nominal command demand can misdiagnose slow co-motion | V2 replaces the absolute0.1mm/s co-motion floor with a float32 endpoint representation budget plus measured TCP/object ratio. Actual slip or unresolvable zero progress R; slow resolved co-motion advisory. Two completed windows unable to finish within registered wall/sim resources produce SAFE_PARTIAL_PROGRESS for Agent strategy change, not a contact failure. FIT eligibility independent. |
| 0.5-degree revolute reference lead | Explicit bounded tracking allowance, not an angle-specific asset rule | Continuous braking-distance reference constraint retained. No hard latch from ordinary lag. |
| Rail stopping-distance lead / integral antiwindup / startup bias | Bounded reference and measured servo-offset domain; stiction needs loading opportunity | Existing causal startup and antiwindup retained; startup failure R, no arbitrary timeout increase. |
| Command projection infeasible / IK unavailable | Cannot submit that proposal safely; numerical optimizer failure alone is not hardware danger | R, command not submitted; current-state safe stop/unload. Nonfinite/failed actual brake not downgraded. |
| Stopping route collision / finite stop not verified | Cannot certify braking/control, independent from target-planner rejection | H/uncertain-state retained. No blind retreat or target jump. |
| Whole-chain IK/closing sweep/measured scene stamp | Geometric feasibility, not a measured safety incident | P; current World and actual object state retained. Stale plan invalidated. Agent selects fresh view/path/candidate/base. |
| Predicted endpoint exit plan | Formerly duplicated current motion admission and prevented otherwise checked long motion | Existing staged opt-in separates current motion/stopping/hold proof from future exit UNKNOWN; no fabricated exit proof or zero exit cost. |
| Release sweep/unloaded criterion0.05N/0.3s stationary dwell | Required domain for changing grasp/base; estimated park is not measured rest | Release permission uses the existing stationary TCP linear/angular cells through known articulation geometry; the prior shared1e-4 scalar cutoff mixed rad/s and m/s. Arm cells/duration, release sweep and collision guards stay. Base-relocation admission retains its own original unloaded/station requirements. Partial exit -> verified released hold/replanning, or motion-forbidden diagnosis if not confirmed. |
| Station/base unloaded guards, speed and collision route | Moving base under contact/unknown pose changes control geometry | Retained. Planning precondition rejects unexecuted route; actual loss of safe boundary fail-closed. No new simultaneous owner. |
| Explicit max_sim / finite episode wall+sim / exit reserves | Resources, not load-safety proof. Default adaptive segment has no implicit45s cap | E/R through original reserved stop/exit. Healthy measured progress can extend nominal horizon within registered resources. ETA alone advisory. |
| Loaded C max20wall/5sim / timeout / contact trust | Finite supervised wait, not task failure | Bounded unload to A. **Fixed:** resource exhaustion or typed controlled fault inside C no longer escapes to actor finalizer. Hard still goes to retained diagnostic owner. |
| Four-view / acquisition / Agent-operation budgets | Experiment resource limits; not physical danger | Existing active-recovery view override retained; finite registered acquisition resources remain explicit. Exhaustion is not task success or permission to reset object. |
| Perception/model worker timeout, missing feedback/JSON | Presentation/numerical availability, not control failure | Existing worker defer / fresh observation / trusted read fallback retained. No model readiness gate on otherwise valid manipulation. Unknown actuator errors remain uncertain. |
| FIT nominal-demand exclusion / SG support / validation queue | Statistical validity, not permission to move | Retained, separate from operation result. Interrupted FIT excluded; partial/diagnosed validation cannot be independent PASS. |
| World owner timeout, owner finish, signal/STOP_AND_EXPORT | Explicit finite ownership/cleanup, not a contact warning | Export then close owned actor; success requires measured park, not rc0. Old locks remain. |
| Unknown exception/unconfirmed unload or stop | No sufficient evidence for another motion | Retain finite diagnostic World, no auto actuation/clearance; fixed the actual dispatcher fall-through for UNRECOVERABLE with no measured latch and for an initially unconfirmed boundary. Final cleanup at owner end. Not described as a measured overload if it is software uncertainty. |

No generic severity-by-substring policy is used to suppress SafetyStop. Only exact, documented action conditions in `constraint_feedback.action_fault` are normalized, with the existing `recovery_supervisor.trusted` qualification. An existing latch cannot pass that qualification. Failed stopping, measured collision, emergency and state-integrity faults never match. Source/state ambiguity remains fail-closed.

## Activation and rollback

Engineering V87 spec must explicitly set `constraint_policy: "classified_continuous_v2"` **and** `staged_exit: true`. Strict launcher validates and strips these adapter fields from frozen V8 compatibility input. Other profiles retain prior protection semantics. No hot loading. Rollback removes this opt-in in a fresh actor; it never changes an existing latch or incident registry.

The user explicitly requested that unsupported experiment-level thresholds lose World/latch authority. That instruction supersedes the old Skill's blanket preservation of every provisional force-stop classification; it does not supersede true collision/emergency protections. Frozen policy/controller/Skill bytes remain unchanged, and the original guard result is retained as a shadow event.

## Evidence boundaries

[R02_LOAD_REPLAY.json](R02_LOAD_REPLAY.json) reconstructs the original gravity-compensated guard wrench from verified raw telemetry/tool poses, without reading plant coefficients. It reproduces the original 409.875s /40.144539-degree /0.600199774 N m tail event. V2 classification continuously brakes the reference; it does **not** predict what contact will do after intervention, certify a safe brake/unload, clear the old latch or prove 90 degrees.

45134 final-command replay uses current real FK, original declared limits and explicit end braking. It must complete all frames and reach zero command velocity; this is not physical stopping evidence. Existing planning/staged-exit/lifecycle fault injection checks World identity, original goal/history and measured-boundary requirements. Synthetic warnings/hazards test the real supervisor and dispatch seams; they are labeled simulation-free and cannot be substituted for contact experiments.

Full physical acceptance is conditional on these checks and current original instance/GPU admission, fresh images and inspected plans. It requires continuous video/raw240Hz evidence, 90-degree door and180mm rail with real measured completion/reverse/independent validation status and verified exit. No earlier episode is spliced into a new PASS.

The directly loaded private closure source was separately inspected (SHA256 `b522bcef78a1191a4a2a5c756c34d11d7df2e22ddaa5f782be61343947a1704c`). Its bilateral contact/preload, bounded centering and0.2s hold transitions match the closure row above. Its source bytes are not copied into this public inventory. Task goal, calibrated six-axis measurement and frozen V83 estimation models are not changed.

[45134_COMMAND_REPLAY.json](45134_COMMAND_REPLAY.json): all4627 recorded final commands complete under current FK governor; max0.011999988m/s, max0.034999975m/s²;186 original-limit braking steps reach zero command velocity. The original measured0.032125952m/s violation still triggers the actual speed guard in fault injection.

Final selected offline suite: **140 passed,39 subtests passed,0 failed**,57.87s. [OFFLINE_RESULTS.json](OFFLINE_RESULTS.json) records the scope and earlier corrected harness failures. These selections overlap earlier suites; counts are not additive. Frozen baseline audit:606 physical /71 revolute /956 release sources, no changes/failures.

## V2 source ownership and physical evidence

V1 is preserved in Git and [R02_LOAD_REPLAY_V1.json](R02_LOAD_REPLAY_V1.json); its140+39 offline result did not establish physical qualification. [PHYSICAL_01.md](PHYSICAL_01.md) records the failed34.54-degree trial and cleanup. V2 explicitly changes policy registration, never silently hot-loads old actor behavior.

Final-command governance now delegates to the inherited revolute/rail reference, contact retention, integration/antiwindup and IK pipeline. Only its final proposal is projected before native submission; rejected proposals restore the last accepted command, velocity and governor history. This replaces the old adapter's duplicated Cartesian controller that shadowed rail-specific feedback. Actual measured supervision remains after physics. Constructor admission records get the complete diagnostic key set before their first retained row; unavailable early values are NaN/false, not fabricated measurements.

No further long physical qualification is launched until revised fault injection, raw failure replay, original-limit FK braking and frozen-source audit pass. Full90-degree /180mm operation remains PENDING until a new version's own continuous video, raw telemetry, recovery and actual exit prove it.

V2 selected regression: **158 passed,47 subtests passed,0 failed**,57.21s. See [OFFLINE_V2_RESULTS.json](OFFLINE_V2_RESULTS.json). The old158/140/etc selections overlap and are not cumulative.

## Actual V2 qualification and shared exit follow-up

[PHYSICAL_02.md](PHYSICAL_02.md) and [PHYSICAL_02.json](PHYSICAL_02.json): measured50.001 degrees,8656 continuous-load-regulation samples,zero safety-stop rows. Actual reverse route genuinely collides and is rejected. World retained across recovery, no reset; final parking unverified, NOT PASS. The old episode is closed/cleaned and cannot be continued or counted toward a later90-degree success.

| Newly exercised limitation | Before | After / unchanged physical basis |
|---|---|---|
| Fixed4/8cm Cartesian clearance candidates | No candidate prevented all onward parking search, even at a released, stationary, collision-checked state | Independently test original timed joint parking routes; full native hand contact envelope applies from their current start through every sample. Neither release nor clearance is fabricated. No valid path -> retained released hold. |
| Three rest shoulder headings | Treated0/+/-20deg as exhaustive; opened panel can occupy those exact cells | Same six retracted rest joints, original20deg heading spacing and current measured shoulder heading within actual native limits. Finite search, original collision/timed-route/actual parking proof mandatory. This extends planner alternatives, not safety margins. |
| Exit-pending action whitelist | Also barred geometric planning | Permit pure existing PLANNING operations in staged profile; acquisition and base actuation remain blocked until verified exit. |
| Ungrasped reverse fallback | Mobile panel mode lacks support validation | Human preference recorded: retained handle reversal first; optional panel push only after failed reverse and release. Not enabled by deleting the unsupported-mode guard. |

The correction is shared across geometry/episodes, with no asset-ID exception, altered physical controller or raised protection threshold. New full Isaac qualifications wait for failure injection, closed-state native geometry replay and frozen-source audit.

Closed-state [R02_EXIT_REPLAY.json](R02_EXIT_REPLAY.json) reproduces all three original native refusals from the second P0 trial's last released hold. The shared rest-heading search then finds native rest at shoulder-40 degrees; original DIRECT/timed validation accepts3196 samples including native hand contact envelopes. This is a geometric/time-law replay, not executed parking and not a retrospective PASS. It reads the retained `planning_current_scene.json` (full native geometry); the existing finalizer deletes redundant `cooked_initial.json`, not a compressed archive of that file. Response/geometry hashes bind the replay inputs.

Non-safety wrist observation/segmentation exceptions are recorded and no longer veto independently native-checked parking. Fresh actual hold/unload is rechecked after the sensing attempt; SafetyStop still propagates immediately. Fault injection covers both outcomes.

Exit follow-up selected regression: **163 passed,59 subtests passed,0 failed**,60.20s. [OFFLINE_EXIT_RESULTS.json](OFFLINE_EXIT_RESULTS.json). Counts overlap prior regressions and are not cumulative. Frozen606/71/956 source audit unchanged.
