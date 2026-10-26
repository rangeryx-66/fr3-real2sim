# P0: motion admission and measured staged exit

Status: **90° physical qualification NOT PASS; hard-latched World retained for read-only diagnosis.** Code and offline regression passed. After a pregrasp path rejection, a different checked IK/approach completed 20.222° forward and 12.255° reverse, followed by confirmed release, local retreat, fresh observation and native parking. A later long segment reached 40.144539° before the unchanged sustained wrist-torque guard latched. Current release, retreat and parking are unconfirmed. The preferred-parking rejection fixture did not fire because 90° was not reached. No full-task success is claimed.

Motion/exit core: local `50683b1`, server `48fdaaceefcb2f95d279074af3678a1b8db87d3d`. Strict launcher registration repair: local `2724b7c`, server `4b94ad7f40eeb03d75d5058be09550c0a381fe0c`, the physically tested source with the incomplete outcome below. The old `aaf583b` Actor was safely finalized for the latest owner priority change; no hot replacement or World reset occurred.

## Incorrect dependency

The revolute whole-chain certificate used `native_g1_safe_park.plan_exit()` at each predicted stopped action endpoint. Failure returned `REVOLUTE_CHAIN_EXIT_UNAVAILABLE`, and target coverage also depended on `exit_valid`. The prismatic certificate similarly returned its final predicted parking result as the entire chain's validity. Both mechanisms could reject safe motion because a remote terminal route had not been found.

The opt-in `staged_exit: true` registration separates these tasks. Current approach, closing sweep, retained command bias, native collision checks, centering extrema, reference lead and braking envelope remain mandatory. Exact stopped target configurations are additionally checked with retained bias and loaded finger widths. Future release and parking are explicitly UNKNOWN at motion admission, not falsely marked valid.

The actual exit owner stops under the original controller, verifies the existing measured robot/TCP/object stationary dwell, and checks the actual jaw-release sweep. It confirms release before planning a local 4/8 cm retreat with the original native predicates. After executing and verifying local clearance, it captures fresh wrist evidence and plans each existing native/shoulder-rest alternative from the latest physical state. Original timed-route checks and measured parking verification remain authoritative. Local clearance is never called parking.

Results distinguish `EXIT_PATH_NOT_FOUND`, `LOCAL_RETREAT_NOT_FOUND`, `SAFE_RELEASE_NOT_CONFIRMED`, `SAFE_HOLD_NOT_ESTABLISHED` and `HARD_SAFETY_STOP`. Missing parking at a verified released hold preserves completed motion and the same World, while new contact/base motion waits for checked exit recovery. A trusted retained grasp can use the existing finite ABC C boundary; no indefinite unsupported load-bearing wait or automatic latch reset is added.

The prismatic scheduling adapter retains the original approach, complete FIT/VALIDATION motion forecasts and independent exit reserves. Unknown future terminal-route cost is reported as unknown. It is not replaced with a fabricated zero-length route or an enlarged execution budget.

## Offline evidence

| Replay | Original approach/closing | Old result with explicit terminal-route rejection | New motion result | Native configurations checked |
|---|---|---|---|---:|
| Closed R02, actual start 44.899°; original 90°/1.15 rad declarations | Passed | `REVOLUTE_CHAIN_EXIT_UNAVAILABLE` | Passed; future exit not queried | 5,136 |
| Closed 45746; actual start 34.575 mm; original declarations | Passed | `CHAIN_EXIT_UNAVAILABLE` | Passed; future exit not queried | 5,376 |

These are native geometric replays with a clearly labelled conservative planning rejection, not new physical episodes or natural collision evidence. They use recorded actual contact bodies, nonzero retained equilibrium commands, current moving-body geometry and declared hinge/rail priors. No hidden dynamics enter the test. The rail's translated native anchor is checked against its recorded reference plus actual displacement; perpendicular geometry tolerance is unchanged.

Historical derived inspection boards had been pruned from these closed archives. Original RGB, camera records and accepted-observation trace identify the replay frames; no missing approval image or hash was regenerated. Initial frame-binding and rail-anchor reconstruction failures remain preserved privately.

The selected final regression passed **133 tests and 26 subtests**. It includes blocked specified parking followed by local clearance, fresh observation and alternative parking; no-retreat and unsafe-release cases; exact stopped collisions; original timed-route rejection; resource exhaustion; same-World recovery; and authoritative hard latches. Overlapping earlier test selections are not added to this count. Frozen controller, configuration, Skill and Physics-ID paths remain byte-identical to the protected baseline.

The first launch attempt omitted the existing scripts import path; the second revealed that the strict launcher whitelist and frozen-V8 compatibility filter had not yet registered the two new opt-in fields. Both failed before any Actor or World was created. Their logs and independent instance records remain preserved. The minimal launcher repair accepts those fields only under the V8.7 profile and removes them from the unchanged V8 resource schema. A complete-entry regression stops deliberately before physical preparation; its combined launcher/isolation/GPU/staged-exit selection passed **44 tests and 14 subtests**. This overlaps the motion-core selection and is not an additional success count.

## Physical qualification still required

### Current physical outcome: hard stop, not complete

The entire run used one Actor/World and preserved the actual object state. After one native base relocation to approximately (0.15, −0.70) m, fresh wrist observation produced a complete handle mask. The automatic candidate pool covered at most approximately 58.34°, so the unexecutable direct-90° request was rejected by **current motion coverage**, without querying future parking. A same-generation replan selected a fully checked 55° successor (family −0.3, −9.375 mm, 180°, 0°). It did not reset the original goal or object. Only the declared forward segment was executed; later reverse/validation declarations were not silently executed.

At 409.875 simulated seconds and actual 40.144539°, original `SUSTAINED_OVERLOAD` triggered. A read-only replay using the original sensor transform, calibration bias, robot-tool gravity compensation and unchanged `ForceMonitor` reproduced the first trigger at exactly 409.875 s:

| Guard quantity | Original 0.15 s trailing mean | Original sustained limit |
|---|---:|---:|
| Maximum pad load | 3.972986 N | 4 N |
| Wrist force norm | 4.062922 N | 8 N |
| Wrist torque norm | **0.600200 N·m** | **0.6 N·m** |

Measured TCP speed at the trigger was 13.891 mm/s, below the unchanged 30 mm/s hard limit. Recorded TCP command tracking error was approximately 0.453 mm and relative slip approximately 1.282 mm. This establishes which original guard fired, not the complete physical cause of the increasing load. A provisional engineering threshold being crossed is not evidence of hardware damage. No force/torque threshold or reset qualification was changed.

| Result | Evidence from this episode |
|---|---|
| Maximum actual operation progress | 40.144539°; 55° and 90° targets not reached |
| Completed reverse | 12.254582° in the earlier block; no post-90° reverse |
| Earlier completed-block release / retreat / park | All confirmed, independently indexed below |
| Current post-fault release / retreat / park | Not executed / not executed / not verified |
| Controller stationary-stop acknowledgment | Not verified; no post-fault stationary window |
| Simulator state | Finite, paused, same World retained; original hard latch present |
| Independent validation | Incomplete; original request preserved, not counted as passed |
| Preferred-parking rejection after 90° | Not exercised |

Read-only command 20 confirmed `SAFETY_LATCHED_WORLD_RETAINED`, `motion_authorized=false`, `physical_park_verified=false` and no qualified reset owner. No motion/reset/finish command was submitted after the hard fault. Actor PID 3138040 remains in the original finite diagnostic session; this is not a new episode or a verified hardware brake. The instance hold and `TERMINAL_SAFETY.json` remain `HARD_STOP_UNPARKED`. The old global hold hash remains exactly `f9cec4d6bac8a3cba1681617b8d6ab64a5d7eb2087b05fb7a4c51dd879d58ab0`.

Private evidence: `STAGED_R02_HARD_GUARD_REPLAY_01.json`, `STAGED_R02_HARD_STOP_AUDIT_01.json`, original responses, sensor chunks and both old/new safety records. All chunk hashes were verified. The interrupted final FIT segment is ineligible under the original selection rules. Earlier safe parking is not substituted for the current unconfirmed terminal state.

The actual R02 90° alternative-parking test and a new 45746 physical episode remain incomplete. The shared robot/Isaac resource is still owned by the retained R02 World; no second controller was started. The native 45746 geometric replay above passed, and the historical 180 mm/reverse/independent-validation/safe-parking episode remains preserved; neither is presented as a new physical success of this source. Further physical continuation requires the existing safety qualification to be satisfied. Fixing a new hard-latch reset owner or changing force control lies outside this narrowly scoped parking-admission change.

### Earlier completed block (separate evidence)

Sequence 5 used a changed direct IK approach (terminal q7 about −1.134 rad, rather than the rejected long branch near +5.12 rad), with fresh RGB-D and actual measured-state revalidation. Forward motion reached 20.221940° in 15.9 simulated seconds; reverse reached 7.967358° (12.254582° reverse) in 25.2625 s. Both completed with valid physical transmission and supervised stopping. After actual state qualification, release was confirmed at 199.841667 s, a checked 4 cm local retreat reached at 203.883333 s, fresh wrist sensing completed, and measured native-rest parking reached at 215.2625 s with zero pad loads. Object state after release was approximately 7.889°. The same World, original 90° goal and history remain active.

The independent 14° reopen request was deferred, not executed: reverse FIT produced no eligible regression samples under the unchanged Physics-ID criteria, leaving the frozen model unqualified for independent validation. This did not block operation or exit. That request remains pending. A hashed private interim audit records the actual outcomes separately (`STAGED_R02_INTERIM_PHYSICAL_01.json`). The original parked-boundary ledger accepted 1,800 extra wall seconds for useful continuation; original simulation and exit reserves are unchanged.

Prepared episode `v87_r02_staged_exit_20261026_01` uses the original scene/physics commitment, original launcher and independent instance admission. A conservative, once-only planning rejection of the first available parking route is registered only after the measured task goal is reached. It does not inject collision or force, and cannot bypass any actual guard. The report must separately establish actual operation, release, local retreat and final parking; no complete success is currently claimed.

The physical qualification configuration selects GPU5 for Isaac and GPU6 for perception using the original 12 GB resource reservation and original unloaded self-check. No measured-capacity override or safety-lock change is used for this new test.

The original 156-step unloaded admission passed. Instance UUID `a5c89fe3-51f0-4e46-ab84-f4cc2d7d753b`, Actor 3138040, runs frozen server source `4b94ad7`. Sequence 2 rejected `gripper_link1`/cabinet collision at the actual timed pregrasp recheck; `physical_grasp_started=false`, no interaction segments, object 0°. The exit owner confirmed unloaded state at 30.25 simulated seconds, executed a checked 4 cm local backoff, refreshed wrist sensing, and reached the original measured native-rest cell at 35.925 s with both pad loads zero. Recovery preserved the World, original goal and history. Sequence 3 reacquired a complete current wrist handle view; sequence 4 generated fresh automatic candidates. No progress from another episode is spliced into this run.

A separate boundary regression reproduced a stationary exit dwell incorrectly consuming ordinary work budget. Requesting the original `RELEASE` phase before the existing braking/settling owner preserves its original exit reserve and limits. Targeted staged-exit/governor regression: **29 tests and 14 subtests passed** (overlapping earlier selections). Frozen future source: local `cb5baa4`, separate server worktree `c80a87e`; this correction is **not hot-loaded into the live 4b94ad7 Actor**. No physical qualification is claimed for this additional boundary correction.
