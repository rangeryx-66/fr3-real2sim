# Hybrid Pull–Push–Regrasp V0: bounded development result

**Result: hybrid physical validation did not pass.** One valid closed-start episode on 46427 established a visual bilateral grasp and the original one-second hold. The first pull stopped at the unchanged measured TCP-speed protection before a useful opening range was reached. Panel pushing and the second planned handle acquisition were not executed.

Scope: **CODEX_AGENT_WRIST / RGBD_SAM3 / KNOWN_HINGE_STATE_COLLISION / IDEALIZED_LOAD_CELL / APPROXIMATE_ACTUATOR_SIMULATION**. Frozen source before physical outcome: `3bbfb47`. References `136b6e0`, `d6e690f`, `bd501b9` remain unchanged.

| Stage / metric | Actual result |
| --- | --- |
| Fresh wrist perception | Two actuator-executed views; one Agent-selected adjustment; pregrasp RGB-D refinement |
| Geometry / plan | 143.45 mm measured handle extent; automatic native-G1 plan; no GT handle grasp fallback |
| Physical acquisition | Verified bilateral contact, L/R 0.533/0.456 N; original one-second hold |
| First pull target | 10° exploratory target |
| Maximum actual angle | 0.024477° |
| Active post-acquisition gain | 0.022185° |
| Raw cumulative positive / negative travel | 0.028145° / 0.003668°; includes incidental settling, no deliberate panel closing |
| Planned panel push / post-push regrasp | Not executed / not executed |
| Handle acquisitions / intentional handle releases | 1 / 1; unexpected recovery acquisitions 0 |
| Original local contact correction | One; contact reports reacquired before resuming the probe |
| Termination | `PROBE_CARTESIAN_SPEED_LIMIT` at `LOCAL_CENTERING → TRANSMISSION_PROBE` |
| Loaded measured TCP peak | 8.248790 mm/s; original limit 5 mm/s |
| Joint-target FK speed at that step | 35.194251 mm/s |
| Peak raw pad load / minimum joint margin | 1.067431 N / 0.868187 rad |
| Safe task termination | Guarded release, clearance and verified `native_rest` parking completed, final pad loads 0/0 N |
| Simulation / wall time | 452.917 s / 3348.320 s (55.81 min) |
| Evidence | 108,700 physics and observation records; wrist F/T valid 108,700/108,700; continuous 13,587-frame video |

## Concrete first divergence

At simulation time 311.687500 s, the original local-centering branch resumed the opening probe. FK of the **actual commanded joint targets** gives 35.19 mm/s, while measured joint FK and measured body TCP give 8.21 and 8.25 mm/s. The original measured-speed stop activated. There were no original force-overload events. The new panel-wrench guard had not been activated.

The `planned_T_tcp` diagnostic is stale during local centering; its apparent jump is not used as command evidence. Joint targets independently confirm the handover discontinuity. TCP/body relative translation within the short pull remained below 0.091 mm. A single-pad zero report therefore does not establish complete grasp separation. This result establishes a control-handover blocker, not a door mechanical limit or a physical rejection of panel pushing.

All-phase unloaded motion reached 27.77 mm/s under the unchanged free-motion trajectory. The 5 mm/s figure above is the original loaded-manipulation protection; no limit was relaxed.

## Implemented and checked offline

The isolated owner exposes handle pull, unloaded reposition, observed-panel push and planned post-push regrasp operations, reusing the original closure, Pink and guarded parking. The panel exception is restricted to nominated moving-panel colliders and existing native fingertip end faces. All other collision checks remain.

Archived RGB-D/native-solid checks found a missing physical central panel surface. Observed upper/lower moving door rails have real native collision surfaces; no collider was added. A closed-G1 upper-rail proposal had an offline 6°→2° contact route, separation and parking path. Release/parking feasibility was checked at 2°, 6°, 10° and 15°, and nonzero-angle approach was checked separately. These are offline preparation results, **not physical panel-push validation**.

## Physical learning outcome

The interrupted pull was preserved independently: 526 native-rate samples over 2.19 s. The numerical worker ran only after verified unloaded parking and produced eight fitting bins, zero validation bins. Its constant-model conditional fit gives a near-closed Coulomb term of 0.01785 N·m (conditional standard error 0.00277 N·m); inertia and damping are unsupported. There is insufficient angular coverage for a state-dependent model and no held-out force or forward-motion validation. No improved physical twin is claimed. Raw and compensated six-axis data remain available privately.

## Execution integrity and next action

Two zero-contact observation starts were cooperatively stopped before the valid attempt, with their complete records preserved: one for a new panel-guard gravity-sign integration defect, one for the opened-door view-planning reference interface. Both had zero closure calls and zero door motion. The repaired source passed frame/sign and current-door callback checks, then was frozen before any physical grasp outcome. The valid episode had one continuous scene, no restart/stitching, no object actuation, no attachment and no extra physical recovery attempt.

The next bounded engineering action is to reproduce and repair the shared local-centering-to-Pink command handover in a separate version, carrying the command/reference/seed state continuously while preserving the 5 mm/s protection. No such controller change or physical retry was performed here.

Raw RGB-D, video, six-axis streams, contact records, hidden dynamics and detailed evidence indexes remain private. All 496 archived files were checksum-verified locally against the server; frozen source checks showed no changes. Owned Isaac/encoder processes were closed. The original benchmark scores and successful baselines were preserved.
