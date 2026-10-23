# V8.1 engineering extension

Latest result: [recovery-first report](../docs/experiments/skill_v81/RECOVERY_FIRST_REPORT.md),
**NOT PASS**. A physical short pull/reverse, base relocation and new sensing
completed, but no post-relocation regrasp occurred. New preload tests and
full-range resumption remain gated.

## Recovery-first stop gate (2026-10-23)

Full 90-degree / 150-mm experiments and independent repetitions are stopped.
The next permitted physical test is one short fresh-scene recovery sequence:
actual release/park, validated native base motion, fresh wrist observation and
automatic new grasp, physical regrasp, measured pull/reverse, then safe parking.
Only after that branch passes may the registered matched legal-preload comparison
run. No object/actuator/guard changes are authorized by these interfaces.

After a current prismatic first-batch native planning failure, the Agent may
request `recovery_search_round:1` on `prepare_grasp`, with the same current
inspected observation and a reason. It evaluates the next **18 disjoint existing
families** using the unchanged ranker and full native checks. It is bounded to
two batches total; a new view, mask, station or acquisition invalidates the
previous search identity. This is candidate search, not physical recovery or a
claim that the complete grasp manifold was exhausted. Revolute planning has no
second-batch override.

If a first-batch plan is native-feasible but the Agent declines it after
inspection, the same second round accepts `decline_current_plan` with
`image_sha256` and `reason`, in addition to the current wrist-image provenance.
The actual plan ticket, image bytes, station, observations and acquisition count
must still match. This enables informed candidate choice without a new contact
score threshold or approval of a mechanically weak plan. It does not allow a
third batch or a pose supplied by the Agent.

The optional prismatic-only `relocate_observe_prepare` accepts a current
inspected wrist image, Agent-selected `target_xy_m`, reason and finite
`planned_segments`, plus the usual legal preload/planning options. It uses the
original current-state station validation and actuator move, fresh sensing and
tentative automatic grasp preparation. It never acquires contact. The Agent
must inspect the new observation and plan before separately approving a grasp.
If the new target is ambiguous it returns without selecting a mask or planning.
Same-episode measured operation costs are exposed at boundaries; a known
insufficient work budget refuses the batch before motion, preserving exit
reserves. This batch is software-tested, **not yet native-validated**.

Prismatic planning retires stale plan/image files by moving them into
`plan_history` and exposes `current_plan_ready` explicitly. Failed recovery
packets cannot offer an old grasp board as current. Physical approval remains a
separate Agent operation. Repeated native checks are skipped only when their
**exact fine-path utility** cannot beat an already fully native-validated winner,
matching the frozen planner's strict comparison. No collision result is invented.

## Contact-repair engineering interface (not a validated release)

`prepare_grasp` takes the same inputs as `build_geometry`. It first validates the
complete finite FIT/VALIDATION declaration using the actual execution protocol
and native unit/limit checks, before any geometry/IK search. It then performs
the existing automatic planning, renders the current observed-support/native
finger board and validates the measured-state return/approach. Its result is
ready for **Agent review**, never physical approval. The Agent must actually
inspect the returned board and separately authorize `attempt_grasp_and_probe`.

`observe_view` takes `target`, current `inspected_image_sha256` and `reason`.
It combines the existing deterministic view feasibility check with actuator
motion/capture only when that same route is feasible. Stale pre-contact evidence
is rejected. Neither operation invents a view, pose or high-level decision.

When fresh images identify the first accepted mask as a different object part,
prismatic `prepare_grasp`/`build_geometry` may explicitly request
`start_new_target_association:true`, with the current inspected wrist SHA,
reason and a current `observed_handle_hypothesis`. Only the latest selected
observation becomes the new association seed. All earlier captures remain in
history; actual wrist routes and the used view budget are preserved. Stale
images, incomplete hypotheses, loaded/hard-stop states and revolute jobs are
rejected. Existing observed-mask preservation/scoring and native execution
checks still own planning. No camera, robot or object is moved by this selection.

`select_observed_target` accepts that same explicit inspected target **without
grasp search**. It applies the unchanged observed-mask preservation, exposes
incremental geometry, and allows a complementary wrist view to add surfaces to
the correct association before one expensive planning call. It grants neither
physical approval nor extra observation budget.

Every live wait exposes `deadline_unix_s`. The publisher rejects closed or
ended episode windows before creating a command file. A published command is
still pending until its actual ACK; publication never implies robot execution.

Prismatic `contact_following:"rate_consistent"` with explicit
`transverse_unloading:true` is an **experimental** alternative to the default
`legacy_window`. It preserves the original causal load filter, correction
direction/gain, 2 mm total closure/centering ball, 5 micrometre per-native-step
cap and all original speed/acceleration/collision/force guards. Only correction
integration cadence changes; requested jaw force and actuator authority do not.
Its matched native test stopped at 23.32 mm, earlier than the 39.50 mm baseline;
it is now rejected by the full Agent planning interface. The explicit matched
fixture remains reproducible, but this negative ablation is not a repair.
The preregistered engineering comparisons are separate from fresh Agent episodes
and independent repetitions; no cached fixture is an online grasp fallback.

The unknown-articulation stage is only a
[prepared protocol](../docs/experiments/skill_v81/UNKNOWN_ARTICULATION_NEXT_STAGE.md).
Current known-axis/state results do not validate that future capability.

Start from the frozen [V8 Skill runbook](../.agents/skills/agentic-real2sim/references/RUNBOOK.md) and its explicit private environment/resource bundle. The original controllers, object/robot physics, actuator caps, safety policies and production estimation sources remain byte-frozen. Read the current [engineering report](../docs/experiments/skill_v81/REPORT.md): a full-range release is **not yet ready**, and independent repetitions are gated.

The separate launcher is `python -m agentic_interaction_v81.launch --spec JOB.json --environment ENVIRONMENT.json`. Its strict job schema is `agentic-interaction/v81`, with explicit `phase:engineering` or a gated `independent_repetition`, scene and opaque physics-file commitment, finite wall/simulation/acquisition/Agent budgets. Never edit the checkout loaded by an active actor. Source hashes, actual job bytes, loaded modules and completion are recorded independently.

Use the original observe/inspect/evaluate-view/move/build/validate/attempt/recover/finish workflow. Publish commands atomically with `python -m agentic_interaction_v81.transport --episode OUTPUT --sequence N --json COMMAND.json`. Actually inspect fresh wrist RGB/SAM/depth/cloud and feedback images before decisions. Preserve failures and all original hard stops; do not wait for reasoning under unstable load.

`build_geometry` requires `planned_segments`. An optional `planning_target_q` checks a longer native-coordinate horizon without inserting a physical action. The Agent must compare the returned feasible path with every requested physical target; a planning horizon is not an achieved displacement. The original robust families/scores and full native approach/closing checks remain owners of executable grasps.

Before any acquisition, `redeclare_actions` may replace that finite `segments` declaration after the Agent inspects the actual plan image (`inspected_image_sha256` and `reason` required). This no-motion operation avoids rerunning unchanged geometric scoring merely to select another already checked target. The plan bytes, accepted observation hashes, station generation and acquisition count must match the original planning ticket. Targets must stay inside the existing native-checked task arc; the original measured-state return/approach validation runs again, and every actual successor still requires the original live check. It cannot change a grasp, preload, observation or controller, nor reuse a grasp after contact or relocation. The old explicit approval via `attempt_grasp_and_probe` remains required.

The prismatic owner's independent V7.5 declaration is synchronized only after those checks; its original whole-chain collision, closing, resource-admission and preload checks still run at acquisition. `python -m agentic_interaction_v81.verify_action_contract` exercises the real frozen owner's declaration guards without launching Isaac. This software check is not native route or contact evidence.

`evaluate_station` takes `target_xy_m`, `inspected_image_sha256`, `reason` and optional `task_horizon_q`. It requires measured release, unloaded/settled robot and object, and verified park. It checks complete original robot/support geometry and a timed native XY-stage route. Its optional task forecast checks only the current observed automatic candidate, not all possible grasps or a future approach. A rejected candidate arc does not prove the station impossible. `relocate_station` takes a prior `evaluated_command`; execution revalidates actual state and uses ordinary actuators. Only XY authority exists: no yaw/height or object-state teleportation. This simulated carriage is not verified wheeled hardware.

After any base movement, old geometry, view paths, approach and execution approvals are invalid. Capture new wrist RGB-D, move through a newly validated arm view when needed, regenerate geometry and automatic grasps, inspect and revalidate. The Agent chooses stations from current observed geometry, known articulation/state and actual safety feedback; there is no stored successful station schedule or grasp transform.

Relocation's additional stationary certificate uses a continuous 0.3 s of consecutive measured240Hz joint, TCP and object configurations, within the verified parking cell. It rejects gaps/stale measurements and real motion; the finite2s hold and original per-step protections remain. Native instantaneous joint velocities are recorded alongside this causal motion certificate, rather than overwritten or presented as drive outputs. Three short unloaded native checks pass, but the earlier native-velocity/configuration discrepancy was not reproduced and is not claimed solved mechanistically. A full interaction is still required.

Execution first tries an original-checker-certified direct route from current measured/equilibrium joints to the original execution start, falling back to the frozen return search if unavailable. It never omits approach checks, original timed actuator execution or measured-state pregrasp refinement. This avoids revisiting obsolete observation waypoints only when the shorter whole route is actually safe.

Parking alternatives are also checked with the original executed timing; a coarse DIRECT result alone is insufficient. A known route-planning rejection after confirmed jaw release/unload may trigger one immediate measured-state exit replan through the original bounded clearance/parking search. This is an exit-only primitive: no additional manipulation, force-history reset, Agent wait under load, collision waiver or hard-latch reset. If either the unloaded preconditions or the second route fails, preserve `SAFE_EXIT_UNAVAILABLE`; do not call local release/retreat full parking. The short native test uses an explicitly labeled software planning refusal and is not a repaired physical outcome for the failed R05 grasp.

Prismatic candidate memory distinguishes an already demonstrated direction/interval from an opposite-direction failure. It uses this episode's measured results only, with the same tested preload, verified exit and fresh geometry; every reconsidered family is generated again and passes the unchanged native checks. A new image alone never authorizes repeating an unchanged failed direction. The initial request remains 1.0 N; the existing legal menu remains 0.5/0.75/1.0/1.5 N at unloaded boundaries. Requested preload is not measured force, and drive authority is unchanged.

After inspecting current wrist evidence, the Agent may explicitly set `exploratory_task_reconsideration:true` on `build_geometry`, with the current `inspected_image_sha256` and a reason. This permits reconsidering an **unfailed direction** outside previously measured support, or a **shorter entire unidirectional acquisition** bounded by a recorded transmission-qualified prefix. The latter excludes the final monitor window and subtracts the existing contact-lead allowance plus measured servo offset. It bounds the whole declared acquisition: splitting actions does not reset it. These are exploratory eligibility changes, not success predictions. Hard/unknown faults, unsafe exits, stale evidence and unsupported failed-direction repetitions remain blocked; every newly generated pose still needs original whole-chain native checks and real physical protections. Completed/failed historical statuses and FIT qualification are untouched.

The V8.1 revolute feedback display uses Cartesian tracking and a rigid hinge arc, never radians-minus-metres. It binds the current attempt and caches identical measurements; point contacts without verified lineage remain unavailable. Its bias/gravity-compensated plots retain dynamic tool inertia and are diagnostic only. Production FIT/VALIDATION still uses the original V3.1 common filtering and one tool-inertia compensation. Signed qualified FIT support and same-episode model hashes gate independent validation, whose model is sealed before execution. Unready validation is deferred with guarded release, not counted complete.

Keep native 240 Hz essential interaction data and sparse wrist checkpoints private. The two passive geometry cameras and hidden dynamics are never Agent inputs. Release/parking and process exit are separate checks from manipulation success. Both full engineering lifecycles must pass before `ENGINEERING_READINESS.json` and the separately frozen repetition registration can be activated. No large paper benchmark is enabled here.
