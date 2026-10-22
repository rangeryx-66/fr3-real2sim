# Common Agent/action API

Transport: CLI `command --episode DIR --sequence N --json FILE`. Atomic JSON publisher; increasing integer sequence; response `agent_responses/NNN.json` includes ACK/completion. Inspect reason/status and current state, never just file existence. `status` reports current files; old WAITING files can remain, so sequence and completed responses are authoritative.

All operations use `{"operation":"NAME", ...}`.

After a plan completes, render its actual observed points and native fingers without historical configuration reads:
`python -m agentic_interaction_v8.inspect_plan --episode OUTPUT`.
Inspect `AGENT_GRASP_PLAN_INSPECTION.png`; its provenance binds current plan/point-cloud hashes and episode robot IO. This read-only helper does not approve execution.

| Operation | Inputs / preconditions | Output / meaning |
|---|---|---|
| observe | unloaded, no hard latch | fresh RGB-D/SAM, inspection board, generation and measured camera/robot state |
| inspect | current decision boundary | current wrist geometry, missing/uncertain regions, candidates, feedback and resources |
| evaluate_view | `target:{eye_world_m:[x,y,z],look_at_world_m:[x,y,z]}` | deterministic IK, route, self/scene collision and visibility; caches result by command N |
| move_and_observe | `evaluated_command:N` | execute cached validated actuator route, measure camera/FK, capture fresh wrist observation |
| build_geometry | `planned_segments:[...]`, reason; drawer preload and family | independent per-view fusion, measured support vs uncertain rear hypotheses, robust candidates + current-state execution checks |
| validate_execution | current-generation plan | measured-state return/approach/closing feasibility; no physical acquisition authorization yet |
| attempt_grasp_and_probe | same predeclared `segments:[...]`, explicit reason/approval, drawer preload | original approach/closure/one-second hold, finite probing/feedback, guarded release/parking; failure preserved |
| inspect_contact_result | attempt feedback | current-attempt contact/force/aperture/motion plots, model status and sources |
| reobserve_and_replan | verified unloaded, recoverable failure | invalidates stale plan, captures fresh actual object state; follow with new candidate generation |
| safe_release_retreat | deterministic safety owner | RELEASED / CLEARANCE_REACHED / SAFE_PARK_REACHED / SAFE_EXIT_UNAVAILABLE, not interchangeable |
| finish | no additional manipulation | guarded exit, close estimator/actor, export result |

Drawer `build_geometry` also accepts `grasp_family:"existing_bar"|"compact_radial_cap"`, `native_g1_preload_request_n:1.0`, `transverse_unloading:false|true` (the **previously validated** bounded no-rail V7.5.1 branch; choose on evidence, not object ID). Cap family is physically validated for prismatic compact caps; frozen revolute owner retains its original bar generator. Do not claim universal knob support.

When SAM is incomplete, inspect semantic/depth hypothesis boards. `observed_handle_hypothesis:{source:"SAM3"|"OBSERVED_DEPTH",id:...,sha256:...,reason:...}` explicitly approves a current observed hypothesis. Obtain exact ID/hash from current packet. Optional alternative `candidate_request` must use current catalog SHA/ID/reason; never hand a saved transform to the planner. Inspect source implementations for the exact current catalog fields if needed.

Segment example (SI native coordinate):

```json
{"role":"FIT_ACCEL_1","coordinate_unit":"m","target_q":0.018,
 "reference_speed_m_s":0.004,"reference_acceleration_m_s2":0.008,
 "profile":"constant","fit_intervals_s":[[0.6,12.0]],
 "allow_target_truncation":false,"hold":true}
```

These numbers are a syntax example, not a prescribed view, target or permanent policy. Agent selects targets from measured state, reachability, excitation and resources. Revolute uses `coordinate_unit:"rad"`; TCP tangential speed/acceleration remain m/s, m/s² and the owner uses perpendicular hinge radius. Estimation effort is N·m/inertia kg·m² for doors, N/mass kg for drawers; do not mix them. Velocity-program variants are defined by frozen `interaction_runtime/probing_profiles_v3.py`.

Predeclare independent role `VALIDATION_*` with no fitting intervals. Common SG121 applies to torque/force and every regressor, downstream inertia exactly once, segment-local only. Drawer full-window sliding eligibility remains active. At least three distinct eligible FIT trajectories are needed before selecting position-dependent resistance. Conditional parameter uncertainty excludes unknown simulator/systematic sensor error.

Action status `TARGET_REACHED` requires measured target/stopped checks. `SAFE_PARTIAL_PROGRESS` is useful but not target completion; `TRANSMISSION_FAILURE`, `SAFETY_STOP`, `BUDGET_EXHAUSTED` remain distinct. Valid partial FIT requires predeclared eligibility; incomplete validation is N/A. Recovery needs verified unloaded state and remaining validation/exit budget. HARD stop cannot be cleared. Actual pad normals are normal-only reports, not complete wrenches, pressure area or proof of separation.
