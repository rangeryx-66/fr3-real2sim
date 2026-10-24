# V8.1 integrated engineering registration — 2026-10-24

Starting point: c1c1d22. This is engineering development, not a replacement
benchmark. Frozen V8.0 physical owners and revolute dependencies remain unchanged.
All new behavior is explicitly activated by `v81_force_replan_validation_v1`.

## Matched preload study

First obtain a fresh, automatically planned, unexecuted wrist-observed drawer
reference. The Agent inspects actual RGB/SAM/depth/cloud and plan, then exports
it with zero acquisitions. Freeze its content hash and the engineering physics
commitment before the four conditions. No GT handle or historical successful
pose is used. All conditions use the same station, observed grasp, physics,
original closure mapping, controller, contact following and motion profiles.

Order: 1.5 N baseline, then 2.0, 2.5 and 3.0 N requested preload. These are
requests, not measured forces or manufacturer ratings. One acquisition per
condition, finite short budgets, and original hard protections. Predeclare
pull to 18 mm, reverse to 6 mm, pull to 24 mm, then independent validation to
12 mm, subject to complete current-state native feasibility. Preserve every
failure. Stop escalation if actual unsafe contact or authoritative protections
prevent it; do not raise task thresholds to fit a failed peak.

Measure 240 Hz normal/tangential pad loads, signed rail traction, transverse
loads, measured/commanded aperture, native master target/state/cap, command/TCP
tracking, TCP/drawer slip and actual repeated-direction progress. Compare
retention over the entire block. Only measured-evidence-supported additional
requests become available to ordinary Agent jobs. If normal load rises without
improved retention, investigate observed contact geometry instead of further
force escalation.

## Integrated engineering episodes

After matched controls, use one fresh numbered scene per engineering version
and joint type. Fresh wrist observations, automatic grasp candidates and Agent
choices are mandatory; matched fixtures are forbidden here. Use fixed declared
engineering dynamics; preserve failures and never present these as independent
repetitions. Use the existing full-range DEBUG budget (5400 wall/2700 simulation
seconds, 600 wall/120 simulation seconds exit reserve, up to 600 seconds at a
verified unloaded Agent boundary).

Maintain the original goal (door 90 degrees / drawer >=150 mm) independently
of shorter intermediate actions. A real collision/IK rejection must preserve
its result and continue with a different fully checked candidate/action, or a
safe Agent-selected relocation and fresh regrasp if feasible. Injected rejection
is software coverage only, not the required physical recovery evidence.

Prioritize qualified acceleration-rich opening FIT, reverse FIT at a different
speed, and a complete separately declared independent VALIDATION before optional
range expansion. Immutable per-segment numeric packets reference existing raw
chunks without copies. Pending validation requests survive MODEL_COMPUTING,
DATA_INSUFFICIENT and WORKER_FAILED; no stale model is sealed. At most one original
one-second guarded endpoint hold permits a quick deterministic worker handover.
Otherwise unload/park, preserve the request, then replan from measured state.
Never freeze PhysX or wait for Agent inference under load.

Only after BOTH complete same-episode engineering lifecycles pass may a new
behavioral release be frozen and two fresh independent repetitions per joint
type begin. No unknown-articulation experiment is authorized here.

Raw RGB-D, F/T and telemetry stay private and compact. Publish source hashes,
derived contact/retention comparisons, real rejection/recovery traces, validation
status, actual range, parking, timing and remaining blockers. No short gate alone
constitutes completion of this integrated task.
