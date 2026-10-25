# V8.4 ABC continuous execution (engineering opt-in)

Activation: `integration_profile: v84_abc_continuous_v1`, engineering only.
Default V8.0/V8.1/V8.3 profiles retain their behavior. No frozen physical,
collision, closure, actuator, progress-clock or estimator mathematics changes.

For the separately registered short comparison only, profile
`v84_abc_short_comparison_v1` permits normal finish after both legacy and
continuous physical strategies have actually produced completed records.
This declares the diagnostic trial finished, never the global 90° goal
achieved; failed records remain failures. Full-task profile cannot use it.

```
A fresh wrist -> current native whole-chain plan -> inspected approval
  -> acquire_execute (finite long native-coordinate action[s])
B native 240 Hz tracking + original hard supervisor + synchronized recorder
  -> achieved stopped endpoint -> C (when explicitly requested and trusted)
C numeric critique -> continue_motion on same prechecked declared action(s)
  -> B, or guarded release/park -> A/fresh recovery/base relocation
```

`acquire_execute` inputs: original approved `segments`, current inspected plan
image SHA, purpose/reason, legal planned preload, optional `retain_contact`.
Current object state, known revolute/prismatic geometry, limits, base and
observation generations are bound by the actual episode/native plan, not
caller-supplied transforms. Plan the whole finite chain with `prepare_grasp`.
An initial executed subset may leave remaining declared actions for C.

`continue_motion` additionally requires the current positive
`contact_generation`. The exact target, type/unit, direction (current q to
target), speed, acceleration, velocity program, purpose/FIT or VALIDATION
role and intervals must match an already approved native-checked declaration.
It checks the current measured successor route again. New geometry, target,
preload or grasp requires unloaded planning; no manual poses are accepted.

Both return real joint progress, per-action completion/stop reason, actual
remaining original goal, contact generation and resource state. The original
240 Hz contact recorder supplies reference/object q, qd, planned target and
actual TCP, joint velocity/reactions/available efforts, F/T, aperture, normals,
slip, safety and timestamp/step. Position derivatives remain segment-local;
do not infer measured motor output from a projected reaction or applied zero.

An explicitly retained C is **not safe parking**. Native physics and original
supervision run during its maximum 20 wall / 5 simulation seconds. Finite
slip and one absent positive-pad report do not themselves veto retention.
Untrusted sensor, unstable recent aperture/slip, expiry or an explicit release
returns through original unload/clearance/park. An episode can continue from
that actual state; C expiry is not episode failure. Never wait indefinitely
for Agent reasoning under load. Use the V84 atomic publisher for loaded C.

`ABC_WAITS.json` records start/end, reason, dependency and sim/wall times.
Read-only contact plots are cached at identical record/model state and kept
out of loaded critique. Parked `validation_queue.boundary` is nonblocking in
this profile; immutable numerical publications remain asynchronous. The
original bounded guarded validation admission and freeze/seals remain.

Feedback levels:

* Advisory: ordinary load warning, missing one pad, finite slip, or original
  nominal-demand warning with resolved actual object/TCP co-motion.
* Local feedback: original continuous clock/Cartesian trajectory limits and
  velocity/acceleration profiles; no new angle-specific rules.
* Recoverable return: two full original monitor windows with unresolved
  object progress or poor actual TCP transmission; original increasing-slip
  stop/startup detector; planning rejection; contact no longer retainable.
* Hard: original force, collision, joint, velocity, lost-contact, invalid F/T
  and controller guards. No clearing, retry-through or threshold relaxation.

The original 0.5-degree revolute lead and original rail stopping-distance
lead remain unchanged. Closed C01 replay indicates substantial real contact
phase drift consumes this allowance. It does not support simply increasing
the cap. The V84 monitor preserves the original nominal diagnosis in its
shadow/history while separating actual co-motion from lost transmission.
This can continue useful slow motion; it does not by itself promise speed.

When a continued FIT segment crosses an original nominal warning, the whole
segment is conservatively excluded from candidate FIT (operation success is
unchanged). No new samples refit the two protected V83 frozen external models.
Complete VALIDATION stays separate, frozen before motion; incomplete stays
excluded. Numerical readiness never authorizes a new grasp or overrides safety.

Rollback: select the V83 profile/source. V84 is constructor-selected and
scoped to new actors; never hot-load into a running episode. Compare fixed
configuration complete episodes before claiming throughput or reliability.
