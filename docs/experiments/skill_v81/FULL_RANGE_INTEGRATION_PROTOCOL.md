# V8.1 fresh blind full-range integration gate

Authorization: user request following `0d5cfbb`. Two new scenes in order:
revolute45936, then prismatic45290. This supplements, never overwrites, the
earlier same-physics V8.1 engineering protocol/results.

One valid episode per joint type; no automatic retry or tuning after outcomes.
An initialization-only infrastructure failure must be documented separately.
After a failed episode, diagnose before deciding whether the other independent
joint-type test can safely proceed. No unknown-axis benchmark is authorized.

Acceptance in ONE continuous scene: actual90° revolute opening or at least150mm
drawer travel, meaningful measured reverse motion (the existing22.5°/37.5mm
protocol minima), qualified bidirectional FIT, at least one complete independently
declared VALIDATION, guarded release and measured safe parking. Do not combine
stages from different episodes. Short-stroke dynamics support remains explicit.

Resources per scene: dedicated `full_range_integration_debug_v1`,5400s wall,
2700s simulation,600s unloaded decision window,600s wall/120s simulation exit
reserve. At most six acquisitions and100Agent operations. No slow Agent decision
under load, no deadline-reset heartbeats, no extra probing in shutdown grace.

The isolated generator uses the existing V7 positive-density cuboid mass/inertia
construction: mass scale0.75–1.25 and density gradients−0.3–0.3. Fresh independent
secret seeds; constant native passive resistance drawn from the registered V7
family: revolute Coulomb0.018–0.030N·m/damping0.1–0.35N·m·s/rad; prismatic
Coulomb0.2–0.6N/damping1–6N·s/m. No spring, object drive, robot authority,
materials, collision geometry or protection changes. Public commitments are
committed before launch. Realized values/seeds/native authored audits are private
scene-generator/evaluator records, never Agent/controller/estimator packets.

Codex inspects fresh wrist RGB/masks/depth/clouds and measured contact/motion
feedback. Choose reachable targets, profiles, stations and legal grasps/preload
from that evidence. No archived grasp/view/station schedule, GT grasp surfaces
or passive camera input. Native colliders/known articulation/state remain explicit
safety/kinematic priors. Any base motion requires verified release/unload/park,
whole-route native validation, actual actuators, new observations and new grasp
approval. All hard stops remain authoritative.

Prioritize minimally informative acceleration-rich opening and different-speed
reverse FIT, then complete a distinct predeclared validation before optional
range expansion. FIT readiness is actual production signed eligibility; if an
immediate validation is deferred, release/park and decide from fresh evidence
after worker publication. Do not count the deferred request as validation. Keep
V3.1 common filtering, single tool compensation, FIT-only model selection and
complete segment isolation. Independently predict with held-out wrist effort,
one measured initial state and no later state correction. Report complete
physical duration separately from derivative-supported prediction horizon.

After each actor/Agent finishes, seal final online actions/model and only then
run the separate evaluator on authored truth. Report parameter errors against
authored references, not unverified effective PhysX inertia. Preserve every
rejected plan, failed contact, partial action, relocation and recovery; compact
240Hz interaction data/sparse wrist checkpoints private, no continuous video.

Both full-range gates must pass before a behavioral V8.1 release/readiness freeze
and two fresh independent repetitions per joint type. A short recovery/validation
pass alone never opens that gate. If either fails, preserve the result and stop
the repetition branch; do not retune it into an apparent independent success.
