# V8.8 mobile base dominant coordinated manipulation

Registered before the first coordinated physical trial. This engineering test
uses the user's two fixed historical assets; neither is described as unseen.
The user's final order is **45290 prismatic, then 45600 revolute**.

| Asset | Selected native joint | Required forward / reverse | Prepared native range |
|---|---|---|---|
| 45290 | joint_prismatic_l_0_abstract_0_2 | 180 mm / >=40 mm | 0–378.413 mm |
| 45600 | joint_revolute_l_0_abstract_0_1 | 90 degrees / >=10 degrees | 0–91.8 degrees |

Ranges are URDF priors, not verified physical maximum travel. Other asset DOFs
remain native and are monitored. One selected joint is operated in each case.

Each formal case has its own 14,400 second absolute wall deadline, starting
with its launcher and including observation, planning, contact, recovery,
validation and exit. Only one Isaac owner may run. The existing 300 second
wall / 120 second sim exit reserve is protected; decision renewal must stop at
the work boundary, before the launcher's unchanged absolute watchdog. A local
planning/contact failure is not an episode termination condition. Hard latches
are never cleared. The historical 40.145 degree accident and closed prior pair
remain NOT PASS with their original evidence and process termination records.

Before formal operation, run offline fault tests, frozen-source audit, then a
separately reported bounded coordinated qualification. Use fresh wrist RGB-D
and actual observed grasp-plan boards. No saved grasps, manual waypoints,
GraspGenX, object resets, teleports or object-joint commands are permitted.
Qualification uses a short forward/reverse chain, measured simultaneous base
and arm motion, combined TCP tracking, original native collision/force/speed
guards, and actual release/retreat/parking. It is not stitched to a formal case.
Qualification must show base contribution exceeding arm contribution along
the task translation, without a hard safety event, before long-range use.

The physical base is the existing driven XY carriage: fixed height/yaw,
native +/-1 m axes and 0.08 m/s velocity caps. This is not a wheeled dynamics
or hardware safety demonstration. A single nine-variable bounded kinematic
solve generates XY and arm targets before the same physics step. A nonlinear
world-TCP governor checks their combined velocity and acceleration. Arm
posture retention is weighted by normalized joint margins; allocation is not
an asset-specific displacement or compensation cap. Existing native drives,
object dynamics, grasp ranking, force/torque protections, reference lead,
startup, contact feedback and estimator coefficients remain unchanged.

Loaded motion certificates include the full robot, gripper, pedestal and
moving object, measured command bias, all three existing centering extrema,
both relative-lead extremes, reversal and the braking envelope. Current-state
successor proofs replace fixed-station reachability proofs. Future final-goal
parking is deferred to the actual stopped state, using existing staged exit.
Planning success is never counted as physical manipulation success.

Record native base position/velocity, configuration-derived base velocity and
acceleration, base command/tracking error, arm compensation, joint margins,
world TCP command/measurement, handle-relative slip, actual pad loads/aperture,
six-axis wrist wrench, all object DOFs, collisions, actions and wall waits.
Keep raw telemetry and continuous video private and save them before cleanup.

Six-axis F/T identification keeps the existing segment-local measured world
sensor/body transforms and Newton–Euler tool acceleration compensation. Base
motion must not be attributed to object inertia. FIT and complete independent
VALIDATION remain separate; failure to identify does not block safe motion.
Fresh hidden plant draws from the existing shared family are committed
privately. Seal both online traces/models after both actors close, then reveal
dynamics solely for post-hoc parameter-error evaluation.

The previous fixed-base pair is a descriptive baseline: 45600 reached 32.185
degrees with 2 grasps; 45290 reached 136.111 mm with 6 grasps. Its base moves
were unloaded relocations. Both lacked reverse/independent validation. The
prior drawer's final parking confirmation was interrupted at its deadline.
New stochastic plants and a single pair do not establish a causal population
comparison. Report actual single-grasp range, coordination allocation,
failures/regrasps/recoveries, independent ID and final safety state for each.

No additional assets or indefinite tuning follow these two formal outcomes.

Startup resource migration: the first qualification instance failed the SDK's
remote root-directory lookup before any observation or contact. Preserve that
failed startup and its cleanup. A subsequent opt-in local mirror must use the
exact official SDK-version ground USD and original texture bytes, verify all
manifest hashes before launch, and require the SDK's original root URI to
match. Only the resource root path is relocated; the original SDK constructor,
USD values, geometry, materials and physics are unchanged.
