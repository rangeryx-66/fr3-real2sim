# Large-range interaction and torque identification V1 — not achieved

**Neither physical objective was achieved.** The general reference-handover repair passed archived/offline tests, but the single registered live scene stopped before grasp acquisition because the existing visual input Skill did not detect the new object's short protruding lip. No opening/closing interaction or object-parameter fit occurred. This is a pre-grasp perception failure, not a demonstrated mechanical limit or a physical validation of the control repair.

Scope: **AGENT_WRIST_RGBD / KNOWN_HINGE_STATE / IDEALIZED_WRIST_LOAD_CELL / APPROXIMATE_ACTUATOR_SIMULATION**. Pre-outcome source/protocol freeze: **a128599**, based on eeda9fa. The previous frozen manipulation/sensor/recovery sources remain unchanged.

## Control repair and offline evidence

The archived LOCAL_CENTERING→TRANSMISSION_PROBE jump was reproduced from actual joint commands, not the stale planned-pose field. Centering left a virtual Cartesian target inconsistent with the newly measured IK seed. Pink immediately corrected that residual while the equilibrium bias was added to the command.

The isolated version rebases the virtual reference to FK of the same measured seed while preserving the current joint position command, equilibrium bias and initial slip reference. It applies the same entry convention to subsequent interaction segments. The Pink solver, closure, actuators, physical parameters and original protections are unchanged. No new runtime quality assertion or force requirement was added.

| Offline check | Result |
| --- | --- |
| Archived original first-step commanded FK speed | 35.194251 mm/s, reproduced in the runtime environment |
| Corrected opening / closing first step | 0.006253 / 0.006253 mm/s |
| Corrected stationary first step | Approximately zero |
| Preserved joint position command | Exact |
| FK/Jacobian numerical discrepancy | 9.87e-10 |
| Separate ideal-follower opening/closing command cycles | Peak 1.875799 mm/s; stationary cycles approximately zero |
| Five-term synthetic estimator check | Rank5; maximum coefficient error1.26e-7 |

These are **offline command/algebra tests**, not loaded tracking, contact-mode or physical-identification results. The original measured loaded-TCP limit remains5 mm/s; the repair has not yet been exercised under contact.

## One registered live scene

Object **47686** is an existing horizontal upper-hinge cabinet, authored120° range, world hinge axis(-1,0,0). The0.70m preparation follows source height metadata. A0.25m import was only an earlier geometry-inspection draft, replaced before grasp planning or physics. A single independently seeded constant-resistance configuration was registered; hidden dynamics remained sealed during actions. The original registered station was reused.

Offline preparation established a collision-free initial observation route. **It did not establish a feasible visual grasp or loaded15–20° path.** This distinction matters: metadata/hinge eligibility alone did not qualify this object for the current visual grasp Skill.

Codex inspected all five fresh wrist RGB/mask/depth/point-cloud packets. Four requested observation adjustments passed the existing IK/collision checks and executed through normal actuators: wider right-shifted framing, closer elevated framing, left-oblique inspection and a final closer oblique view. The lip became clearly visible, but no handle mask passed the unchanged0.25 detection threshold.

| Wrist view | Highest raw score | Accepted handle geometry |
| --- | ---: | --- |
| Initial | 0.005707 full frame;0.014954 bounded observed-depth crop | No |
| Wider framing | 0.024536 | No |
| Closer elevated view | 0.045410 | No |
| Left oblique | 0.073730 | No |
| Final close oblique | 0.088379 | No |

The builder returned `NO_ACCEPTED_COMPLETE_MASK`. Scores were removed at confidence filtering, before contact ranking or grasp IK. No manual mask, lowered threshold, changed prompt, GT handle surface, saved grasp or GT pose correction was substituted. The observation budget ended; the object was not replaced. The evidence does not isolate whether appearance, semantic mismatch or detector generalization dominates, and does not prove that the lip is physically ungraspable.

| Physical / evidence metric | Actual result |
| --- | --- |
| Continuous scenes / valid door-contact episodes | 1 /0 |
| Physical grasp acquisitions / releases / regrasps | 0 /0 /0 |
| Useful maximum angle; opening / closing travel | 0°;0° /0° |
| Handle pull / panel push / regrasp modes | Not executed |
| Original force, collision or loaded-speed stop | None; loaded-speed behavior not evaluated |
| Peak pad normal report | 0 N |
| Minimum joint margin, unloaded | 0.702114 rad |
| Maximum measured TCP speed, unloaded | 29.995777 mm/s, under the existing free-motion trajectory |
| Simulation / runner wall time | 121.6875 s /1382.8098 s (23.05 min) |
| Native observation / physics records | 29,205 /29,205 at240 Hz |
| Valid wrist F/T records | 29,205 /29,205 |
| Continuous video | 1280×960 H.264,3650 frames,121.667 s |

The tiny raw angle variation (~1e-9° accumulated) is numerical settling, not useful interaction. Inherited parent report fields still display a nominal3° short-task value; no such action was issued. This round's registered exploration objective was15–20°, conditionally30°. None was executed.

## Torque identification and termination

Implemented a measurement-only model `tau=I*qdd+b*qd+C*sign(qd)+Gs*sin(q)+Gc*cos(q)`, with existing bias/tool gravity/tool inertia compensation, full six-axis moment transformation, segment-local processing and independent validation labels. Complete segment samples are retained independently and workers do not delay loaded execution. The new nonvertical axis permits gravity excitation in principle; actual object excitation was zero.

**Inertia, Coulomb/viscous friction and gravity coefficients: not estimated. Parameter uncertainty, parameter errors, held-out torque error and independent angle/velocity prediction error: unavailable.** Interaction segments, model updates and held-out trajectories were all zero. Raw/compensated unloaded wrist traces are preserved only as sensor diagnostics; tool motion is not fitted as door resistance. The synthetic test is not physical accuracy evidence. No complete physical twin is claimed.

The runner ended from the last verified unloaded, collision-checked observation configuration, with zero pad loads. No grasp required releasing. The guarded parking primitive was not invoked; **SAFE_PARK_REACHED is not claimed**. The actor and encoder exited after complete export. No initialization crash, scene restart, stitching, object actuation or physical retry occurred. A nonfatal invalid-background-depth arithmetic warning was preserved; it did not cause the detection rejection.

All477 archived evidence files (150,435,175 bytes) were checksum-verified privately. Manifest SHA256: `51bf143aeab629f3af3862186ccec080a0325dfb2920fd0779818e92a23398ba`. All72 frozen runtime source hashes and the preceding frozen source set remained unchanged. Raw RGB-D, video, telemetry, hidden configuration/truth and detailed evidence indexes remain private.

The next prerequisite is an offline visual/grasp qualification of a nonvertical-hinge object before registering another expensive interaction episode. The present handover repair still needs one real-contact test; it must not be labeled physically validated. No additional case, detector tuning, controller adjustment or experiment was started after this bounded scene.
