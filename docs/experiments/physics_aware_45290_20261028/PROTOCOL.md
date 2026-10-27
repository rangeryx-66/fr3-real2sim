# Physics-aware drawer study: staged causal test

Research question: does updated measured physics improve held-out manipulation
relative to normal reactive control and a stale model? Negative results are
valid. Asset 45290 only; one prismatic joint; xArm7/G1 and simultaneous native
XY/arm control from 168206f. No Artisan, new assets, simulator resets within
recovery, hard-limit changes or unbounded framework work.

## Gates before physical comparisons

1. Audit observed F/T signal, parameter sensitivity, original motion bounds and
   actuator/load mapping. Offline demand sensitivity is a theoretical check,
   not evidence of real slip or an identification result.
2. Verify native friction material bindings and combine modes on actual pad
   and handle contacts. Read back passive rail friction/damping and moving-body
   masses. These private plant records never enter policy inputs.
3. Qualify the prospective coordinated braking correction with a finite short
   loaded forward/reverse and original safe exit. Preserve any failure/latch.
4. Obtain one fresh, automatic, unexecuted observed grasp reference for this
   controlled study; no archived successful pose. Apply identical geometry and
   native admission to each method. Contact preload calibration reports request,
   measured two-pad normals, saturation and six-axis F/T separately.
5. Verify actual relative slip and transmission response under bounded load;
   no rule mapping a plant coefficient or request directly to a slip outcome.
6. If policies collapse to the same outputs or contact/ID has no useful
   sensitivity, report that gate and do not launch long operations.

## Comparison design

Register changed conditions and method order before observing their outcomes.
Use the same physical condition, initial state, sensing, gripper authority,
coordinated controller and independent safety owner across A/B/C. Simulator
instances are independent across comparisons, never stitched within a trial.

- A: strong reactive baseline, normal F/T/reference/contact feedback, measured
  grip calibration and recovery. No explicit mass/damping regression.
- B: same action selector and abilities, model M0 from initial-condition FIT;
  no new dynamics fit after the controlled plant change.
- C: finite active probes, original six-axis F/T estimator, measured M1 with
  uncertainty, then selection of preload/speed/acceleration.
- D: offline oracle prediction with the same feasible control menu; no GT
  values in A/B/C inputs. Oracle is not a measured manipulation upper bound.

To distinguish model knowledge from extra contact practice, include a matched
probe-exposure reactive control when short comparisons have useful effects.
Report both ordinary A cost and probe-matched A cost. Keep all failed recovery,
unloading/regrasp and probe time in total task cost.

The selector maximizes predicted throughput subject to conservative empirical
contact capacity, then minimizes reliable preload among equally fast actions.
Also report the preload/time Pareto set. There is no invented rigid-handle
damage, squeeze energy or high-preload penalty. If maximal legal preload plus
normal feedback already performs equally well, explicitly report no operational
benefit; smaller preload alone is a distinct, limited finding.

Only FIT informs model/selection. Declare independent targets and profiles
before execution. Match actual velocity or completion-time strata for jerk,
impact and fluctuation comparisons; slowdown alone is not a smoothness gain.
Short comparisons precede any 180 mm extension. Exact condition/repetition
counts are frozen after sensitivity/contact calibration and before comparisons.

## Evidence

Per episode retain video, raw 240 Hz contact/motion/F/T, action decisions and
model inputs/outputs, source SHA, plant-side readback, GT separation and actual
release/retreat/parking/latch. Close every finished/abandoned owned actor.
Report Q1–Q4, success/slip/regrasp counts, relative slip, actual normal loads,
F/T peaks/variation, q/qd/qdd/jerk, task/probe/wait wall time, XY/arm allocation,
parameter support/error and independent predictions. Small samples cannot
establish population significance or hardware safety.

PhysX combines the two contacting materials according to their combine modes;
changing only one authored coefficient is not proof of the effective pair value.
Reference: [NVIDIA PhysX combine-mode documentation](https://nvidia-omniverse.github.io/PhysX/physx/latest/_api_build/structPxCombineMode.html).
