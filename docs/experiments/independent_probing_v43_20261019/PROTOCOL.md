# V4.3 pre-registration

One fresh continuous Agent-guided 45936 research episode; at most the existing one recovery regrasp. References: 5fa0f98 runtime, 0305095 estimator, a2ea768 previous outcome. No engineering rehearsal or recorded successful grasp/view input.

## Blind configuration

Generate one new independent secret seed and physically consistent mass/COM/inertia, Coulomb friction and damping from the established `xarm_ft_id_scene.PRIORS` family. Constant resistance, vertical cabinet hinge; no nonzero gravity-torque claim. Freeze private JSON read-only and commit its SHA256 before launch. The scene generator must read it to author physics. Agent/controller/estimator observation interfaces receive no truth; the Agent will not open private job, hidden configuration/authored-physics/audit files or object mass APIs during online decisions. This is interface-level scientific blinding, not adversarial OS isolation from a simulator that must instantiate the parameters. Truth is evaluated only after actor exit and all online decisions finish.

## Runtime and bounded execution

Actual entry: `scripts/run_informative_probing_v31.py`, RuntimeV3, OperationsV31, unchanged V4.2 ProgressClock/adaptive segments and unchanged V3.1 mathematics. Freeze/hash active code before launch. Runtime: 3000 wall seconds including 300 reserved for guarded exit; 1500 simulation seconds including 90 reserved. Outer launcher: 3300 wall seconds, providing separate export/shutdown allowance. Agent waits bounded to 300 seconds at unloaded boundaries. Target approximately 30–40 minutes; resource ceilings are not segment timeouts.

Use shared GPU capacity, one owned actor. FAST_DEV essential 240 Hz numeric data, checkpoint wrist RGB-D, bounded event clips, target <100 MB; no duplicate raw archive/public media upload.

## Agent decisions and segment priority

Inspect fresh wrist RGB, masks, metric depth and measured clouds, then decide up to four useful view adjustments. Automatically rank and plan from measured incremental surfaces. Inspect the fresh grasp before approving execution. Known hinge/state, station, native scene collision safety, simulated camera mounting and approximate actuators remain explicit.

Choose endpoints after observing actual grasp/IK opportunity. Prefer an acceleration-rich opening FIT, overlapping different-speed reverse FIT, then a distinct-profile complete VALIDATION inside the physically supported region. Optional larger-angle FIT toward 10–15° comes **only after** complete VALIDATION. No fixed 13° target. Preapprove conditional successors while stable and loaded; otherwise use original guarded exit before Agent reasoning. A partial result remains partial; a transmission/safety failure cannot continue loaded.

Motion family starts from `configs/probing_v42_plan.json`: plateau FIT (.012 m/s, .04 m/s², low fraction .125, .45 s ramp/.55 s plateau), reverse FIT (different attainable speed), validation sine (.008 m/s, .03 m/s², low fraction .35, 3.1 s period). Each actual target, role, speed profile and interrupted-FIT intervals are declared atomically before execution. No max_sim_s=45 default. No hold shortening, force/physics change or GT pose correction.

## Identification and assessment

Primary measurement: compensated robot-side six-axis wrist F/T projected with the explicit known hinge prior; tau = I*qdd + b*qd + C*sign(qd). Production SG121 common filtering and single downstream-tool inertia compensation unchanged. FIT only for parameters/uncertainty; segment-local derivatives and complete held-out VALIDATION. Predeclared, transmitting interrupted FIT prefixes may qualify; incomplete VALIDATION always excluded and N/A.

Verify asynchronous updates from this episode. Report actual angle/speed/acceleration excitation, regressor conditioning, uncertainties, complete held-out torque RMSE, once-initialized free forward angle/velocity predictions (no future-state correction), and post-exit parameter errors. Authored inertia is not an independently verified effective PhysX inertia. Preserve online model bytes; optional frozen V3 comparison uses identical eligible raw intervals, never retunes V3.1.

PASS requires complete independent physical VALIDATION and credible held-out dynamics prediction with supported parameter interpretation. Large motion, fitting alone or safe parking alone is insufficient. Retain all physical failures; do not retry or substitute an easier configuration. Initialization/interface-invalid failures may be repaired with evidence and without claiming a physical outcome.
