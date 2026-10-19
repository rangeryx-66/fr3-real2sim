# Interactive articulated twin benchmark

This is an independent experiment on `codex/piper-mobile-door`. The physical-contact baseline at `f5dcc6f` and the EE-only interaction loop at `cd616606415bae9176f7545dd3e071f5e658fcbe` remain unchanged. New execution code checks the frozen baseline files before starting Isaac.

## Run

On the lab server, from `/data1/home/rangeryx/fr3_real2sim_piper_mobile`:

```bash
env -u PYTHONPATH /data1/home/rangeryx/.conda/envs/anygrasp/bin/python \
  scripts/run_interactive_twin_benchmark.py \
  --config configs/interactive_twin.yaml --stage full
```

Stages: `capability`, `kinematics`, `sensitivity`, `physics-fit`, `heldout`, `full`.
MODE B accepts a verified hardware log bundle using `mode: REAL_LOG_TO_SIM` and `--real-log bundle.json`; see [the bundle contract](interactive_twin_real_log.md). It never opens a hardware connection and rejects missing robot-only calibration or servo mapping evidence. No real logs are included in this experiment.

The orchestrator launches the configured Isaac Python for each independent plant. The configuration fixes the output directory, source assets, seed, deployment rule, candidate/probe/rollout budgets, GPU assignments and Shanghai cutoff. A frozen manifest must not be edited after TEST starts. Use a new output/version for a changed method and rerun the same TEST set.

## Data and stage boundaries

- Budgets are frozen at 12 grasp candidates and 4 probe directions per configuration, 16 system-identification launches per configuration, 48 per asset shared by three configurations, and 25,200 active rollout seconds per asset. Every process also obeys the global Shanghai cutoff.
- DEV is asset 7320, excluded from unseen results. TEST selection uses prepared visual geometry and asset metadata before execution. Failed selected episodes stay in the denominator.
- A–D: prepare visual handle/proxy, plan bounded bar-side-pinch candidates, establish actual contact, run the frozen constrained probe, fit measured EE poses and save the estimate.
- E–F: robot-only response calibration, repeated LOW/MEDIUM/HIGH response sensitivity, and bounded independent twin rollouts. P1/P2/P3 train; P4 is held out.
- G: save T0 initial prior, T1 EE kinematics update and T2 accepted physics update (or unchanged prior plus uncertainty if identification fails).
- H: freeze estimates before independently replaying the held-out action under each twin.
- I: continue physical interaction using the saved estimated articulation. Physics prediction improvement and manipulation success are separate claims; the frozen controller does not automatically become a new physics-optimized policy.

Physics grid/held-out rollouts are independent fresh episodes with the complete recorded approach/closure/probe command history. Final manipulation re-establishes the same grasp and loads the saved estimate. This is an experiment-level closed loop with documented resets, not a claim that the simulator remains continuously running during offline optimization.

The controller never receives reference object type, hinge axis/origin, object joint state or moving-link trajectory. Scene assembly necessarily imports the asset before the online boundary. GT evaluation occurs only after observations and estimates have been saved and simulation paused.

## Command replay

For constrained motion, the common input `u(t)` is the timed Cartesian drive reference, direction and active/hold state, plus gripper commands. Each plant runs the same frozen robot controller using its own robot encoders, Jacobian and robot dynamics. Its feedback-generated motor efforts are diagnostic output, not a reference trajectory to impose on the twin. Position commands used for approach are actual controller commands, never measured `q(t)` state assignments.

Early DEV diagnostic runs also tested literal motor-effort replay. Such replay freezes the reference trajectory's gravity compensation and can become unsafe when the object response changes. Those failures are retained; they are not evidence of identifiable hinge parameters. Neither replay method sets observed robot or door states at each step.

Native passive resistance is authored once before simulation. `tau_c` uses equal native static/dynamic friction effort; `b` is configured in N·m·s/rad and converted to the native angular USD unit. No script applies torque to the object during execution. The reference geometry, inertia and fixture are preserved in the twins; only the designated kinematics and passive resistance fields change.

## Identifiability and held-out rules

- Motion/noise normalization uses a frozen robot-only repeat report and declared quantization floors. Repeatability does not establish real sensor accuracy.
- A complete training segment remains valid if a later held-out segment fails. Incomplete segments are never labeled complete by the final episode status.
- Missing or unsafe grid rollouts are censored evidence, not silently removed parameter regions.
- A diagnostic best parameter is not automatically accepted. Intervals are discrete grid support, not statistical 95% confidence intervals.
- Best residual relative to the noise scale is reported to expose model mismatch, including inaccurate kinematic estimates absorbing resistance error.
- T0 inherits dataset articulation, which can already be accurate. No assumption is made that T1 must improve it.
- B0–B3 are internal ablations, not reproduced official Act2See benchmark results. Oracle is an evaluation diagnostic only.

## Hardware and scientific limits

The current runs are **SIM_TO_SIM_BLIND_SYSID**, not real-to-sim hardware validation. Stock PiPER feedback does not provide bilateral fingertip loads, a tactile array or a calibrated six-axis wrist force sensor. SDK effort is current-derived and must not be called external torque. B3 remains unavailable without independent calibration.

The frozen simulation contact supervisor uses native contact reports to enforce safety and retention. Identification does not consume them, but this supervisor still needs a hardware replacement. Tangential slip is not fully observable online from the available stock signals. Post-run ground-truth slip metrics are evaluation only.

Interaction proxies are approximate manipulation geometry. No triangle-level fidelity, new shape reconstruction, full joint-limit recovery or measured real hinge-friction value is claimed. Only the actually observed articulation range is recorded. Final benchmark summaries must distinguish successful stages, blocked stages, not-run stages and failed episodes.
