# REAL_LOG_TO_SIM replay adapter

This adapter runs independent **Isaac rollouts from already collected real logs**. It neither opens CAN nor sends commands to a robot. Its native execution path is `run_real_log_bundle(..., run_job)` → the existing episode runner in `mode=replay`. A missing bundle or independent servo calibration is a `BLOCKED_BUNDLE_EVIDENCE` result, not a simulated substitute for real data.

```bash
python scripts/run_interactive_twin_real_log.py \
  --bundle /path/to/real_episode/bundle.json \
  --output results/real_log_twin/episode_01 \
  --runtime-config configs/interactive_twin.yaml
```

Use `--validate-only` to check evidence without launching Isaac. Runtime overrides are `--isaac-python`, `--gpu`, and `--deadline-shanghai`. No real-log collection or live hardware controller is included.

## Supported command contract

The first version accepts only the frozen **240 Hz Cartesian constrained-drive** contract. It does not silently resample a lower-rate SDK log, invent motor torque from current, or convert measured robot states into commands. A hardware integration must independently validate any rate/servo conversion before producing this bundle.

The full command tape is a JSON list, beginning at `t=0`, with every step at 1/240 s. Each frame contains exactly:

- `t`, `phase`, `compliant`, `retention_armed`;
- `arm_position` and `arm_velocity`: six actual issued joint references, in rad and rad/s;
- `finger_mode`: `position` or `effort`;
- `finger_position`: two actual issued references in meters;
- `finger_effort`: independently mapped simulator-equivalent command, within the frozen limit;
- `cartesian_input`: null in position mode; otherwise `{reference_world_m, direction_world, active}`.

`arm_position` is an issued reference, **never a replay of measured q(t)**. For compliant frames, the native controller computes its own feedback effort from the twin's encoders and robot dynamics. Raw SDK effort, current, measured q, and force-manifold fields are rejected as tape inputs. The first 120 frames describe the initial settle; the tape must also include approach, actual grasp/retention history, and all four declared probes. The adapter verifies that each observable probe's command hash exactly matches the corresponding full tape segment.

## Bundle schema

All file paths below are relative to the bundle unless absolute.

```json
{
  "schema": "interactive-twin-real-log-bundle-v1",
  "mode": "REAL_LOG_TO_SIM",
  "command_tape": "commands.json",
  "reference_logs": {"P1":"P1.json", "P2":"P2.json", "P3":"P3.json", "P4":"P4.json"},
  "robot_calibration": "robot_calibration.json",
  "servo_mapping": "servo_mapping.json",
  "job_template": "native_scene_job.json",
  "initial_scene": "initial_scene.json",
  "estimated_articulation": "estimated_articulation.json",
  "structured_memory": "structured_memory.json",
  "estimate_support_scope": "pre_physics_EE_only",
  "physics_protocol": [
    {"probe_id":"P1", "split":"train", "duration_s":8, "speed_m_s":0.00025, "pattern":"opening"},
    {"probe_id":"P2", "split":"train", "duration_s":8, "speed_m_s":0.0005, "pattern":"opening"},
    {"probe_id":"P3", "split":"train", "duration_s":10, "speed_m_s":0.0005, "pattern":"stop_restart"},
    {"probe_id":"P4", "split":"heldout", "duration_s":8, "speed_m_s":0.000375, "pattern":"short_pulse"}
  ],
  "grid": {"tau_c":[0,0.004,0.012], "b":[0,0.3,1.0]},
  "initial_physics_prior": {"tau_c":0, "b":0},
  "budget": {"maximum_candidates":9, "simulation_launches":12, "wall_clock_s":25200}
}
```

The real P1–P4 files use the observable schema in `interactive_twin.sysid`: source `real_robot_log`, mode `REAL_LOG_TO_SIM`, complete timestamped commands and q/qdot/EE/gripper observations. Original simulator logs cannot be relabeled as real. Current/SDK effort may be logged but is not used by this first adapter.

The job template supplies existing native `source`, `asset_root`, `plan`, and optional safe runtime fields. It cannot supply object actuation, a plant resistance configuration, a replay tape, controller-memory overrides, or unverified initialization. The initial scene file supplies:

- `source: "measured_prepared_initial_scene"`;
- `joint_coordinate_zero_is_initial: true` — the prepared prior is already re-zeroed at the observed initial configuration;
- `robot_q_initial_verified: true` and six `robot_q_initial_rad` values, applied **once at initialization**;
- `T_world_asset` and the frozen measured `fixture` description.

The saved EE estimate must be a reliable revolute estimate (`confidence > 0.9`) with at least 5 mm of measured EE excitation. Memory must declare `GT_inputs: false`, preserve supporting observations, accepted fit history, and a reliable-estimate attempt ending before P1. Physics and held-out observations do not become kinematic fitting support.

## Independent servo calibration gate

A noise report must come from at least two no-contact real robot repeats, with `hardware_calibration: true`. That report alone is insufficient: repeat noise is not servo transfer calibration.

`servo_mapping.json` must provide:

- exact robot/controller IDs and a verified hardware firmware version;
- `independently_calibrated: true`, `frozen_before_object_data: true`, `object_contact_data_used: false`;
- `q_measurements_used_as_commands: false`, `current_or_sdk_effort_to_external_torque: false`;
- `command_kind: "cartesian_constrained_drive"`, `sample_rate_hz: 240`;
- `native_servo_sha256` matching `current_servo_hashes()`;
- a **predeclared** `predeclared_maximum_motion_loss`;
- at least two `free_space_validation_pairs`, each containing `real_log`, `sim_log`, and both file SHA256 values.

The adapter independently recomputes trajectory/onset/velocity/tracking losses for those real/sim no-contact pairs against the frozen noise scales. Merely marking a mapping “verified” does not bypass the response check. This is evidence validation, not proof of the authenticity or quality of hardware data; inspect original acquisition records before making a research claim.

Optional `sensitivity_logs` maps condition names to at least two real repeat logs per condition. Sensitivity is recomputed from those traces. Without independent sensitivity evidence, candidate rollouts can produce diagnostics and B0/B1 held-out comparisons, but no physics update is accepted. B3 is unavailable because current-to-simulator transfer calibration is not implemented. No real-data Oracle is invented.

## What is executed and saved

1. Validate the original real logs, initial scene, independent robot response mapping, command contract and fixed budget.
2. Save T0/T1; run each candidate under the same full approach and **P1–P3 prefix only**.
3. Fit motion responses using the full planned grid; missing/unsafe candidates remain censored.
4. Save the frozen fit and T2 (or unmodified T1 physics if unidentifiable).
5. Run independent B0/B1/B2 full-tape rollouts; evaluate P4 only after parameter selection is fixed.

The simulator uses its native contact/collision supervisor. Real logs do **not** require simulator contact manifolds or separate left/right pad-force signals. This does not establish a deployable hardware slip/contact supervisor.

The adapter writes the bundle identity, calibration validation, rollout ledger, individual native jobs/reports, physics fit, T0/T1/T2, held-out comparison, and final status. It records measured held-out improvement separately from any real-to-sim success claim. No real-data run has been claimed by implementing or unit-testing this interface.
