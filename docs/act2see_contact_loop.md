# EE-only Act2See-style physical-contact interaction

This independent experiment preserves the `f5dcc6f` known-model contact baseline.
The original runner, gripper/contact implementation, nominal grasp, fixed base,
proxy, friction, closing preload, force limits and 0.05 rad joint-margin gate are
unchanged. It replaces **only post-grasp interaction** with a constrained probe.

## Public method and scope

The [public Act2See description](https://lauyihong.github.io/act2see/) describes
observation + structured model + attempt history, probe/execute/skip, grasp and
constrained motion, estimation from EE trajectories, and model updates. At the
2026-10-03 inspection, its [linked repository](https://github.com/lauyihong/Act2See)
contains the project webpage, not released robot controller code. This is a
small implementation of that published interaction loop, not a claim to run the
authors' unreleased controller. No VLM planner is included.

## Run on the experiment server

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
env -u PYTHONPATH -u CUDA_VISIBLE_DEVICES \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_piper_act2see_loop.py \
  --source /data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb \
  --asset-root results/semantic_interaction/asset \
  --plan results/semantic_interaction/nominal_minimal/plan.json \
  --output results/act2see_contact_loop --gpu 1 --repeat 3
```

The default cutoff is the next 05:00 Asia/Shanghai. A successful first trial is
followed by three repetitions and a zero-probe control. A failed first trial is
reported and followed by a zero-probe control; it is not counted as a successful
opening. `--trial` runs one process; `--trial --no-operation` runs the control.
Use a GPU with adequate free memory; GPU selection does not change the scene.

## Data boundary

- Initial grasp and collision scene are the frozen baseline inputs.
- Runtime control uses robot q, encoder-difference qdot, measured TCP SE(3), contact loads and contact
  patches in the TCP frame. RGB video is recorded; initial RGB-D remains part of
  the frozen scene inputs. PnP/ICP is not instantiated.
- Robot Jacobian, mass matrix, gravity and Coriolis compensation concern the known
  robot, not the unknown object. The world Jacobian is checked against robot FK
  float64 finite differences before enabling the probe. PhysX Jacobians are shifted
  from the actual body COM to the TCP, not from the URDF link origin.
- Simulator object articulation, joint chain and moving-link pose handles are
  access-guarded until simulation has paused and the EE-only estimate is saved.
- Fitting receives measured EE transforms only. No object joint state, type, axis,
  origin or measured moving-link trajectory enters control or fitting.
- The moving collision body's online transform is predicted from EE motion under
  the retained-grasp assumption. Actual PhysX dangerous-contact checks remain
  active every physics step. Prediction is not presented as measured object pose.

## Short constrained probe

The frozen approach, slow closure and initial force hold run first. Only then do
arm joint position springs switch to robot dynamics compensation plus `J.T @ wrench`.
Finger effort and damping remain unchanged.

- Drive speed: 0.5 mm/s with a 2 s ramp.
- Position stiffness along the current probe direction: 150 N/m.
- Orthogonal translation and rotation stiffness: zero.
- Damping: 100 N s/m along drive, 12 orthogonal, 0.04 N m s/rad rotation.
- Task force/moment caps: 0.20 N / 0.01 N m; original grasp load and robot effort
  limits additionally apply. An implicit damping solve handles the 1/240 s step.
- Extra TCP overspeed stop: 5 mm/s, evaluated from measured EE pose differences.
  Native velocity and encoder-difference velocity are both logged. Original robot
  speed limits are retained.
- At most four directions from the initial measured grasp frame: outward,
  outward ± 0.25 lateral, lateral. There is no grasp search.
- A safe attempt with <2 mm travel after 12 s returns to a non-driving compliant
  hold before the next direction. Each attempt has a 35 s budget; all probes share a 20 mm excitation budget.
- Fitting begins only after 5 mm measured displacement and enough samples.
  The revolute acceptance gate requires score-gap confidence >0.9, position RMSE
  <0.3 mm, rotation RMSE <0.001 rad and rotation span >1 degree.
- An accepted estimate becomes `structured_memory.json` and an estimated URDF.
  Following uses its current tangent, refreshed every 1 mm while preserving
  the bounded compliant lead and velocity ramp, with the same compliant primitive. Every 2 s, new EE samples refine the model. Estimated 5.5 degrees is
  the online stopping target; actual >=5 degrees is checked only after stopping.

## Safety and measurement limits

The frozen self/environment contact acceptance and load limits are retained.
Margin <=0.05 rad, sustained bilateral-load loss, excessive slip proxy, effort/
velocity violations or dangerous contacts stop the experiment. No raw triangle,
micron ownership or pad-triangle gate is reintroduced.

With GT moving-link motion and PnP excluded, exact rigid-object relative slip is
not fully observable. A force-weighted pressure center was rejected in testing:
load can transfer between corners without material slipping. The online signal
therefore uses **contact-plane normal drift and tilt**, at the same 1 mm / 0.1 s
gate, plus the unchanged bilateral load, opening and dangerous-contact checks.
The native contact manifold is a simulated tactile/proximity observation. It does
not provide a measured maximum in-plane slip; after identification, EE position
versus estimated screw-motion consistency also uses the existing 1 mm / 0.1 s stop.
This additional residual detects model-inconsistent motion without GT. The maximum
true relative slip remains explicitly `null`; neither signal supplies that ground truth.
Hardware would need sufficiently rich tactile/slip sensing to reproduce this
observer. Final true relative transform drift is computed only after pausing.
This observability limitation must be disclosed with any physical-opening result;
it is not a claim of fully validated maximum-slip protection on a real robot.

The zero-probe control uses the same compliance with zero drive. EE fitting starts
after the passive controller transition, so initial elastic alignment is logged
separately instead of contaminating the articulation fit.

No attachment, weld, object external force, or execution-time object state command
is issued. Scene assembly uses the existing initial-state loader.

## Artifacts

Each run saves continuous video, physics contact records, measured EE/q/wrench
traces, attempt history, every fit, structured memory, optional estimated URDF,
and final GT-only evaluation. Actual results belong in the dated experiment
report; the software checks alone do not establish interaction success.

`python scripts/evaluate_piper_act2see_contacts.py RUN_DIRECTORY` performs an
optional **post-run** diagnostic: it reconstructs motion from recorded contact
normals using the now-permitted GT axis/origin. Its estimated slip peak is labeled
as a reconstruction, and the final angle is checked against simulator GT. It is
not an online observer or a directly logged GT trajectory.

## Verified experiment

[2026-10-03: four physical openings and a zero-probe control](experiments/piper_act2see_contact_20261003/README.md)
contains actual phase-by-phase results, continuous video, measured traces, the
failed engineering trials, and the remaining full-slip observability limitation.
