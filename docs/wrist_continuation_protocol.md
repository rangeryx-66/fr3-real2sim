# Checkpoint-command continuation protocol

This extends `15e6384` without changing official robot/gripper geometry, object
physics, collision acceptance, closure force/gains or the 0.05-rad joint bound.
The run finishes as soon as the requested usable range and captures are obtained;
wall budget is a ceiling, not a requested waiting duration.

## Contact-load policy

- Original 2 N becomes a raw-sample warning. Stop task drive, settle, validate the
  existing bilateral window, and resume at lower requested reference speed only
  after load is normal and grasp remains stable.
- Mean load >2 N for three consecutive 50-ms windows stops the current grasp.
- Dangerous body/root/environment collision and genuine instability stay hard
  stops independently of the load average.
- 10 N is an immediate **SIMULATION-ONLY contact-reaction emergency proxy**,
  derived from the existing model's 10-N actuator effort ceiling. Commanded
  gripper force is NOT increased. This is not a manufacturer-certified contact
  safety cap and must not be deployed on real hardware without calibration.
- Official SDK gripper effort is described as torque in 0.001 N·m; it is not a
  calibrated finger contact-force measurement. See the
  [official SDK interface](https://github.com/agilexrobotics/piper_sdk/blob/master/asserts/V2/INTERFACE_V2.MD).
- Spike/impact/regrasp/settle/recontact samples are effort-invalid, not automatically
  manipulation failures. Raw samples and stop events remain retained.

## Recovering the old physical progress

`run_wrist_continuation.py` reconstructs the actual **issued** robot commands and
original base-route interpolation from the preserved experiments. A new simulator
initializes the original scene once and executes those commands through physics.
It never assigns measured robot q, object joint position/velocity, object pose,
contact impulses, attachment or external object force. Native contact establishes
and validates grasps again. A failure to reproduce contact is recorded; restoration
is not presumed successful from a JSON file.

The restoration prefix uses the recorded actuator effort commands, including
the compliant phases. It does not call the input-response system-identification
replay's feedback-torque recomputation branch. That branch introduced small
robot-response differences that changed subsequent closure contacts. After the
prefix finishes, the original measured-state feedback controller resumes. This
distinction is confined to checkpoint restoration, not physics identification.

The old actual maxima (24.878 degrees and 5.88 mm) are provenance/evaluation only.
Online continuation uses saved sensor-estimated state and fresh RGB-D/robot EE
observations. Actual restored progress is independently evaluated after stopping.

After replay, the existing runtime resumes compliant motion or performs protected
release/regrasp. The existing model memory and successful relative grasp template
are reused. Cartesian and joint-space clearance share the same finite alternative
budget; home is optional.

## Run

```bash
/data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_wrist_continuation.py \
  --config configs/wrist_reconstruction_extended.json \
  --source-root results/wrist_maximum_20261006 \
  --output results/wrist_checkpoint_continued_20261006
```

`run_clock.json` persists the ceiling across supervisor restarts. Never overwrite
an active actor. During running, use raw `observations.jsonl` tails and component
files; no final experiment verdict is generated from a temporary blocker.

Reconstruction requires three clean states per object, each at least eight real
wrist views: closed/at least30/at least60 degrees and closed/at least5/at least10 cm.
No ArtGS training is allowed before both objects satisfy that gate. Existing
state observations retain acquisition provenance and original camera poses.
