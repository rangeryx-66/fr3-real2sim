# Maximum-range wrist/mobile skill

Independent `maximum-range` mode based on c1959ad. Historical periodic capture,
physical-contact baseline, force/controller constants and robot geometry are
retained. This is simulation with an approximate interaction proxy and nominal
D435 intrinsics/extrinsic, not calibrated real hardware.

## Single entry

On the experiment server, from `/data1/home/rangeryx/fr3_real2sim_piper_mobile`:

```bash
python3 scripts/run_wrist_reconstruction.py \
  --mode maximum-range \
  --config configs/wrist_reconstruction_max_range.json \
  --output results/wrist_maximum_20261006 \
  --stage full --resume
```

Omit `--resume` for a new output directory. The saved start time and cutoff are
not reset by resuming. Capture has 12 hours; backend receives 6 hours only after
**both** objects have two qualifying clean states and independently evaluated
physical span. Previous experiments and pre-contact entry smoke are retained.

## Operation

No periodic 15-degree/5-cm release. Measured-q margins, native contact danger,
original load and speed limits remain active. Relative contact-plane drift is a
diagnostic and effort-invalidity flag, not a default whole-task stop. It is not a
complete slip observer. Confidence residual is not relabelled physical slip.

The first bilateral grasp saves the measured TCP and official fixed TCP/gripper
transform relative to the observed handle. Regrasp tries 12 deterministic local
templates, then up to 12 diversified bar-family alternatives; actual closure and
bilateral validation remain mandatory. Recorded aperture is not a closing goal.

A released hand retreats to clearance instead of requiring home. Mobile routes
check the actual locked arm, camera housing and chassis. No base move while the
object is held. The platform remains the existing simulated kinematic SE(2)
platform, not verified wheel/navigation dynamics.

Each operation recovery has a finite 60-base/6-full-path/4-actual-base budget;
12 cycles and 48 base moves per object. Observational anomalies pause further
release commands and require three consistent above-1-mm observations to confirm
motion. A dangerous physical event is never converted to a warning by this mode.

## Progress and resume semantics

- `maximum_progress.jsonl`: new sensor-observed maximum states.
- `max_range_checkpoint.json`: current/max state, measured q/base, handle
  observation, successful template, capture/effort and recovery cursor.
- `observed_progress.jsonl`: RGB-D registrations with confidence audit; EE-only
  fallback explicitly marked `EE_PROXY_NOT_OBJECT_MEASUREMENT`.
- `execution_sources/` + expanded source: actual execution implementation.
- `observations.jsonl`, `command_tape.jsonl`, `physics_steps.jsonl`: complete
  journals; legacy JSON counterparts contain only bounded diagnostic tails.

Recoverable failures run in the same live Isaac process. `--resume` preserves an
already-live actor and its cutoff. If that actor crashed after manipulation,
this version returns `COLD_RESUME_REQUIRES_VERIFIED_ACTION_REPLAY`; it does not
pretend that a JSON checkpoint restores physical state. Automatic cold replay
and continued reobservation remain an explicitly unfinished path. No object
joint position, contact impulses, weld or attachment are restored.

## Results and limitations

`maximum_range_evaluation.json` is an independent **post-stop** actual-object
motion report. It never enters control. State CSVs and REPORT distinguish actual
range from online RGB-D estimate and EE proxy. Reconstruction gates use both
clean captures and independently confirmed actual span; progress is not zeroed
by scan or reconstruction failure.

Effort validity is separate from manipulation acceptance. Invalid relative
motion/contact/regrasp segments retain raw measurements but are excluded from
summaries. Available normal+friction tensor projection is labelled simulation
measurement; otherwise commanded wrench projection is labelled an uncalibrated
proxy. EE motion onset cannot prove minimum static friction.

At implementation freeze: eight policy/integration regressions passed; all 15
existing wrist regressions passed across Isaac and ArtGS environments. Physical
results must be read from the experiment artifacts, not inferred from these
software tests. No new 60-degree/10-cm result is claimed by this document.

## First physical integration regression

Both initial nominal grasps established bilateral hold (7320 approximately
0.526/0.519 N; 45746 approximately 0.518/0.529 N), then the new recovery halt
accidentally changed finger force-hold to position mode and removed preload.
Failures are retained under `hold_transition_failure_<id>`; neither is claimed
as opening success. Recovery now preserves finger hold mode while stopping the
arm; a genuinely failed grasp monitor is explicitly disarmed for its release/
recovery phase, then actual closure must re-establish bilateral validation.
The original contact-loss threshold, force cap, gains and closure law remain.
7320's corrected physical run reached incremental release; the full recovery
cycle and maximum range were still pending at this note's creation.
