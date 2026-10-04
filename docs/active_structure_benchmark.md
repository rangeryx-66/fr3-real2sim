# Provisional discovery and active structure refinement

Independent entry, based on `eb0fefa`. Existing physical/contact/controller,
mobile recovery and robust fitter modules are imported unchanged. No physics
parameter fitting exists in this experiment.

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
 /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
 scripts/run_active_structure_benchmark.py --config configs/active_structure.yaml
```

Stages: `dev`, `controls`, `freeze`, `benchmark`, `summary`, `full` (default).
A later replication needs a new output directory and deadline; existing outputs
and TEST manifests are immutable.

## Before TEST

Diagnostic DEV is exactly 45600/38516/45403 configuration 0, using their existing
reachable nominal grasp and unchanged approach/contact/hold. Controls 7320 and
45621 repeat the old 5.5 degree task. TEST cannot be frozen until both controls
pass and all three DEV trials actually complete at least one refinement segment.
The remaining pre-budgeted DEV actions can finish after the method freeze;
TEST waits for each GPU lease and never shares a GPU with an active DEV world.
No subsequent DEV or TEST result changes the frozen method.

The frozen selection excludes all 25 earlier prepared IDs and the entire prior
57-asset source pool. Selection is category-balanced and visual-geometry ranked,
using the existing data preparation/proxy/deployment rules. Six TEST asset IDs, two
initial configurations each, thresholds and budgets are saved before physical
TEST execution. Neither outcome-based removal nor per-asset parameter changes
are supported.

## Discovery semantics

- `ACCEPTED`: original strict coarse structure gate.
- `PROVISIONAL_REVOLUTE`: measured travel at least 5 mm; revolute score evidence,
  measured rotation, temporal/block bootstrap axis/radius/tangent consistency,
  and plausible local radius. It grants only bounded refinement, not successful
  identification. Native grasp retention and safety guards remain active.
- `UNOBSERVABLE`: insufficient excitation, contradictory type evidence or
  unstable candidate set. Provisional thresholds are distinct from final ones.

Neither classifier nor controller receives asset joint type/axis/position.
Candidate directions come from measured EE hypotheses. No vertical axis prior.

## One bounded refinement

120 segments maximum; each requests about 1 mm with a 4 s timeout. Total measured
path is capped at 120 mm and simulation time at 270 s. The estimated target is
6 degrees. The safety conditions always override these caps/goals.

At each segment, take candidate tangents near the measured EE, maximize worst
case angular excitation, and require incremental exact IK, >0.05 rad margin,
and the unchanged scene/contact collision constraints. The existing compliant
controller executes only that small direction. Refresh the candidate set from
measured EE after each segment and call the unchanged robust SE(3) fitter on
provisional entry and every five segments. Model-consistency prediction uses
the complete fitted reference pose; the measured first frame is not silently
substituted for that latent reference. The 1 mm safety threshold is unchanged. Force, speed, contact and slip limits remain unchanged.

Reverse is optional; this initial protocol uses a complete independent forward
validation action. It does not require submillimetre reverse fitting. Final
validation retains the existing robust fit/model consistency and independent
validation rules, and explicitly enforces <0.30 mm position residual and
<0.001 rad rotation residual. The final training span must reach 2 degrees.

An accepted model drives the existing separate 8 s forward / 2 s dwell held-out
action; no held-out frames enter fitting. Autonomous T0/T1/T2 same-command
prediction remains separate from geometric cross-action prediction. Incomplete
native replay is reported as incomplete, without a full RMSE.

## Safety and limits

No baseline grasp, official geometry, proxy, collision, force, friction or
mobile route change. The existing slip guard also incorporates model
consistency error; its legacy label does not alone establish physical slip.
GT is available only to scene initialization and the post-stop evaluator.

7130 is excluded as exposed diagnostic data. Existing RGB-D tracking is not a
validated physics-settle-to-pregrasp relocalization/replanning interface, so no
object-specific fix or GT joint reset is added.

Outputs include frozen method/TEST manifest, per-episode and per-asset CSV,
discovery-to-refinement transitions, failure breakdown, plots, continuous
native videos and REPORT. Deployment, interaction, discovery, refinement and
end-to-end success have separate denominators; provisional is never counted as
accepted structure identification.
