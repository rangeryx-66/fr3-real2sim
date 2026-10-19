# Articulated interaction skill

Independent simulation entry; original PiPER grasp, contact, geometry, safety,
unknown-joint controller, mobile recovery and physics calibration remain intact.
This collector does not fit friction parameters or rebuild collision geometry.

## Run

```bash
python scripts/articulated_interaction_skill.py \
  --job path/to/frozen_physical_contact_job.json \
  --joint-type revolute --goal open_for_multistate_capture \
  --output results/multistate --gpu 0 --ffmpeg /path/to/ffmpeg
```

Use `--joint-type prismatic` for a linear object. Default requested states are
0/5/10/15/20 degrees or 0/.02/.04/.06/.08 meters. Safe intermediate states are
also captured every 2.5 degrees or .01 meters. `--targets` changes capture goals,
not physics, force, speed or joint-margin limits. The effort-probe goal defaults to 0/2.5/5 degrees or 0/.01/.02 meters,
collecting the same observable data over a shorter target sequence.

New prepared dataset proxy: first run `--stage prepare --asset-root ASSET` with
the existing reference job. It uses the frozen generic deployment, twelve
bar-side-pinch candidates and eligible SE(2) recovery. Then run its
`execution_job.json`. `--candidate` selects an existing preflight-approved list
entry; it never generates a new grasp or changes force/geometry. Failed trials
remain separate episodes; captures are not merged across simulator resets. Only setup reads the source joint family to import a
passive object; the probe/follow controller receives EE estimates, not GT axis
or object joint state.

A frozen JSON `--manifest` contains `episodes` with `episode_id`, `job`,
`joint_type` and optional `goal`. All failures remain in `batch_summary.json`.
`--stage export --output EXISTING_CAPTURE` regenerates backend bundles without
running physics. Execution stops by the timezone-aware deadline or the frozen
motion/wall-time budgets.

## Outputs and meaning

- `multistate_capture.json`, `states/state_NNN`: calibrated RGB, metric optical-Z
  depth, object/moving-part masks, metric world point cloud, camera/robot/grasp
  transforms, estimated state, stable-hold flag and termination reason. Each state
  carries object/part IDs. Base pose uses m/m/m/degree; `T_world_grasp` is the
  current measured official TCP proxy and `T_world_initial_grasp` retains the
  initial planned grasp. Contact centroid is not separately measured.
- Masks are simulated instance-ID render observations; they are not a real SAM
  implementation. Missing/occluded part pixels are recorded per view.
- `effort_samples.jsonl`, `effort_segments.json`, `effort_summary.json`: first
  rested start, open-state restarts and continuous moving effort stay separate.
  Slow force-limited control is unchanged; these are effective response values,
  not an exact minimum breakaway experiment or fitted friction constants.
- Native contact reports expose normal impulses only. The optional read-only
  PhysX tensor observer separately collects normal and friction buffers. Only
  verified bilateral buffer coverage can produce a full-force measurement.
  Otherwise the primary effort is explicitly a commanded Cartesian **proxy**;
  raw tensor and normal-only diagnostics remain available. ContactOffset is not
  a penetration tolerance. No new contact acceptance rule is introduced.
- `skill_result.json` separates partial capture success, all requested targets,
  physical post-run state verification and backend-input availability.
- Independent simulator link/joint logs enter only post-run evaluation. They do
  not generate motion or select targets online.

## Reconstruction interfaces

`reconstruction/art_input.json`: state-grouped RGB, masks and calibrated cameras,
with optional estimated articulation labels. This is ART-style input: the ART
project page does not publish an executable official loader/checkpoint contract;
no ART inference is claimed.

`ditto_pairs.json` + `pair_A_B.npz`: adjacent and closed-to-final pairs, shared
world frame and bbox normalization, deterministic 8192 points, exact official
`pc_start`/`pc_end` tensors of shape `(1,8192,3)`. Load with
`articulated_interaction_skill.backend_sample.load_ditto_sample`. The official
notebook's hardcoded camera is not reused.

`house_ditto_input.json`: paired RGB-D/cloud observations, an interchange
manifest rather than a claim to implement every HouseDitto preprocessing stage.
`urdf_anything_plus_input.json`: optional image/metric-scan observations.

`twin_update.json` stores estimated structure, observed range and effort with
provenance. Source assets are preserved; full joint limits and newly reconstructed
geometry are not inferred from a short capture. Backend reconstruction must run
separately before claiming a reconstructed geometry update.

## Reposition boundary

Initial deployment recovery is executable. Mid-episode base motion is prohibited
while gripping. A continuation requiring recovery returns
`REPOSITION_REQUIRES_SAFE_REGRASP` if safe release/retreat and a fresh observed
handle pose are unavailable. Automatic mid-episode visual regrasp is not supplied
by this collector, and no simulator reset or teleport substitutes for it.

## Primary contracts

- ART: https://kyleleey.github.io/ART/
- Ditto: https://github.com/UT-Austin-RPL/Ditto/blob/master/notebooks/demo_depth_map.ipynb
- HouseDitto: https://github.com/UT-Austin-RPL/HouseDitto
- PhysX force buffers: https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html
- Normal-only contact impulse: https://nvidiagameworks.github.io/PhysX/4.0/documentation/PhysXAPI/files/PxSimulationEventCallback_8h_source.html
