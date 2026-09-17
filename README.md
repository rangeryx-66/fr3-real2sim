# FR3 grasp, scan and physical asset identification

This repository contains the research code used to grasp objects with AnyGrasp + MoveIt 2 + FR3/Franka Hand in Isaac Sim, scan released objects with a stationary RGB-D camera, reconstruct visual shape with the **external official MV-SAM3D** implementation, and estimate payload mass and center of mass (COM) from paired empty/loaded FR3 torque records. It also contains CoACD collision and Isaac USD packaging utilities.

**Reproduction status.** The grasp and PayloadID branches were run on the lab Isaac host. The stationary scan and official MV-SAM3D 2/4/8-view inference were run there as separate stages. `result.glb` is an MV-SAM3D output; it does **not** by itself have verified metric scale, collision or measured inertial properties. The provided metric/CoACD/USD utilities require metric mesh and inertial inputs. There is currently no validated one-command `MV-SAM3D GLB → metric USD` runner, so do not describe an arbitrary GLB as a validated physical asset. The station simulator initializes an object at the scan station; automatically transporting a previously grasped object to that station is not part of the tested runner.

The repository does **not** redistribute AnyGrasp SDK, weights or issued license files, MV-SAM3D weights, Isaac/Arena assets, the official Scalable Real2Sim code or generated experiments. Get each from its original distributor under its own terms. No GT mesh, pose or dynamics parameter is used to fit the PayloadID estimator; GT is used only after fitting for evaluation.

## Code map

| Stage | Main code | Process/runtime |
|---|---|---|
| Grasp inference | `src/infer.py` | Separate AnyGrasp Python environment; consumes measured masked point cloud; preserves SDK score/rank/pose |
| Plan and execute | `calibration/unseen_backend.py`, `calibration/stability_backend.py`, `src/*backend.py` | ROS 2 MoveIt 2 + isolated Isaac bridge; grasp → pregrasp → approach → close → target-height lift |
| Scan | `calibration/run_scan_station.py`, `real2sim/scan_station.py` | Isaac RGB-D two-ring stationary scan; RGB, depth, masks, intrinsics and camera pose |
| MV-SAM3D inputs | `experiments/prepare_mv_sam3d_inputs.py` | Select 2/4/8 views, write `images/`, object RGBA masks, selection manifest and overlay |
| MV-SAM3D inference | `scripts/run_mv_sam3d.sh` | Calls the *external* `run_inference_weighted.py` with `gaussian,mesh` decoding |
| Mass and COM | `calibration/run_payload_capture_recovery.py`, `calibration/run_payload_fresh_gate_validation.py`, `real2sim/payload_id_official_mass_com.py` | Static paired empty/loaded torque, corrected actual-q velocity gate and Drake q-mismatch correction |
| Collision and USD | `real2sim/collision_mesh.py`, `real2sim/asset_builder.py`, `real2sim/validate_asset.py` | CoACD and Isaac USD packaging/reload checks once metric mesh and inertial JSON exist |

`calibration/` and `src/` also retain historical research entry points for audit. The supported workflow is described below; a historical runner's absolute path or result directory is not a current default. `config/mv_sam3d_static_frozen_v1.json` records the tested 8-view selection (2/4 are ablations). `config/fr3.urdf` and `config/fr3.srdf` are the generated FR3 descriptions used in the experiments.

## External versions and setup

Use Linux with an NVIDIA GPU. The reference host used Isaac Sim 6.0.1 / Python 3.12, a separate RoboStack ROS 2 Humble/MoveIt environment, a separate AnyGrasp environment, a separate MV-SAM3D environment, and a Drake/official PayloadID environment. Keep these interpreters separate; Isaac's Python, ROS, AnyGrasp and the official estimator have incompatible dependency sets. A package-exact ROS environment export is `ros_environment_explicit.txt`; paths and GPU drivers are host specific.

| External checkout | Tested commit | Install destination |
|---|---|---|
| [Isaac Lab Arena](https://github.com/isaac-sim/IsaacLab-Arena) | `c8cca680cc32c770ed315aab9574a9040fe078fe` | any path, referenced by `ISAAC_PYTHON` and `ARENA_METADATA_JSON` |
| [Franka description](https://github.com/frankarobotics/franka_description) | `7aeeddc449edf8d62b594f9e36a81da53e7796f9` | `./franka_description/` (contains visual/collision meshes; not vendored) |
| [MV-SAM3D](https://github.com/devinli123/MV-SAM3D) | `abb04b5e8af5bc33b0265bdf19937e76bbb6bcdd` | any path, `MV_SAM3D_ROOT` |
| [Scalable Real2Sim](https://github.com/nepfaff/scalable-real2sim) | `a8e4d97cbb0c3ea887a69fa313bcd3a252c5a8a3` | any path, `OFFICIAL_PAYLOAD_ROOT` points to its `scalable_real2sim/` directory; the robot_payload_id snapshot used in the FR3 adaptation was `c52e31cf26c83b33aee5e56f805e1d4d710fd549` |
| AnyGrasp SDK | user's vendor supplied build | any path, `ANYGRASP_SDK_ROOT` |

After cloning this repo, copy `.env.example` to `.env`, set paths, then load it with `set -a; source .env; set +a` in each shell. `.env` is gitignored. If downloads require the lab proxy, run `source ./proxyon.sh` **before downloading** and set `PROXY_ON_SCRIPT` only if the runtime needs it. There is no proxy script or credential in this repository.

Place the vendor supplied AnyGrasp files at their original SDK relative locations:

```text
$ANYGRASP_SDK_ROOT/
  gsnet.py
  log/checkpoint_detection.tar   # vendor checkpoint; not committed
  license/                   # complete issued license bundle; not committed
```

Do not rename a license file or create a replacement. Follow the vendor SDK README/activation instructions. `src/infer.py` points `--sdk` to this directory and passes its unchanged checkpoint to `create_detector`. The preflight checks for the checkpoint and an issued `.lic` file without reading its contents. MV-SAM3D checkpoints are placed according to its upstream README and loader; this repository does not copy them.

Get the Arena maple scene metadata from the external Arena checkout, set `ARENA_METADATA_JSON`, and regenerate the local asset inventory/GT meshes with `src/probe_arena_assets.py` in Isaac Python. `assets/arena_complex/inventory.json` is included for IDs and source USD provenance, but the `.npz` mesh exports and `.usdc` scene layers must be regenerated from Arena on the host. Those GT exports are used for simulator collision/diagnostic scene construction, never as MV-SAM3D reconstruction input. `FR3_ASSET_DIRECTORY` points to that output directory.

After cloning the official Franka description into `./franka_description`, run `python scripts/relocate_description.py` to rewrite the generated URDF mesh paths to this checkout and validate referenced mesh files. The URDF still contains the same FR3 kinematics and inertials. The generated SRDF is already in `config/`.

Run `python scripts/check_environment.py --stage all` to see missing external components. Use `--stage grasp`, `scan`, `mv` or `payload` for a smaller check.

## Reproduce the grasp executor

From this repo root, with `.env` loaded and Arena assets prepared:

```bash
bash run.sh sim        # Isaac process
bash run.sh bridge     # ROS joint-state/action bridge, separate terminal
bash run.sh moveit     # move_group, separate terminal
bash run.sh trial 10   # AnyGrasp + MoveIt trials, separate terminal
```

Wait for `SIM_READY` and MoveIt's planning ready log before trials. The Arena household-object branch is `calibration/launch_real2sim.py` with `--target`, `--seed`, `--mode ANYGRASP`, `--gpu`, `--port`, `--domain`, `--output`, `--skip-scan` and optionally `--capture-payload --payload-v2 --mass-com-only`; run `--help` for all arguments. It launches the same frozen executor via `run_real2sim_pipeline.py`. Set `ROS_DOMAIN_ID` to avoid other ROS sessions. The simulator uses local HTTP only for transport between runtimes; MoveIt handles IK, collision and trajectory planning. The grasp result is determined from actual contact, target lift and hold, not trajectory completion.

Transforms use column vectors, meters and `T_A_B` mapping B coordinates into A. AnyGrasp input/output is `camera_optical`; `T_base_camera` is saved with scan/grasp data. The mapping to `fr3_hand_tcp` is documented in `src/frames.py`; do not apply the flange mount transform a second time. The target is a free PhysX body during physical success checks.

## Reproduce a stationary scan and MV-SAM3D inference

The station runner creates a clean station scene and parks the arm out of view. For the tested continuous two-ring distribution:

```bash
"$ISAAC_PYTHON" -u calibration/run_scan_station.py \
  --target soup --seed 1030 --output results/station_soup \
  --gpu 3 --port 18930 --official-frame-count 1800 \
  --skip-reconstruction

bash scripts/run_mv_sam3d.sh results/station_soup/scan_station soup \
  results/station_soup/mv_sam3d 0 8
```

`--skip-reconstruction` stops the older ReconViaGen route after acquisition. The scan writes per-frame `rgb/`, depth, masks, intrinsics, `poses/` and `scan_manifest.json`; `eval_gt/` is diagnostic only. The input adapter selects four azimuths on each of the horizontal and high rings by deterministic frame index. Check `inputs/views_08/input_mask_overlay.png`, `selection.json`, mask area and pairwise overlap before interpreting a generated mesh. The official MV-SAM3D script prints its timestamped `result.glb`/`result.ply` path into `inference.log`. For the historical 2/4-view comparisons, run the same script with `2` or `4`. `config/mv_sam3d_static_frozen_v1.json` records the exact tested 8-view indices for the 1800-frame scan.

The source RGB-D and `T_base_camera` are necessary to recover metric scale and audit observed versus generated surfaces. Do not directly treat MV-SAM3D's raw absolute scale as measured. For a metric visual mesh with an appropriate real-image texture and an inertial JSON, the existing postprocessing commands are:

```bash
"$ISAAC_PYTHON" -m real2sim.collision_mesh metric_visual.obj results/object/collision
"$ISAAC_PYTHON" -m real2sim.asset_builder \
  --name object --visual metric_visual.obj --texture measured_texture.png \
  --collision results/object/collision/collision_manifest.json \
  --inertial inertial_object_simulation.json --output results/object/usd
```

The object inertial JSON must contain `mass` (kg), `center_of_mass` (m) and a 3×3 `inertia_matrix` (kg·m²) in the mesh/object frame. The asset builder writes a USD layer and readback metadata. Use `real2sim/validate_asset.py` for a fresh Isaac reload/drop check. The tested MV-SAM3D path ends at the generated visual GLB; the metric alignment and texture requirements above are explicit inputs, and their correctness must be verified before claiming a simulation-ready asset.

## Reproduce static paired mass and COM validation

`calibration/launch_payload_fresh_gate.py` is the latest isolated PayloadID capture entry. It calls the frozen grasp executor, then uses a recent free-space stability window before starting torque recording. The old historical support verdict is stored as shadow data. Recording still rejects contact loss, table support, relative slip, non-quasi-static `q` and transient torque. The two validated batches are `run_payload_capture_recovery.py` (raisin/sugar/mug/banana capture diagnostics) and `run_payload_fresh_gate_validation.py` (mug/sugar start-gate correction). These are **fixed experiments with fixed seeds**, not generic production services. They write both accepted and rejected attempts under `results/`.

For a single experiment, pass `--capture-payload --payload-v2 --mass-com-only --calibration-force 60 --calibration-mu 1` to `calibration/launch_payload_fresh_gate.py`, then collect a matching empty run with `--baseline-only --center-q <payload/excitation_center.json> --gripper-opening-mm <actual captured opening>`. The paired wrapper runs those steps automatically, retaining only settled actual-q windows. `calibration/evaluate_cross_object.py` uses robot-only Drake q-mismatch torque correction; GT mass/COM enters the evaluator *after* fitting for error and residual diagnosis. The current static gate uses q finite difference <0.002 rad/s; COM fitting uses actual q and matched opening. The 0.3 rad dynamic excitation design is preserved but **not executed** in this latest static comparison. Inertia remains geometry fallback.

Measured COM is **not globally enabled**. On six evaluated objects, soup/mustard/sugar passed the estimator confidence gate; banana had 5.97 mm GT error but 18.58 mm COM uncertainty, while mug/raisin failed. See `docs/VALIDATION.md`. Use identified mass only when its own confidence checks pass; otherwise keep explicit geometry fallback. No USD was rewritten from the latest validation.

## Provenance and limits

- The repo contains adapters and research runners, not upstream projects or their model weights. External project licenses and any AnyGrasp commercial license remain their owners' responsibility.
- Reproducing *identical* numerical results requires the stated external commits, scene assets, simulator/driver versions, fixed seeds, GPU behavior and source hashes recorded with each run. Contact dynamics can vary across platforms.
- Some historical scripts in `calibration/` and `experiments/` retain host-specific defaults. Set the documented environment paths for the current entry points. They are kept for audit and are not silently included in the supported launch path.
- The scan-station scene and the grasp-to-station transport are separate tested skills; a physically continuous grasp → placement → scan run still needs an explicit transport handoff.
- A generated GLB alone is not an Isaac physics asset. Metric alignment, texture, collision decomposition, inertia frame agreement and fresh reload are separate checks.
