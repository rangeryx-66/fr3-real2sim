# Multistate collection → actual ArtGS → inferred articulated twin

Basis: `3af4cd3`, branch `codex/piper-mobile-door`. This integration leaves successful grasp/contact/controller/proxy/safety source files unchanged. ArtGS was selected explicitly by the user after Ditto's official Box model/demo links returned `sharedNotFound`. That dependency failure and earlier collection failures remain in the run records.

## One entry, independent resumable components

On the existing lab server checkout:

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
export PATH=/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:$PATH
unset PYTHONPATH
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  environments/artgs/bin/python scripts/run_articulated_system.py \
  --config configs/articulated_system.json --stage full --gpu 1
```

The frozen configuration ends at **2026-10-06 05:00 Asia/Shanghai** and has an eight-hour overall budget. A later experiment requires a new output directory and an explicitly updated deadline configuration; a changed config cannot silently resume the same run.

Stages: `collect`, `prepare`, `reconstruct`, `twin`, `preview`, `validate`, `report`. `full` first runs reconstruction on durable existing captures, then physical extension collection, and immediately processes newly acquired views. A failed release/reposition does not cancel the reconstruction component. Each stage saves commands, logs, return status and retained intermediate files. Stage-level resume is supported; interrupted optimization is not claimed to restore optimizer state exactly.

To process already acquired extension data without restarting the robot:

```bash
environments/artgs/bin/python scripts/run_articulated_system.py \
  --stage reconstruct --object 7320 --pair small \
  --capture-root results/articulated_system_20261005/7320_visibleviews \
  --output results/articulated_system_20261005/extension/7320 --gpu 1
```

Omit `--pair` after collection stops to build the small and largest actually captured state pairs. `--interaction-type-prior` is a separately labelled official ArtGS option: only the joint family from measured-EE interaction identification is supplied; its axis, center and parts remain image/point-cloud inferred. Native automatic type results are preserved in different output folders. No dataset GT axis is substituted.

## Official backend and environment

Source: <https://github.com/YuLiu-LY/ArtGS>, pinned commit `7c1f41be2cb8b96abca13c6a9668dcf7e06d8c2a`, recursive submodules. Author demo data/checkpoints: <https://huggingface.co/datasets/YuLiu/ArtGS-Dataset>. The `storage_45503` author checkpoint is used only for its own official demo. Target assets are optimized from their actual RGB-D observations; this is scene-specific optimization, not general pretrained inference.

Independent `environments/artgs` uses Python 3.10, torch 2.8/CUDA 12.8 runtime, locally built CUDA 12.5 extensions, Lightning 2.4, pytorch3d 0.7.9, tinycudann 2.0, the official pointnet extension, simple-knn and the repository's depth/alpha rasterizer. Source/environment hashes and versions are saved in `backend_provenance.json`. CUDA builds do not modify the Isaac environment. The official environment recipe is also in the linked README. On this machine readonly parent paths supply torch; exact package snapshots accompany results. Model/demo archive checksums are retained.

Actual bounded schedule: 3,000 coarse + 3,000 type prediction + 5,000 joint iterations, seed 61. The official downstream path expects `iteration_10000`: a symlink points to the actual 3,000 checkpoint, explicitly recorded, without claiming 10,000 steps. Losses and trainer implementations are the author's original code.

## Input and output contract

- Real simulator camera RGB, optical-Z depth, intrinsics and camera transforms.
- Simulator instance masks, explicitly not SAM or real segmentation, exclude robot/table pixels. Invisible surfaces are not replaced with reference meshes.
- Frozen-state orbit cameras render with physics paused; object joint state is never commanded for capture.
- The second capture revision uses a 64° horizontal orbit-camera FOV to include the whole object; the earlier narrow-camera captures and outputs remain diagnostic failures. Per-camera rays are resampled to a shared intrinsic matrix required by the official loader. Out-of-image samples are masked, not fabricated.
- All input states share one meter-preserving normalization. Each state has its own cloud; moving states are never merged into one static cloud.
- True loader files: `transforms_train_start/end.json`, `transforms_test_start/end.json`, RGBA/depth folders, `point_cloud_start/end.ply`, detection marker and provenance.
- Two-view historical inputs have no independent held-out views. New captures reserve views. State 1 is independently withheld from state-pair reconstruction and evaluated by `evaluate_artgs_state.py`.

The held-out renderer uses the measured estimated state label to supply phase. It is **conditional geometric prediction**, not autonomous motion or physics prediction. Larger opening does not automatically imply improved reconstruction. Native joint-type mistakes, incomplete meshes and empty meshes are reported, not patched with GT.

Held-out RGB/silhouette metrics evaluate the learned Gaussian representation. Exported TSDF meshes receive a separate qualitative Isaac import/assembly check; good Gaussian rendering alone does not establish accurate URDF collision surfaces.

A twin contains actual `reconstructed_parts/`, metric `reconstructed.urdf`, `state_observations/`, `effort_profile.json`, and `twin_update.json`. State folders link to durable source observations: archive with `tar --dereference` for portability. Initial assembly is preserved by moving-mesh recentering around the inferred parent-frame joint origin. The joint range is the inferred/observed preview range, not a recovered full limit. Mass/inertia are neutral preview priors. An independent Isaac scene imports the URDF and drives that reconstructed joint for an assembly video; this is **not physical robot manipulation success**.

## Segmented physical continuation

Original approach/closure/contact hold and limits are reused. At a 0.10 rad warning margin, preplan collision-checked joint-space retreat/home and existing finite SE(2) recovery. A straight 4cm retreat is only a fallback: preservation of all six TCP pose coordinates is unnecessary for a released-arm retreat. Release aperture is measured opening plus a bounded 20mm clearance; the route model uses the same aperture actually reached before retreat. Observe current moving-part RGB-D, register it to initial observations, and transform handle targets from observations. Release slowly; observed spontaneous motion rejects release and recloses. Otherwise retreat, follow the collision-checked base route, stop/lock, observe again and physically regrasp. No attachment, object external force, joint command or reset restores task progress. Maximum three repositions; failure remains a result. The base model is the existing kinematic SE(2) chassis route, not wheel/navigation dynamics.

## Effort interpretation

Verified normal+friction buffers provide force on the handle in world coordinates, in N; they are not a sum of jaw normal loads. Motion-onset windows, stable low-speed windows, actual EE speed and raw series are saved. EE onset can include grasp compliance. Onset force is **task-opening resistance**, not an exact minimum friction value. Unverified force remains command proxy. No tau_c/b/tau_s fitting is performed and effort is not silently mapped to URDF friction.

When a completed episode has an independent evaluator log, `effort_profile_object_evaluation.json` additionally uses an actual moving-link rigid marker at the initial observed grasp location to identify motion onset. This simulation-only postprocessing is separate from online EE signals, and cannot feed control or pose selection. A stationary object with a moving compliant EE cannot produce a claimed object-startup measurement in this profile.

Release/regrasp orchestration explicitly clears obsolete joint effort commands when switching to the original position-control gains. This fixes a mode-transition command bug without modifying grasp closure, gains, force limits or physics parameters. The earlier release failures are retained.

## Validation and records

`tests/test_articulated_system.py` checks metric assembly/gauge handling and prevents unverified force from becoming a measured physics claim. Actual official demo, target reconstruction, held-out renders and URDF imports are necessary evidence in addition to tests.

`python scripts/report_articulated_system.py --root results/articulated_system_20261005` generates actual reconstruction and collection tables, preserving errors and differentiating software stage completion from physical/reconstruction quality success. Before publication, consult the per-run reports and videos; a valid URDF import alone does not establish a useful articulated reconstruction.
