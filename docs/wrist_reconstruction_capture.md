# Wrist-camera articulated reconstruction experiment

Basis: `1b3b3cd`. The successful contact/gripper/proxy, Act2See controller, mobile search and effort implementations are retained verbatim; SHA256 checks cover eight files. This independent orchestration adds released-arm scans and fresh target replanning.

## Run/resume

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
PYTHONNOUSERSITE=1 environments/artgs/bin/python scripts/run_wrist_reconstruction.py \
  --config configs/wrist_reconstruction.json --stage full --gpu 0
```

Components and original failures are retained independently. `capture`, `audit-old`, `coarse`, `backend`, `report` can be resumed. Full optimization requires inspecting actual coarse renders/clouds/centers and recording `coarse_review.json` with `full_optimization_allowed: true`; this is a backend quality review, not permission to ignore robot safety. Coarse failures must not consume 20k joint iterations.

## Camera boundary

PiPER's product and arm-URDF documentation do not establish a universal calibrated wrist camera. The manufacturer's demonstration supports D435 or Orbbec Petrel (https://www.hackster.io/agilexrobotics/control-piper-arm-with-hand-gestures-7c2779). The supplied D435 configuration uses a nominal RGB FOV and a clearly labelled **simulation-design** mounting transform. It is not per-device CameraInfo or real hand-eye calibration. No output may claim real deployment calibration until those are replaced by actual measurements. Full-background RGB remains intact; SAM3 produces sensor-derived masks. Instance segmentation is saved as QA only, never the reconstruction mask or regrasp pose source.

A proposed optical camera pose is mapped to TCP by `T_world_TCP = T_world_camera @ inverse(T_TCP_camera)`. Robot IK, >0.05 rad margin and a collision-checked joint path must pass before acquisition. Camera pose updates from measured TCP are rigid extrinsics, not independent orbit-view teleports. Normal room surfaces provide a background; no object mesh or physics parameter is modified. The nominal housing is included in scan/environment feasibility; actual mounting clearance still requires hardware validation.

## Regrasp

After safe release, retreat and an optional existing SE2 route, the wrist camera physically moves to an observation pose. Registration uses unmasked RGB-D in the observed handle neighborhood. Registration uncertainty can reject localization; a 1mm change from the earlier world pose cannot. Current observed targets are replanned through the existing generic grasp family/IK/collision planner before executing original closure and hold. No GT joint/pose, attachment or object drive is introduced. Release-motion safety remains unchanged.

## Data/ArtGS

OLD, WRIST_CLEAN and ORACLE_CAPTURE receive the same full-FOV adapter and 10k/5k/20k budget, point-cloud initialization and official stage hyperparameters. An oracle camera can move freely only at an actually reached state; it is labelled evaluation-only and cannot count toward wrist/system success. Inadequate actual state span is reported, not replaced by GT articulation commands.

Old input audits revealed some later object clouds reaching the distant floor. The updated sensor writer deep-copies RGB/depth/segmentation from a completed render together and applies sensor masks afterward. Rectangular RGB-D is preserved by the ArtGS adapter: the prior central square crop could remove structural extremities. Original files/outputs remain available. These are acquisition/adapter corrections, not backend geometry substitutions.

SAM3.1 checkpoint loading into the current SAM3 image code reports four missing vision-backbone keys; retained as a compatibility risk. If sensor segmentation cannot localize an object, it is a recorded blocker, not replaced by simulator masks.

Only a high-coverage, large-span oracle result under the official budget can trigger ArtGS's method death criterion and the one permitted REArtGS++ fallback. Missing robot capture or insufficient span does not justify such a verdict.

## Deployed source chain

The server also retained older root-level copies of entry scripts. The independent wrist runner explicitly imports the checked-in `scripts/` chain and checks the preplanned retreat runtime API before starting Isaac. Old files and failed run outputs are retained. A zero process exit code is not physical success: the ledger separately reports the episode physical status.
