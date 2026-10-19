# Wrist reconstruction status — 2026-10-06

Basis: 1b3b3cd, codex/piper-mobile-door. **Incomplete experiment, not successful wrist reconstruction.**

## Camera provenance

AgileX's manufacturer-authored example lists D435 and Orbbec Petrel as alternatives, not one compulsory PiPER wrist camera: https://www.hackster.io/agilexrobotics/control-piper-arm-with-hand-gestures-7c2779 . D435 sensor specifications: https://www.realsenseai.com/products/stereo-depth-camera-d435/ .

The independent simulation profile uses D435-like 1280×720 RGB-aligned optical rays (K: fx=fy=925, cx=640, cy=360). This is nominal, not measured CameraInfo. The mounting transform is a labelled simulation design assumption, NOT an official hand-eye calibration. Housing/environment feasibility is checked analytically; real mount clearance, camera/arm clearance and per-unit calibration remain unverified. Initial localization currently uses an external RGB-D view; reconstruction capture targets the measured-TCP-mounted camera. This is not a fully wrist-only perception system yet.

## Actual runs

| Object | Actual grasp | Release/retreat | Clean wrist states | First blocker |
|---|---|---|---|---|
| 7320 | bilateral hold established | release + arm-home retreat completed | 0; one attempted closed state with 0 views | SCAN_NO_IK; 9 view proposals lack IK, 3 would crop the object; handle reobservation also lacks IK |
| 45746 | bilateral hold established | release completed, retreat interrupted | 0 | FINGER_BACK_OR_ROOT_HANDLE_LOAD:gripper_link1 during SYSTEM_RETREAT |

Actual v3 run directories on server:
- results/wrist_reconstruction_20261006_v3/capture_7320
- results/wrist_reconstruction_20261006_drawer_v3/capture_45746

Minimum joint margins: 7320 0.0634903 rad; 45746 0.1715745 rad. No attachment, object actuation or online GT pose was introduced. Final evaluator motion was only 0.00762° / 0.00195 mm, respectively; neither represents multi-state opening success. The videos contain approach/closure/release/retreat and the stop. New task effort has not been measured because manipulation was never reached. Existing effort implementations/logs remain unchanged.

First attempts failed on the installed Camera API. Another run loaded older root-level entry copies instead of scripts/. Both infrastructure failures are retained; explicit checked-in source-chain imports now prevent that shadowing. No contact/collision/grasp thresholds were changed.

## Input/backend diagnosis actually run

The sensor smoke obtained RGB/depth/QA dimensions 1280×720 and a SAM3 object cloud approximately 0.521×0.284×0.332 m. This verifies sensor plumbing, not robot-executable scanning. SAM3.1 compatibility reports four missing keys.

OLD 7320 ArtGS coarse ran **10,000 iterations** in 215.4 s, with point-cloud initialization. Predict and full stages did NOT run. Old later-state clouds contain distant-floor points and imply ~4.1 m normalization for a ~0.53 m object. The apparent moving cluster is outside the object; coarse geometry is distorted. Full optimization is refused for this contaminated input. The exact historical modality misalignment mechanism is not proven.

The adapter now preserves rectangular full sensor FOV. The new writer obtains copied modalities from completed renders; its multi-state alignment still requires successful acquisition validation. No adequate WRIST_CLEAN or matching large-span ORACLE data was produced, so there is no valid OLD/B/C comparison or ArtGS-method verdict. REArtGS++ fallback was not triggered. No new mesh/URDF/Isaac reconstructed twin is claimed.

## Reproduction and saved progress

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
PYTHONNOUSERSITE=1 environments/artgs/bin/python scripts/run_wrist_reconstruction.py \
  --config configs/wrist_reconstruction.json --stage full --gpu 0 \
  --output results/wrist_reconstruction_new_attempt
```

This command is bounded by the configured Shanghai cutoff. Existing outputs are retained and stages are independently resumable. A process exit code of zero does not imply physical success; read report.json / multistate_capture.json. The eight frozen physical/control/mobile/effort files passed SHA256 regression checks. Frame-transform/FOV checks passed.

Next required work is camera-task deployment/viewpoint feasibility and safe retreat orchestration, while retaining the contact baseline. These are not solved by changing ArtGS iterations or inventing a universal camera extrinsic. No repeated unchanged capture or full ArtGS run is warranted until those blockers are resolved.
