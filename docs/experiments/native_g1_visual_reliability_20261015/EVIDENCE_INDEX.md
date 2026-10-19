# Evidence index — private raw data

Reference `9ae41fb`; frozen executable input version `4e91cc2`.
Registered physical order: 45600 → 45606 → 45671, one episode each.

Public: implementation, protocol, compact scalar results, source/configuration
hashes and offline diagnostic summaries. No raw archive, RGB-D or video is
published to GitHub or a GitHub Release.

## Private archive

- Server: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_visual_reliability_20261015_private.tar.gz`
- Checkout-local: `engineering_artifacts_20261015/native_g1_visual_reliability/native_g1_visual_reliability_20261015_private.tar.gz`
- SHA-256: `50174cab70134033449e40498af25d480a452bf777fcd0dba3a721ef2bf5b1f8`
- Bytes: **352,056,053**; members: **1,306**. Lossless outer gzip plus the
  original lossless chunked telemetry. The local copy was verified to match this checksum.
- Byte-identical extracted local videos: `engineering_artifacts_20261015/native_g1_visual_reliability/videos/<asset>.mp4`.

Archive root: `native_g1_visual_reliability_20261015/`.
For each asset, `<asset>/physical_01/` contains:

| Evidence | Files |
|---|---|
| Fresh observations | `rgbd/initial*/rgb.png`, `depth_m.npy`, `camera.json`, robot self-exclusion, raw/extended masks, detection attempts |
| Pregrasp observation/correction | `rgbd/pregrasp*/`, `visual_correction.json` |
| Observed-only plan | `visual_plan.json`, `visual_planning/`, `visual_source_boundary.json` |
| Continuous native-contact video | `contact_baseline.mp4` |
| 240 Hz raw/processed streams | `observations_chunks/`, `physics_steps_chunks/`, `closure_commands_chunks/`, each with `index.json` |
| Sensor interpretation | `sensor_metadata.json`, `joint_sensor_index_metadata.json`, native contact/sensor API source snapshots |
| Actual loaded source | `active_source_provenance.json`, `VISUAL_EXECUTABLE_HASH.json`, `visual_adapter_provenance.json`, executed adapter snapshots |
| Outcome | `report.json`, `live.jsonl`, `perception_recovery.json` |
| File integrity | `PRIVATE_EVIDENCE_HASHES.json`, `VIDEO_DECODE.log` |

Each case directory also contains its actual job, process start/exit and GPU
capacity records. Root files include `PREREGISTRATION.json`, `PAIRED_RESULTS.json`,
`CONTACT_MISMATCH.json`, `FREEZE_AUDIT.json`, `OWNED_PROCESS_AUDIT.json`, model and
analysis-source hashes. `analysis_source_snapshot/` preserves the actual scripts.

`diagnostics/` preserves repeated exact-byte SAM3 tests, missing-branch
intervention, archived geometry evaluation, observed-only reruns, plan comparison
and private video QA. GT comparisons are offline only; they were not used to
correct executable visual poses.

## Integrity and interpretation

| Case | Video duration | Resolution/rate | 240 Hz observation = physics records |
|---|---:|---|---:|
| 45600 | 179.900 s | 1280×960 / 30 fps | 43,177 |
| 45606 | 154.833 s | 1280×960 / 30 fps | 37,162 |
| 45671 | 185.600 s | 1280×960 / 30 fps | 44,547 |

All chunks matched their indices and all videos fully decoded with zero decode
errors. 45671's close view is arm-occluded; the continuous overview inset and
raw contacts remain. Failed-case `slip=0` in the legacy overlay is not a valid
post-acquisition slip measurement; no grasp had been acquired. Use the offline
closure-relative motion and explicit acquisition event instead.

Native normal impulses/contact points are not full wrenches, friction-anchor
measurements or pressure areas. G1 contact/load feedback here is simulation-only.
All three actors and recorders exited; no other user's processes were terminated.
