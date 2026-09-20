# Agent skill handoff (v1)

This is the integration contract for an agent that invokes the existing **Grasp**, **ScanStation**, **MV-SAM3D**, **PayloadID**, and **AssetExport** skills. It describes actual artifacts and safety gates. `scripts/make_handoff.py` emits a normalized read-only handoff JSON for the first four stages; it does not move the robot or change a model. Source artifacts remain authoritative.

For PayloadID implementation and reproduction, including the TCP Jacobian origin fix, corrected `dq` semantics, actual-opening empty baseline, per-pose Drake q-mismatch compensation and COM confidence gate, read [`FR3_PAYLOAD_COM_GUIDE.md`](FR3_PAYLOAD_COM_GUIDE.md) before running a capture. The historical estimator-only entry point is not the final q-compensated production path.

## State and ordering

```text
GRASP_STABLE ──(physical place/release/park verified)──> STATION_PLACED
      │                                               │
      └──> PAYLOAD_IDENTIFIED / PAYLOAD_FALLBACK      └──> SCAN_ACCEPTED
                                                        └──> VISUAL_ONLY_MV_SAM3D
                                                              └──(measured metric alignment + inertial + collision)──> ASSET_READY
```

PayloadID must run **while the object is still stably held**. A station scan happens **after placement and release**. The current station runner initializes an object in a clean scene; it does not physically transfer the object from a prior grasp. Do not assert continuous object identity between these branches without an independently recorded placement/identity handoff. The MV-SAM3D raw GLB is `VISUAL_ONLY_MV_SAM3D`, never `ASSET_READY` by itself.

Every handoff has `schema=fr3_agent_handoff/v1`, `stage`, `target_id`, `source_artifacts`, `checks`, `next_allowed`, and `created_utc`. `source_artifacts` values are absolute paths and SHA-256 digests where possible. Keep one run directory per episode; never use a result file from another seed or object. Upstream JSON is untrusted input: the agent must inspect status/checks rather than treating the existence of a file as completion. The wrapper returns nonzero on failed gates and leaves the prior stage unchanged. A handoff is a snapshot, not a command or permission to move hardware.

Frame convention: all transforms are 4×4 homogeneous matrices, meters, column vectors, `T_A_B` maps coordinates from B to A. `fr3_link0` is robot base, camera is optical (x right, y down, z forward), `fr3_hand_tcp` is TCP; ROS quaternions are xyzw, Isaac API commonly uses wxyz. The grasp result's `T_base_camera` and `src/frames.py` are the sources of truth. Do not infer an object frame from filename or asset category.

## Skill contracts

| Skill | Required input | Gate and resulting handoff | Output consumed next |
|---|---|---|---|
| `Grasp` | `target_id`, scene seed, Arena asset/protocol, AnyGrasp SDK path/issued license, frozen MoveIt config | Actual result JSON has `success=true`, `category=SUCCESS`, flags `PICKED`, `RETAINED`, `CLEAR_TABLE`, `lift_ge_8cm`, no `DROP`, and `final_support.passed/currently_clear=true`. Production mode is `ANYGRASP`; oracle/GT is diagnostic only. | `GRASP_STABLE` with result path, selected rank, stable flags. Use for PayloadID or physical placement. |
| `ScanStation` | The placed and released object, arm parked, scan camera, target identity, output directory. For a physically connected episode also require a placement record from the agent/robot; the current runner does not create one. | `scan_manifest.json`: schema `fr3_scan_station/v1`, `state=placed_released_arm_parked`, accepted frames ≥ requested QA minimum, every selected frame has RGB/depth/object/gripper masks, camera intrinsics and `T_B_camera`. `eval_gt/` never enters reconstruction. | `SCAN_ACCEPTED` with scan manifest and `identity_continuity` (`VERIFIED` only with an external physical placement record; otherwise `INDEPENDENT_SCENE`). |
| `MV-SAM3D` | `SCAN_ACCEPTED`, `selection_manifest.json`, official model checkout/weights, selected 2/4/8 view folder. Default eight. | Selection target and view count match scan target; selected masks are nonempty; official inference produces `result.glb`. Save chosen frame IDs and adjacent overlap. | `VISUAL_ONLY_MV_SAM3D` with GLB path, view count, `metric_scale_verified=false`, `physics_ready=false`. |
| `PayloadID` | `GRASP_STABLE`, while still holding object; `capture.json` payload and matched empty windows, q/dq/torque timestamps, actual opening and `T_TCP_object`. | Static q-FD speed `<0.002 rad/s`, settled torque, matching opening, current free-space bilateral contact, no cross-pose relative slip. At least four paired poses; estimator `fit.accepted=true` and mass/COM uncertainty gates pass. | `PAYLOAD_IDENTIFIED` with mass/COM frame, uncertainty and source. Otherwise `PAYLOAD_FALLBACK`: report explicit reason and use geometry COM/inertia; never silently substitute GT. |
| `AssetExport` | Metric-calibrated visual mesh and measured texture, collision manifest, inertial JSON in same object frame, source/provenance. | Require measured RGB-D metric report before trusting MV-SAM3D geometry; run CoACD, build USD, then fresh Isaac reload/physics readback. | `ASSET_READY` only after reload validation. The current repository does not implement a validated one-command raw MV-SAM3D GLB → metric USD transform. |

### Existing file locations

- Grasp: `<episode>/<target>_seed<seed>_ANYGRASP.json`, raw AnyGrasp candidates beside it. See `calibration/unseen_backend.py`.
- Static scan: `<scan_run>/scan_station/scan_manifest.json`, `rgb/`, `depth/`, `masks/`, `gripper_masks/`, `poses/`; the station summary is `<scan_run>/scan_station_run.json`.
- View selection: `<mv_run>/inputs/selection_manifest.json`, `views_08/selection.json` and `input_mask_overlay.png`. The official model writes a timestamped `result.glb` path to `inference.log`.
- Payload: `<episode>/payload/capture.json` and `<episode>/empty/capture.json` plus `.npz`; cross-object evaluator writes `<batch>/summary.json` with object → `methods.drake.fit`. `GT_residual` and `COM_error_mm` in evaluation summaries are **audit only** and must never be an agent selection criterion.
- Asset: `collision_manifest.json`, inertial JSON (`mass`, `center_of_mass`, `inertia_matrix`), USD and `metadata.json`; `real2sim/validate_asset.py` handles reload.

### Commands to emit handoffs

```bash
python scripts/make_handoff.py grasp \
  --result results/episode/soup_seed1031_ANYGRASP.json \
  --output results/episode/handoff_grasp.json

python scripts/make_handoff.py scan \
  --manifest results/station_soup/scan_station/scan_manifest.json \
  --output results/station_soup/handoff_scan.json

python scripts/make_handoff.py mv \
  --scan-handoff results/station_soup/handoff_scan.json \
  --selection results/station_soup/mv_sam3d/inputs/selection_manifest.json \
  --glb /absolute/path/to/result.glb \
  --output results/station_soup/handoff_mv.json

python scripts/make_handoff.py payload \
  --grasp-handoff results/episode/handoff_grasp.json \
  --summary results/fr3_payload_fresh_gate/summary.json \
  --output results/episode/handoff_payload.json
```

`scan` without a physical placement record emits `identity_continuity=INDEPENDENT_SCENE`; the agent may inspect/reconstruct that scene but must not claim it scanned the same object just grasped. If an agent does physically place/release the object, its own placement record should contain `target_id`, `released=true`, `arm_parked=true`, `object_stationary=true`, pose/frame, timestamp and run ID, and it should verify that record against scan target before marking continuity. The wrapper does not fabricate this evidence.

## Fail closed and retry behavior

- `NO_GRASP`, `NO_EXECUTABLE_CANDIDATE`, `NO_IK`, `NO_PLAN`, `APPROACH_FAIL`, `BAD_CONTACT`, `UNSTABLE_GRASP`, `DROP`: do not hand off to scanning or PayloadID. The frozen grasp executor owns its bounded candidate/regrasp budget.
- `CONTACT_LOSS`, `RELATIVE_SLIP`, support/table contact or invalid quasi-static torque window: abort that PayloadID capture, keep raw logs, try another already allowed stable capture/seed. Never weld the target in production or increase force merely to make a gate pass.
- `SCAN_QA_FAIL` or missing RGB/depth/mask/pose: reacquire the scan; do not send invalid frames to MV-SAM3D.
- `MV_INFERENCE_FAIL` or insufficient/incorrect views: keep the scan, rerun view preparation/inference. Do not select by GT mesh quality.
- `MASS_COM_UNCERTAIN`: write explicit `GEOMETRY_FALLBACK` COM/inertia, preserve identified mass only if its own acceptance conditions pass.
- `METRIC_ALIGNMENT_MISSING`, `COLLISION_FAIL`, `USD_RELOAD_FAIL`: no `ASSET_READY` state. Retain the visual GLB and diagnostic artifacts for the agent.

Agent-level retries must be bounded and logged with `run_id`, seed, parent candidate, actual parameters and reason. Do not silently mutate grasp force, friction, mesh filters, thresholds, AnyGrasp score, or source assets between repeats. Mark a material parameter change as a new experiment/config version.
