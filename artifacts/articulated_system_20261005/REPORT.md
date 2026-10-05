# Articulated collection and ArtGS integration

Official ArtGS commit: `7c1f41be2cb8b96abca13c6a9668dcf7e06d8c2a`. Independent environment. Scene-specific optimization; official demonstration weights are never used as target meshes/axes.
Successful physical grasp/contact baseline remains unchanged. New capture/orchestration outputs preserve failed attempts. Collection failure does not cancel reconstruction.

## Actual reconstruction runs

| Pair | State | Type | Train views | Independent views | Held-out RGB RMSE | IoU |
|---|---|---|---:|---|---:|---:|
| 45746_large | COMPLETE | p | 2 | False | 0.04159628413617611 | 0.9957340657711029 |
| 45746_small | COMPLETE | p | 2 | False | 0.03030636813491583 | 0.9971042573451996 |
| 7320_large | COMPLETE | p | 2 | False | 0.12252744287252426 | 0.9497661292552948 |
| 7320_large_interaction_prior | COMPLETE | r | 2 | False | 0.12229462713003159 | 0.9532086849212646 |
| 7320_small | COMPLETE | p | 2 | False | 0.10398244857788086 | 0.9758584499359131 |
| 45746_large_multiview | COMPLETE | p | 7 | True | 0.12916883690790695 | 0.9516468969258395 |
| 45746_large_multiview | FAILED |  | 7 | True | None | None |
| 45746_small_multiview | COMPLETE | p | 7 | True | 0.12323223562403159 | 0.9519456733356823 |
| 7320_small_multiview | COMPLETE | p | 6 | True | 0.16866080649197102 | 0.8510758355259895 |
| 7320_small_multiview_interaction_prior | COMPLETE | r | 6 | True | 0.15764544252306223 | 0.8563020899891853 |
| 45746_large_multiview | COMPLETE | p | 6 | True | 0.1055187419988215 | 0.8113566190004349 |
| 45746_small_multiview | COMPLETE | p | 6 | True | 0.10644091991707683 | 0.8175012096762657 |
| 45746_small_multiview | COMPLETE | p | 7 | True | 0.12853571358654234 | 0.8263300591044955 |
| 7320_large_multiview | COMPLETE | p | 7 | True | 0.21052202582359314 | 0.7484177350997925 |
| 7320_large_multiview_interaction_prior | COMPLETE | r | 7 | True | 0.21190220365921655 | 0.7444882061746385 |
| 7320_small_multiview | COMPLETE | p | 7 | True | 0.20233807091911635 | 0.6963170170783997 |
| 7320_small_multiview_interaction_prior | COMPLETE | r | 7 | True | 0.20934852957725525 | 0.7073286639319526 |

Type-prior results use only the existing measured-EE identification of joint family. ArtGS still estimates axes and geometry. They are explicitly not blind ArtGS type prediction.
Held-out state is not used in reconstruction. Its measured articulation label supplies rendering phase: conditional geometric prediction, not autonomous physics prediction. Old two-view train-view render checks are not independent validation.

## Physical collection and recovery

| Run | Asset | Captured states | Physical stop | Completed release/reposition/regrasp | Actual final displacement |
|---|---|---|---|---:|---:|
| 45746 | 45746 | [] meters | IMPLEMENTATION_ERROR:'NoneType' object has no attribute 'get' | 0 | 1.5384252037620172e-05 |
| 45746_recovery_v2 | 45746 | [0.0, 0.0102, 0.02034, 0.03034] meters | ISAAC_NATIVE_PROCESS_CRASH | 0 | None |
| 45746_recovery_v3_infrastructure_retry | 45746 | [0.0, 0.0102, 0.02034, 0.03034, 0.04119, 0.05141, 0.06166, 0.06166] meters | RELEASE_CONTACT_NOT_CLEARED | 0 | 0.06009930372238159 |
| 45746_recovery_v4_mode_reset | 45746 | [0.0, 0.0102, 0.02034, 0.03034, 0.04119, 0.05141, 0.06166, 0.06166] meters | RUNNING_OR_REPORT_MISSING | 0 | None |
| 45746_sensorfix | 45746 | [0.0] meters | CAPTURE_OBJECT_MASK_EMPTY | 0 | 0.010086823254823685 |
| 45746_visibleviews | 45746 | [0.0, 0.0102, 0.02034, 0.03034, 0.04119, 0.05141, 0.06166, 0.06166] meters | NO_SAFE_RELEASE_RETREAT_IK | 0 | 0.06011628732085228 |
| 7320 | 7320 | [] degrees | IMPLEMENTATION_ERROR:'NoneType' object has no attribute 'get' | 0 | 0.0075831825079469475 |
| 7320_recovery_v2 | 7320 | [0.0, 2.58575, 5.09994, 7.62325, 10.12454, 12.66411, 15.1843, 17.71887, 18.58075] degrees | IMPLEMENTATION_ERROR:'candidate_grid' | 0 | 18.672504341669672 |
| 7320_recovery_v3_legacy_adapter | 7320 | [0.0, 2.58575, 5.09994] degrees | SUPERVISED_INFRASTRUCTURE_REPLAY | 0 | None |
| 7320_recovery_v4_mode_reset | 7320 | [0.0, 2.58575, 5.09994, 7.62325, 10.12454, 12.66411, 15.1843, 17.71887, 18.58075] degrees | RUNNING_OR_REPORT_MISSING | 0 | None |
| 7320_sensorfix | 7320 | [0.0] degrees | CAPTURE_OBJECT_MASK_EMPTY | 0 | 2.6786612408180734 |
| 7320_visibleviews | 7320 | [0.0, 2.58575, 5.09994, 7.62325, 10.12454, 12.66411, 15.1843, 17.71887, 18.58075] degrees | IMPLEMENTATION_ERROR:'candidate_index' | 0 | 18.672504341669672 |

## Interpretation and provenance

- Mesh and joint outputs are inferred from actual sensor observations; dataset reference geometry/axes are not substituted. Poor geometry, wrong joint classification and empty meshes remain failures.
- Meter scale and ROS-optical/OpenGL camera transforms are preserved in `input_provenance.json`. Different states remain separate; robot pixels are excluded with simulator instance masks, explicitly not real-world segmentation.
- URDF limits are bounded preview/observed ranges, not recovered full joint limits. Preview mass/inertia are neutral priors, not measured physics.
- `reconstructed_preview.mp4` drives the newly inferred twin joint in an independent Isaac scene. It demonstrates import/assembly only, never robot contact success.
- Effort measurement requires verified normal+friction buffers. Unverified data remain command proxies. Onset windows are task-opening resistance, not exact minimum friction; two finger clamping loads are not added as pull force.
- No friction model fitting, collision geometry changes or safety threshold changes are performed.

## Files

`reconstruction_comparison.csv`, `collection_status.csv`, per-run logs/provenance and per-twin `reconstructed_parts/`, `reconstructed.urdf`, `effort_profile.json`, `state_observations/`, `twin_update.json`.
Missing/failed components are not declared complete. State observation symlinks must be dereferenced when transferring twin packages.
