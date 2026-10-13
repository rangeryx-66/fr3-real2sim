# Frozen native G1 held-out doors — interim status

Reference `a022bfe`. **ORACLE_HANDLE / KNOWN_MODEL / HELD_OUT_EVALUATION / XARM7_NATIVE_G1 / APPROXIMATE_ACTUATOR_SIMULATION**. This evaluation remains in progress: four valid physical episodes completed, two cases undergoing initialization repair/retry, four later cases reserved. **No final six-case physical success rate is available.**

## Registration and unchanged skill

[Ten-case manifest](REGISTERED_OBJECTS.json), commit `b7f9039`, was published before any IK call. Selection used metadata and closed-state geometry, excluding previously registered/planned/manipulated assets. The set spans vertical/horizontal straight/curved handles, authored object heights 0.4–1.0 m, hinge arrangements and rear clearances. No lock/spring requirement was found in the authored single-axis models; this is a model eligibility statement, not hardware inspection.

Each case uses fresh geometry-derived grasps, a common canonical scene seed, automatic base/wrist/branch search and `wall_s=300`. No saved successful plan, manual pose or per-object controller is used. Planning coverage is **10/10 candidates; 9/10 selected plans cover 30°**. The selected 45717 plan reaches 27° then `NO_CONTINUOUS_PINK_IK`; it remains registered and eligible for its one physical attempt. Full future reachability is not added as a fatal execution gate. 45916's final in-progress check overshot the budget argument: actual planning wall time 312.35 s. The unchanged planner is not retroactively patched to hide this.

All ten generated execution bodies have the same SHA256 `9c4132d61f88487e9282f2ee324b82f017c4f60b633bd633fe84cafa72140165`. The loaded entry is the exact frozen `run_xarm7_native_transfer_repair.py` (SHA256 `c08fb22edcf6cffdddef32c3ddaf37a8123f6f7f59025d5e785e8305b97065e9`); native configuration remains `68fb6af5f4efd432b2355cabe37876a67f6c5ea3eac0b224603f08fb18d62939`. Grasp scoring, closure/position preload, approach/Pink control, limits, collisions, gains and physics are unchanged.

## First-six interim evaluation

| Order | Asset / handle | Geometry | Planned | Physical grasp | Actual after valid grasp | ≥30° | Status / first divergence |
|---:|---|---|---:|---|---:|---|---|
| 1 | 45526 / original-30 | Vertical straight bar | 30° | Not tested | — | Not reached | Initialization input mapping: expected l_1, actual moving visual link l_0; repaired, first valid physical episode in progress |
| 2 | 45638 / original-48 | Vertical short bow | 30° | Not tested | — | Not reached | Initialization input mapping: expected l_1, actual moving visual link l_2; repaired, awaiting sequential launch |
| 3 | 45667 / original-32 | Low horizontal straight bar | 30° | Yes, correct handle | **30.000259°** | **PASS** | No recorded physical divergence |
| 4 | 45623 / original-33 | Vertical long bow | 30° | No | — | FAIL | Bilateral hold not established; pre-grasp excursion 0.002695° is not task progress |
| 5 | 45717 / original-35 | Low horizontal short bow | 27° | No | — | FAIL | Bilateral hold not established; pre-grasp excursion 0.002129° is not task progress |
| 6 | 45850 / original-41 | Low-cabinet short vertical bar | 30° | Yes | **30.000088°** | **PASS** | No recorded physical divergence |

Planning failures remain in the registered denominator (currently none). Initialization incidents are preserved separately and are **not consumed physical manipulation attempts or physical grasp failures**, following the user’s explicit protocol amendment. The first two actors returned OS code 0 after automatic app shutdown despite logged `RuntimeError`; no task video or 240 Hz task stream was generated. Reported orchestrator wall estimates are 68.01/18.00 s, measured from reservation to process-exit metadata timestamp, not runner timings. Simulation/physical force/angle metrics are unavailable, not zeros.

Remaining four: 45671/original-35 (horizontal long bow), 45783/original-49 (high horizontal short bow), 45908/original-37 (vertical long bar), 45916/original-46 (vertical short bow). All selected plans predict 30°; all remain **RESERVED_NOT_EXECUTED**. They are not part of this round's six physical attempts.

## 45667 physical evidence

Normal closed start; fixed base/cabinet; one grasp, no intentional releases or regrasps, restart/stitching, attachment or object actuation. Grasp acquisition: t=139.075 s, angle 0.060756°, L/R loads 0.475/0.541 N. This small acquisition excursion is recorded, not concealed. Maximum angle after valid grasp 30.000259°, final 30.000260°.

Peak pad load 0.914 N; translation/orientation drift 2.456 mm/0.186°; minimum margin 0.389237 rad; maximum loaded TCP speed 1.651 mm/s, all-phase speed 24.994 mm/s (pre-grasp). Maximum joint reference error 0.004643 rad. No recorded force-window warning/overload or both-filtered-pad unloading interval. Non-handle proximity/contact-pair entries have zero reported impulse/force throughout; pair counts alone are not evidence of loaded collision. No new diagnostic gates are applied.

Simulation 472.017 s; reported runner wall 3015.054 s. 113284/113284 observations/native physics records, supported telemetry fields present every step, no reported export issue. Continuous video: 472.000 s, 14160 frames, 1280×960, fully decoded without errors. Actor exited normally; GPU resources were released. Raw contacts are normal point impulses; friction anchors are not captured by the frozen interface, and reported pad loads are not calibrated hardware total forces.

## Integration amendment and current blocker

This is **not a clean unchanged-input batch**. The new input export first failed identifier association before IK; a uniform identifier-only repair retained geometry/transforms and all error logs. Later, the preparer’s old microwave `door_link=l_1` default caused the first two official initialization failures. The user explicitly authorized initialization repair and retry without consuming a physical manipulation attempt. While 45667 continued untouched, only not-yet-reserved metadata was corrected from each authored moving visual link, without changing geometry, plans, station/grasp poses or frozen control. Scheduling resumes only independent remaining cases; it does not manually continue a scene. See [full amendment](INPUT_INTEGRATION_AMENDMENT.md).

GPU 2 was empty before the initial sequence. The old coordinator waited for exclusive GPU availability, then completed cases 4–6 after resources became available. That restriction has now been removed under explicit user instruction. Scheduling uses actual free VRAM, permits other users’ processes, and keeps only our own actors sequential. Cases 1–2 retain their original initialization evidence under `initialization_01`; their moving-link metadata was repaired without re-planning. Case 1’s first valid physical episode is running, followed by case 2. Completed valid physical episodes are not repeated.

Development comparisons remain separate: 45134 previously 30.000061° (its physical plan preceded the scoring repair; offline regression is compatible), 45443 30.000179°, 45249 30.000196°. They are not pooled into held-out outcomes. Current completed evidence proves transfer to low horizontal and vertical straight-bar instances; it does not establish held-out robustness.

GT handle/articulation/collision geometry and dynamic object-state feedback remain execution dependencies; actuators and mount are approximate simulation models. No visual perception, unknown-axis inference, fitting or hardware equivalence is claimed. Private evidence root: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_heldout_20261013`; local successful video: `engineering_artifacts_20261013/native_g1_heldout/45667/contact_baseline.mp4`. Code, compact result tables and evidence hashes are public; raw telemetry, assets and videos remain private.

Evidence indexes: [offline pre-physical snapshot](OFFLINE_EVIDENCE_INDEX.json), [completed official attempts 1–3](COMPLETED_EVIDENCE_INDEX.json). These are timestamped-stage snapshots of paths, byte counts and SHA256 values, not embedded raw data. The private completed-three archive contains 605 entries, 367826560 bytes, SHA256 `afd8b58837c1debb40e104e0f32066d1119d0fdbad38c6c9e51afad1ed6f2a6b`; it is not a completed six-case archive.
