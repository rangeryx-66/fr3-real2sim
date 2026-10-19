# 45415 closure fix — one physical trial

**ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT: PHYSICAL PASS, 30.000082°.** One fresh normal-closed-start episode established the original bilateral/FORCE_HOLD grasp on `original-40`, then opened through real robot contact. Zero releases, regrasps, base/cabinet motion, attachments, artificial object actuation, restarts, stitching or manual continuation.

| Configuration | Valid handle grasp | Actual post-grasp opening | Reopening events | Result |
|---|---|---|---|---|
| Original Benchmark A station / original closure | Yes | 28.990278° | Not reevaluated | Joint-5 margin boundary |
| Improved station / original closure, previous development_01/02 | No | None | 188 | Bilateral hold not established |
| **Same improved plan / physically loaded repair** | **Yes** | **30.000082°** | **0** | **PASS** |

The same-plan failures are the controlled comparison; the older 28.99° case used another station.

The explicit executable was `scripts/run_oracle_door_closure_continuity.py`, SHA256 `5fb2530b0b637f266be51cc2025b5fae7f7f8b139358ee5a59ae57108ef02638`. The loaded active closure source hash was `b522bcef78a1191a4a2a5c756c34d11d7df2e22ddaa5f782be61343947a1704c`. No filename substitution selected the executable. Actual argv, class alias and update-code file were recorded. Runtime executed **22,218 updates and 1,350 unilateral repair branches**, with **zero command reopenings**.

The only control change retained the prior aperture command during unilateral centering: `self.opening = min(self.opening, measured_opening)`. Exact prior grasp/base/IK plans, opening-controller excerpt, gains, effort, physics, geometry, contact/force rules, 0.05 rad margin and 5 mm/s loaded-TCP protection stayed unchanged. [Source/plan proof](INPUT_PROVENANCE.json), [actual loading](active_source_provenance.json), [actual branch execution](closure_fix_execution.json).

| Measured quantity | Result |
|---|---|
| Bilateral/FORCE_HOLD acquisition | 97.975 s; filtered L/R 0.52165 / 0.52930 N |
| Safe actual 30° crossing | 797.125 s; 30.00008201599° |
| Maximum loaded TCP velocity | **1.314707 mm/s**, below retained 5 mm/s |
| Minimum joint margin | **0.130148 rad loaded; 0.122364 rad all phases** |
| Peak L/R pad loads | 0.96935 / 1.25244 N |
| Maximum handle-relative drift | 1.28029 mm / 0.40771° |
| Simulation / actor wall duration | 797.129 / 3472.339 s (~57.87 min wall) |
| Complete 240 Hz streams | 191,311 observations + 191,311 native records; 22,218 closure records |

**Remaining limitations:** left-pad positive native-contact duty after acquisition was **77.7%**, right **100%**. The first missing left report was at **8.542°**; endpoint loads were **0 / 1.044 N**. Overall effective contact and opening persisted, with no interval where both pads lacked positive intended-handle contact. This demonstrates successful manipulation, not stable bilateral contact throughout or universally valid effort estimation.

The real **100.785 mm/s closure-entry transient** remained before contact. The checked archived sweep had 0.557 mm excursion, no existing-model collision and 28.496 mm gripper/object clearance. The overall speed peak was **441.113 mm/s in unloaded PREGRASP**, not the loaded value above. These are disclosed separately; no protection was weakened. See [transient evidence](PRECONTACT_TRANSIENT.png) and [full metrics](PHYSICAL_METRICS.json).

Before physical launch, the exact active class replayed 24,000 archived inputs: 188 → 0 reopenings. Cooperative-marker and real empty-Isaac SIGINT export tests restored all fixture values/types/float bits and sealed video; they loaded no robot/object and consumed zero physical trials. Offline replay was not counted as physical validation. The actual trial's indexed streams match all buffered record counts, with no telemetry errors.

**Preservation:** 11,247 frozen/old files unchanged; one exclusive physical reservation; all physical/self-test processes closed. [Audit](FINAL_AUDIT.json). This exact configuration is [frozen as a one-run capability baseline](VERIFIED_BASELINE_FREEZE.json); reproducibility is not yet established.

**Evidence:** server root `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/45415_closure_verification_20261011`; local verified root `engineering_artifacts_20261011/45415_closure_verification/results`. Continuous video `physical_01/contact_baseline.mp4` (797.100 s), explicitly accelerated preview `physical_01/summary_20x.mp4`, raw native/observation/closure chunks, events, metadata and hashes are preserved. GitHub contains source/configuration, metrics and figures; raw media/data remains on the server and locally.

**Recommended next action:** preserve this success and separately smooth the unloaded approach-to-closure position/velocity equilibrium handover; investigate the later single-pad load shift from saved data. Do not hide the transient or add symmetry/quality gates. No additional experiment was started. GT handle geometry, hinge model, dynamic object feedback and collision models remain dependencies; this is not autonomous/GT-free or hardware execution. [Detailed evidence](DETAILS.md).
