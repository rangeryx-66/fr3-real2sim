# 45176 contact transmission: one physical trial, 30° FAIL

**Correct-handle grasp succeeded; sustained opening did not.** The normally closed scene reached a native maximum of **1.429023°**, then receded to **1.351082°** while contact slipped. The owner performed the registered cooperative stop and full export. Exactly one new physical attempt occurred; no retry, release, regrasp, base move or scene stitching followed. This round does **not** establish a second successful 30° door skill.

Classification: **ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT**. Oracle handle mesh `original-46`, known hinge/collision geometry and GT dynamic articulation/moving-body feedback remain inputs. No autonomous-perception or hardware claim is made.

| Quantity | Archived 45176 Oracle trial | New single trial |
|---|---:|---:|
| Correct handle / valid grasp | PASS / original-46 | PASS / original-46 |
| Maximum actual angle | 1.306967° **sampled at 1 Hz**; exact maximum unknown | **1.429023° at 240 Hz** |
| Final angle | 1.176065° sampled | 1.351082° native |
| ≥30° | FAIL | **FAIL** |
| Relative position drift | 10.073 mm at last sampled state | 7.959 mm maximum |
| Relative orientation drift | Unknown | 1.1859° maximum |
| Peak raw pad loads L/R | Sampled lower bounds ≥0.469 / ≥1.092 N | 0.732 / 1.136 N |
| TCP speed, all / after valid grasp | Exact maxima unavailable | 15.516 / 1.320 mm/s |
| Joint margin, all / after valid grasp | Sampled loaded minimum 0.17495 rad; exact minimum unknown | 0.08660 / 0.17914 rad |
| Native observations / physics records saved | 0 / 0; buffer lost on SIGINT | **49,145 / 49,145** |
| Simulation / actor wall time | Last live 200.996 s, video 201.500 s / 943.570 s | 204.771 s / 976.599 s |

The old and new maxima have different sampling rates, and the episodes spend different amounts of time after grasp. Their drift/load values do not justify a quantitative robustness-improvement claim. Before valid grasp the new door's maximum excursion was 0.028376°; it is not credited as manipulation. Gain from valid-grasp angle to maximum was 1.405259°.

## Diagnosis and exact new shared change

Archived video, 1 Hz states and saved acquired transform showed initial correct-handle grasp followed by unilateral loading, stalled/receding opening and increasing slip. In 93.1% of saved opening samples, the left pad reported zero load. The old planner's closing rank equated two first distal contact points with a stable grasp. Original native convex hulls also fill the bow handle's visual rear cavity: the registered lower section has about 12.49 mm visual rear ray gap but zero native rear gap. These support ineffective contact transmission as the likely original mechanism; **old native contact points/normals, peak speed/load and exact loss intervals are missing and are not invented**.

New files `handle_contact_quality.py` and `plan_contact_transmission_door.py` add shared pad-footprint ray scoring of local bilateral surface support, flatness, closing-normal alignment and known hinge-opening direction. Both original visual and native surfaces enter the score; first-visible visual normals are oriented toward the casting pad to handle inconsistent/double-sided source winding. The existing native closing/root/non-handle audit and approach/camera/arc checks remain unchanged. Scores are planning evidence, never new execution gates.

The family adds pad-depth-derived insertion offsets and small wrist pitch alternatives, retaining both rolls and multiple handle sections. The first top-score-only pool favored central grasps but reached only 9°; its evidence is retained. Section-diverse selection then found a 30° candidate within a combined 251.31-second offline planning time, below the 600-second budget. The selected pose uses section −0.3, insertion −5.0111 mm, roll 180°, pitch 0° and the existing station `[0.5366970742, −0.7003701286, −0.0999999791, 120.8597011569°]`. It was generated from geometry, not hand-written or copied from a GT grasp template. A prior station was only a base-search seed.

Its predicted bilateral score increased modestly from 0.09250 to 0.09967; minimum approach/arc planning margin was 0.08665 rad. Offline original collision/camera checks reported no failures and maximum path FK error was 0.0347 μm. **This prediction did not translate into contact retention.** Reachable range remained ahead of contact score in the inherited ranking, so the stronger central patch candidate (score ≈0.20983, 9° feasible interval) was not selected. Neither a future-path IK score nor ray-estimated area certifies pressure-bearing contact.

## First actual physical divergence

Contact began at 122.6375 s; valid FORCE_HOLD was acquired at 128.6292 s with filtered left/right loads 0.53353/0.52185 N. The verified closure repair ran 928 unilateral updates with **zero aperture-command reopenings**. The scene used the exact frozen `ee14128` runner, closure, smooth pre-contact handover and Pink opening code; gains, effort, friction, physics, geometry and protections were not changed.

At valid grasp, the positive native contact on each finger was at pad-local **y ≈ 0**, the front edge of the pad interval **−10.022…0 mm**, with x ≈10 mm. Contact normals differed from the pad-plane normals by **8.48° / 5.78°**. The positive-impulse solver points therefore lay at oblique front-edge contacts. This supports poor seating, but is not a measurement of continuous pressure area; independent ray coverage did not predict load retention.

The first left zero-load report was at **135.1833 s / 0.65347°**, with 0.468 mm drift. At **137.1458 s / 0.78783°**, left zero loading became continuous through the stop: **67.625 s**. Left/right positive native-load duty after grasp was **10.35% / 100%**. At the native maximum, 179.3042 s, drift was already 5.239 mm; the door then receded while drift increased to 7.959 mm and right load stayed around 1.08 N. No interval lost both pad/handle contact reports. The failure is loss of effective bilateral transmission, not automatically complete grasp detachment.

Measured TCP versus FK of commanded joints after removing the retained opening-equilibrium offset differed by at most **46.0 μm** (95th percentile 40.7 μm) and **0.00541°**. The robot followed its commands while the handle-relative pose drifted by millimetres. The acquired-frame hinge reference is geometrically consistent; when the grasp cannot transmit motion, virtual reference advancement becomes arm motion/sliding instead of door progress. These records do not establish a unique complete contact-wrench/friction model.

No existing collision, force, loaded 5 mm/s speed or 0.05 rad joint protection triggered. The owner stopped the observed opening plateau/recession and increasing slip via `STOP_AND_EXPORT`, not a new controller assertion. The runner's diagnostic `first_divergence` is null because no guard failed; the physical divergence above is extracted offline. **A mechanical limit was not established.**

## Compatibility, evidence and next blocker

The original frozen 45415 plan still passes its original offline collision/camera checks, has 0.12236 rad minimum sampled planning margin, FK error below 0.05 μm and positive patch score 0.21296. Its source, plans and configuration remain byte-identical. No new 45415 physical run occurred; this preserves existing compatibility, but does not validate newly replanning 45415 with this scorer.

The next concrete blocker is **load-bearing front-edge contact on the native wedge-like handle surfaces**. A later shared-planner change should predict the supporting plane/contact depth at a common physical closing configuration and favor actual interior/opposed load-bearing patches, rather than independent ray coverage and nominal 30° reachability. The filled native rear cavity may constrain such seating; flag that representation limitation explicitly if it prevents a suitable grasp. Do not silently remove/split colliders or claim that all alternative grasps are impossible. No further planner tuning or physical attempt was made after this failure.

All 49,145 records of each main 240 Hz stream and 21,534 closure-command records exported losslessly, with all 28 telemetry fields available and no telemetry issues. Raw impulses, points/normals, collider ordering, finger/TCP/joint states, efforts/reactions, aperture and sensor mapping metadata are preserved. Continuous video is 204.7667 s; `summary_8x.mp4` is an explicitly accelerated preview. Full server evidence: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/45176_contact_transmission_20261011`. Local archive/checksum paths are in `LOCAL_ARCHIVE_VERIFICATION.json`.

The actual runner SHA256 is `a1d6d3a…`, closure `b522bcef…`, trajectory helper `669c4601…`, opening excerpt `08131e64…`; full hashes and executable path are recorded alongside this report. Pretrial source/plans were published in `3fe7cd1`. The post-run audit verified **11,258 frozen files plus all 623 files in the prior `ee14128` archive: no changes**. Actor PID 3524627 exited normally after full export, return code 0. This single-trial round is finished.

![Measured failure evidence and geometric prediction](COMPARISON.png)
