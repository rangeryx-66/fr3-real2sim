# Frozen ten-door benchmark — completed

Reference `d65c10e`; original registration `b7f9039`; frozen skill `a022bfe`. **XARM7 / NATIVE G1 / ORACLE_HANDLE / KNOWN_MODEL / HELD_OUT_EVALUATION / APPROXIMATE_ACTUATOR_SIMULATION**.

**Strict ≥30° physical task success: 4/10 (40%). First-grasp success: 6/10 (60%). Conditional post-grasp ≥30° success: 4/6 (66.7%).** First-grasp success includes the frozen initial closure/centering behavior, not a second closure attempt or regrasp. All ten now have exactly one valid closed-start physical attempt; first-six results are preserved. No new object, replacement, replan, per-case tuning or physical retry occurred.

The user subsequently designated **45908 as a half-success**: correct physical grasp and 29.891745° opening, with the final collision error retained. Supplementary credit is **4.5 points / 10**, not a full-task success rate and not a retroactive change to the registered ≥30° rule. Its raw report still says failure. [User annotation](USER_PARTIAL_CREDIT.json).

## Final physical results

All targets are 30.000°, rather than each object's full authored range. Measured maximum below means maximum after valid grasp; failed closure excursions are shown separately and never counted as manipulation success.

| Asset / registered handle | Geometry | Planned range | Valid first grasp | Max actual after grasp | ≥30° | Outcome / terminal reason |
|---|---|---:|---|---:|---|---|
| 45526 / original-30 | vertical straight round bar | 30° | Yes | 30.000412° | PASS | 30° reached |
| 45638 / original-48 | vertical short curved bow | 30° | Yes | 30.000170° | PASS | 30° reached |
| 45667 / original-32 | horizontal straight round bar, low mounting | 30° | Yes | 30.000259° | PASS | 30° reached |
| 45623 / original-33 | vertical long curved bow | 30° | No | — | No | Bilateral grasp not established |
| 45717 / original-35 | horizontal short bow, low mounting | 27° | No | — | No | Bilateral grasp not established |
| 45850 / original-41 | vertical short straight bar, low cabinet | 30° | Yes | 30.000088° | PASS | 30° reached |
| 45671 / original-35 | horizontal long curved bow, low mounting | 30° | No | — | No | Bilateral grasp not established |
| 45783 / original-49 | horizontal short bow, high mounting | 30° | Yes | 0.810198° | No | Existing progress/transmission policy after one local correction |
| 45908 / original-37 | vertical long straight round bar | 30° | Yes | 29.891745° | No | PARTIAL (0.5); Loaded link2–door-panel contact protection |
| 45916 / original-46 | vertical short curved bow, opposite-side handle | 30° | No | — | No | Bilateral grasp not established |

Planning produced 10/10 executable candidates; 9/10 selected plans cover 30°. 45717's selected plan ends at 27° (`NO_CONTINUOUS_PINK_IK`), but its actual trial failed before grasp; it is not relabeled as an observed IK failure. The nominal planning budget was 300 s each; 45916's last in-progress operation finished at 312.35 s. Both facts remain reported.

| Failure category | Cases | Evidence |
|---|---|---|
| Initial bilateral-contact acquisition | 45623, 45717, 45671, 45916 | One-sided native handle contact throughout the final centering interval; no GRASP_VERIFIED event |
| Existing post-grasp progress/continuation policy | 45783 | Valid bilateral grasp; initial rebound; repeated stall decisions; one very small local correction; no safety-protection event |
| Actual loaded non-handle contact | 45908 | Native positive impulse at link2–original-32 door panel near 29.89°; existing collision guard ends the run |
| Offline planning / IK termination | None as the first observed physical stop | All cases have plans; 45717's 27° plan is a separate planning limitation |
| Force overload / speed / joint-margin termination | None | Source protections retained; these did not terminate the ten official physical episodes |
| Initialization incidents | 45526, 45638 before their valid episodes | Preserved earlier wrong moving-link inputs; repaired and retried before manipulation, consuming zero physical attempts |

No base movement after initialization, intentional release/regrasp, attachment, object-joint actuation, external object force, manual continuation or state stitching occurred.

## Contact acquisition: evidence and next bottleneck

Descriptive geometry groups: straight handles **4/4 grasps, 3/4 tasks**; curved/bow handles **2/6 grasps, 1/6 tasks**. This small selected set is not a population robustness estimate. Curvature and zero collision-proxy rear clearance are not sufficient causal explanations: 45638 has a zero-gap diagnostic and succeeds, and 45783 also establishes bilateral contact despite zero-gap diagnostics.

The four closure failures are not dominated by aperture tracking error. Their final measured opening closely follows the command, but remains much wider than the static score's best common aperture:

| Failed initial grasp | Static best aperture mm | Actual closure-end mm | Command mm | Actual minus static mm | Mean CENTER L/R load N | Closing-normal alignment, loaded side |
|---|---:|---:|---:|---:|---|---:|
| 45623 | 11.330 | 19.552 | 19.519 | 8.222 | 0.061 / 0.000 | 0.931 |
| 45717 | 13.753 | 32.164 | 32.097 | 18.411 | 0.169 / 0.000 | 0.846 |
| 45671 | 11.330 | 24.678 | 24.478 | 13.348 | 0.223 / 0.000 | 0.920 |
| 45916 | 13.753 | 30.170 | 30.140 | 16.417 | 0.056 / 0.000 | 0.870 |

The loaded-side contact centroids remain near the distal pad end: logical pad coordinates approximately **(-11.75, +59.75, -26.00) mm**, with no positive-contact counterpart on the other finger. The closing-normal alignments are 0.846–0.931, whereas successful straight-bar grasps show approximately 1.0 on both sides. 45638 reaches bilateral contact at 17.316 mm versus static 16.175 mm; its recorded normals also align nearly with closing. These are observed contact/geometry differences, not new acceptance tolerances. Contact-manifold points are not pressure-area measurements; strict pad-footprint fractions in the diagnostic JSON are numerically sensitive at CAD boundaries and are not used to infer actual contact area or retroactively redefine success.

The failed static contact scores (0.0079–0.0139) exceed successful 45638's 0.0016. Therefore, maximizing support at some narrow common aperture is not sufficient: that aperture may never be reached under the real first-contact/centering sequence. Rear-gap measurements and actual pad support must be evaluated at the same reachable pose/aperture. [Detailed contact locations, normals and aperture samples](TEN_CASE_CONTACT_DIAGNOSTICS.json).

**One recommended shared improvement for a later development round:** make candidate scoring **closure-path consistent**. Evaluate support at the first collision-constrained aperture and within the existing centering motion, then prefer candidates that obtain opposing distal-pad support there. Penalize candidates whose score depends on a much narrower, unattained aperture or persistent tip-edge contact. Reuse the native G1 geometry/coupling and current closure primitive; this is candidate ranking, not a new force-symmetry/centering gate or increased grip effort. This recommendation is not implemented or physically verified in this evaluation.

Two post-grasp limitations remain distinct:

- **45783:** acquisition occurs at 0.810198° with filtered L/R loads 0.468/0.462 N. During the probe the angle decreases to about 0.636°, so additional post-grasp opening is **0°**. The existing stall rule fires at t=274.579 s and again at 276.588 s. Local correction lasts one 240 Hz step and commands about **5 µm**; contact is already present, so the primitive exits immediately. Maximum drift is only 0.188 mm, maximum loaded TCP speed 0.416 mm/s and peak pad load 0.865 N. This is an early continuation-policy/initial-transmission failure, not evidence of mechanical impossibility, severe slip, overload, IK exhaustion or missing telemetry. The role of initial rebound versus actual sustained force transmission remains unresolved without a separately authorized experiment.
- **45908, partial success:** 29.891745° actual maximum with no release/regrasp. At t=632.358 s the native report records link2 touching panel original-32: **1.6606 N**, separation **−0.0418 mm**. One positive loaded non-handle contact is recorded, then the existing guard stops. Peak handle-pad load is a separate quantity, 1.074 N. Minimum arm margin is 0.726 rad and loaded TCP peak 3.018 mm/s. Actual arm posture differs from the saved endpoint-IK path by up to **0.272 rad**, largest at joint1 near the contact; no >3 mm TCP diagnostic divergence was recorded. Planning coverage does not certify the actual differential-IK elbow trajectory. Do not attribute this collision solely to handle curvature or call it a mechanical limit.

A saved-data kinematic check predicts moving-body deltas from the known URDF axis/origin and the first measured closed pose. Across all ten, position disagreement is below **0.000517 mm** and rotation disagreement below **0.000008°**. This rules out a large dynamic hinge-sign/body-state mismatch in these records; it is an offline diagnostic and never an execution gate.

## Runtime cost

| Asset | Planning wall s | Initialization profile wall s | Episode simulation s | Episode wall s | Sim / wall | Mean wall ms per physics step* |
|---|---:|---:|---:|---:|---:|---:|
| 45526 | 171.548 | 23.620 | 342.133 | 1952.503 | 0.175× | 23.779 |
| 45638 | 259.954 | 24.790 | 657.308 | 3737.989 | 0.176× | 23.695 |
| 45667 | 104.315 | 27.337 | 472.017 | 3015.054 | 0.157× | 26.615 |
| 45623 | 194.674 | 86.337 | 352.221 | 2088.673 | 0.169× | 24.708 |
| 45717 | 169.258 | 26.550 | 180.529 | 1123.670 | 0.161× | 25.935 |
| 45850 | 199.014 | 25.184 | 622.242 | 3841.789 | 0.162× | 25.725 |
| 45671 | 203.169 | 74.643 | 166.183 | 1067.677 | 0.156× | 26.770 |
| 45783 | 268.431 | 25.909 | 276.587 | 1571.864 | 0.176× | 23.679 |
| 45908 | 194.612 | 29.690 | 632.362 | 3710.250 | 0.170× | 24.447 |
| 45916 | 312.351 | 30.126 | 156.692 | 920.248 | 0.170× | 24.471 |

*Includes initialization and export; it is not an isolated PhysX solver measurement. Ten valid episodes total **3858.275 s simulation / 23029.718 s wall (6.40 h)**, aggregate **0.1675× real time**, with **925,986 physics steps**. Planning totals **2077.328 s (34.62 min)**, reported separately. Prior failed initialization launches and resource waiting are excluded from these valid-episode wall durations.

| Asset | PREGRASP sim s | APPROACH sim s | CLOSE sim s | CENTER sim s | Probe sim s | OPEN_CONTINUE sim s |
|---|---:|---:|---:|---:|---:|---:|
| 45526 | 25.958 | 25.167 | 69.921 | 1.150 | 7.042 | 211.625 |
| 45638 | 38.250 | 25.183 | 66.725 | 3.692 | 14.287 | 507.900 |
| 45667 | 41.763 | 25.083 | 70.808 | 0.654 | 18.833 | 313.604 |
| 45623 | 226.637 | 25.083 | 64.479 | 35.521 | — | — |
| 45717 | 54.854 | 25.175 | 47.950 | 52.050 | — | — |
| 45850 | 58.204 | 25.083 | 67.754 | 2.438 | 21.354 | 446.017 |
| 45671 | 40.600 | 25.083 | 58.921 | 41.079 | — | — |
| 45783 | 181.742 | 25.083 | 61.454 | 3.508 | 3.508 | — |
| 45908 | 20.371 | 25.083 | 68.954 | 0.200 | 12.804 | 503.675 |
| 45916 | 31.108 | 25.083 | 53.862 | 46.138 | — | — |

The JSON contains every phase, minor settling/preload phases, and command-changing versus command-stationary durations. These describe issued commands, not inferred physical rest. Most simulated time is continuous opening (51.4%), pregrasp motion (18.6%) and closure (16.4%); centering is 4.8%. Command-stationary samples total about **16.98 s** across all ten, so arbitrary dwell removal is not supported by this record. The 100 s closure/centering budget is fully consumed on the four grasp failures; unsuccessful geometry can therefore consume substantial simulation time without achieving bilateral contact.

Initialization totals 374.186 s (1.63% of valid runner wall). Even an upper bound including the last progress interval, video finalization, buffered export and result writing totals only 124.741 s (0.54%). More than **97.8%** of runner wall remains within the stepped execution segment. Buffered serialization/export is not the main wall-cost driver in these records.

The saved subsystem timers cover only each initialization hold, because the adapted runner never flushed their accumulated end-of-episode values. In that measured subset, physics/native callbacks take **9.716 s**, rendering **4.041 s**, camera/video reads/writes **2.646 s**, passive telemetry **0.953 s**, and **9.188 s is unaccounted** within 26.544 s of timed steps. The unaccounted section includes state/FK/contact/collision checks, action application and other Python work. Physics/native callback time is the largest individually measured component, but these percentages cannot establish the full-episode physics-versus-render-versus-Python split. No extra physics profiling episode, timing change or fidelity reduction was performed.

Per-phase wall time beyond initialization is unavailable for the original six cases. The last four have an external observer of the existing progress file. Its one-simulation-second update intervals provide wall-time bounds or censored lower spans, not exact per-phase timings; short phases can be missed. For example, 45783 APPROACH is 127.8–139.3 s wall, CLOSE 409.3–422.3 s; 45908 OPEN_CONTINUE has at least 2897.9 s of directly observed wall span. [Complete runtime analysis and bounds](TEN_CASE_RUNTIME_ANALYSIS.json).

## Concrete Oracle Handle → SAM3 + RGB-D plan (not executed here)

1. **Keep this frozen oracle skill and all evidence intact.** Create only a thin visual input adapter to the native G1 grasp generator. Do not modify closure, Pink, gains, force/speed/joint protections or physics.
2. **Acquire actual calibrated simulated RGB-D at the normal closed start.** Save RGB, optical-Z depth in metres, intrinsics, camera-to-world transform and robot FK state. Current benchmark videos are RGB recordings, not a substitute for saved depth. Use the existing fixed overview first and label it as such; wrist sensing requires its own correct xArm tool/camera transform. No GT instance mask or handle-index cropping.
3. **Run the existing SAM3 mask and RGB-D handle estimator.** Inputs are RGB/depth/calibration only. Generate bar/curved-handle hypotheses from the masked cloud and surrounding observed panel plane. Explicitly record hidden-backside/thickness priors. Never use the asset manifest's handle geometry, GT pose or saved successful template to generate or shift a candidate.
4. **Use official native G1 geometry and actual nonlinear coupling.** Replace PiPER-specific q slicing, pad dimensions and tool offsets in the old visual adapter with the existing xArm7/native-G1 model. Compute observed contact support at a common pose/aperture; candidate scoring uses observed surfaces and declared priors, not oracle handle meshes.
5. **Preserve the source boundary.** Freeze visual candidate poses before known-model IK/collision filtering and station search; those checks may select among visual candidates but do not correct them toward GT. Initially retain known hinge, dynamic progress and global collision geometry for opening/protection, explicitly labeled **VISUAL_GRASP / KNOWN_ARTICULATION**, not fully GT-free execution. Simulator contact pairing remains a declared dependency.
6. **Shadow first, then pregrasp re-observation.** Compare poses/contact accessibility against Oracle Handle offline while its controller is unchanged. Use measured robot FK for self-occlusion exclusion. Re-observe before closure and refine with common-visible RGB-D registration; log residuals without creating convergence or force-quality gates. If perception fails, re-observe or return an explicit visual failure; no silent GT grasp fallback.
7. **Next separately authorized bounded physical check:** pre-register 45526 (straight positive), 45638 (curved positive despite zero-gap diagnostic), and 45623 (unilateral-contact control). Freeze one visual adapter/configuration, then test first closure and a short opening before the 30° objective. Preserve all failures and camera perturbations; no per-object hand-tuned poses. This is a proposal only—no visual integration or extra episode occurs in this round.

## Provenance, initialization repairs and private evidence

[Continuation protocol](CONTINUATION_PROTOCOL.md) and [last-four freeze](LAST_FOUR_FREEZE.json) preserve the existing plans. Actual entry SHA256 stays `c08fb22edcf6cffdddef32c3ddaf37a8123f6f7f59025d5e785e8305b97065e9`, generated execution body `9c4132d61f88487e9282f2ee324b82f017c4f60b633bd633fe84cafa72140165`, native config `68fb6af5f4efd432b2355cabe37876a67f6c5ea3eac0b224603f08fb18d62939`. Every frozen source hash remains unchanged; the new code is evaluation/analysis orchestration only. Fresh actor source provenance is archived per case.

This is **not an entirely unchanged-input batch**. Before valid planning, collider identifiers were normalized uniformly without geometry changes. Later the old preparer's `door_link=l_1` default caused pre-physics initialization errors on 45526/45638. Their original failed inputs/logs remain under `initialization_01`; moving visual link mappings were corrected to l_0/l_2 and retried under explicit user authorization. Other not-yet-launched mappings were corrected generically beforehand. The last four matched that corrected input freeze and had **no new initialization failure or repair**. No completed physical scene was restarted or stitched.

GPU scheduling uses available VRAM, not idle/exclusive ownership. All owned actors ran sequentially and were closed after complete exports; no other user's process was terminated. GT handle geometry, known articulation/dynamic body feedback and collision models remain dependencies. Official native G1 geometry/mount is used with approximate simulation actuation; no hardware-equivalent dynamics, autonomous perception or physics identification is claimed. Inherited runner report labels saying development are retained verbatim; this pre-registered set is the held-out evaluation.

All 925,986 observations and native physics records were decoded and matched their indexes/reports. All ten continuous videos decode without errors. Supported 240 Hz contact/joint/robot/gripper fields and sensor conversion metadata are preserved. Contact impulses are normal point impulses; friction anchors are absent, so normal projections are not calibrated total interaction forces. Missing acquired-grasp drift/loaded-speed metrics on grasp failures are unavailable, not zero.

[Final strict results](FINAL_TEN_RESULTS.json), [case metrics](TEN_CASE_METRICS.json), [integrity audit](TEN_CASE_EVIDENCE_INTEGRITY.json), [all-ten evidence index](TEN_CASE_EVIDENCE_INDEX.json), [last-four private archive index](LAST_FOUR_EVIDENCE_INDEX.json), [private archive summary](LAST_FOUR_ARCHIVE_SUMMARY.json). Server root: `/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_heldout_20261013`. The earlier first-six report and private archive remain unchanged; this last-four archive supplements them. The public case-metrics file contains summaries and omits raw single-contact vectors, points and normals; archive indexes refer to the full private original. Raw assets, telemetry, snapshots and videos are private, with no public Release upload. **Stop after the four registered cases.**
