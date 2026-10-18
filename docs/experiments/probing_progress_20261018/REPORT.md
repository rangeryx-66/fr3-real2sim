# Probing progress clock and segment completion

Reference: `7548aeb`. Scope: shared controller/interface repair, archived replay and short FAST_DEV engineering tests. **No full Agent episode or new large-range identification claim.** Original V3/V4 outcomes remain unchanged. Isaac results are KNOWN_MODEL / APPROXIMATE_ACTUATOR_SIMULATION; the contact fixture was originally planned automatically from measured RGB-D, but was reused here to isolate control rather than test fresh perception.

## Cause and shared repair

The archived V4 recovery segment ran 44.996 simulated seconds and reached 5.81055° (5.73745° gained). Its median reference lag was 0.220232°. The old clock divided its angular rate by `1 + lag / 0.05°`: median divisor **5.40465**. The separately passed **0.5° overshoot** limited the terminal reference; it did not replace the default 0.05° lag divisor. Thus ordinary tracking lag progressively suppressed the intended velocity/acceleration program. The archived reference reached only approximately 6.11°.

The speed conversion also used distance to the hinge origin, including axial displacement: approximately 0.52844 m versus the perpendicular hinge radius **0.42433 m**. This independently understated angular speed by approximately 20% for the same tangential speed. The replay reproduces legacy clock updates to 6.12e-6° RMS on the long V4 segment; stopped/decelerating samples are not incorrectly counted as clock updates. See [archive replay](ARCHIVED_REPLAY.json).

The active HANDLE and PANEL paths now share `Operations.rotate_segment` and `ProgressClock`:

- Convert tangential speed with `|| (TCP − hinge_origin) × unit_axis ||`.
- Advance the declared bounded speed profile without a normal-lag divisor. Preserve angular and linear acceleration/speed bounds.
- Smoothly brake as the reference approaches the existing explicit half-degree lead allowance; a blocked measurement cannot accumulate an unlimited reference. This allowance is a progression control parameter, not a new safety gate or hardware validation.
- Retain the existing measured slip-versus-progress transmission stop and original force/collision/joint/actuator supervisor. No force, gain, effort, collision, closure or estimator-mathematics changes.
- Check completion **after checked deceleration using measured object state**. The first short robot test exposed target-crossing recoil; continuing the same reference within the remaining budget fixes premature completion. No grasp-reference reset hides slip.

The frozen legacy clock remains available for reproducing prior results; active shared rotation no longer calls it. Replaying the new clock against unchanged slow archived states still causes braking; it is **not** a counterfactual physical speedup demonstration.

## Segment contract and FIT preservation

Actions may declare `max_sim_s` (default 45), `allow_target_truncation` (default false), and ordered `fit_intervals_s` before execution. A nominal simulation of the declared profile, perpendicular radius and acceleration reserve checks budget consistency. An impossible requested interval returns `NOT_STARTED_BUDGET_INFEASIBLE`; an explicitly shortened interval is reported separately as `SELECTED_INTERVAL_REACHED`, with `requested_target_reached` retained. Nominal timing assumes ideal tracking and is not a contact/IK success guarantee.

Outcomes distinguish `TARGET_REACHED`, `TIMEOUT / SEGMENT_TIMEOUT`, and `TRANSMISSION_FAILURE / CONTACT_TRANSMISSION_STALL`. Safety exceptions remain authoritative. A timed-out action never becomes task success because some samples are useful.

With explicit job opt-in `allow_interrupted_fit: true` (worker flag `--allow-interrupted-fit`), a safely stopped timeout can contribute **only its predeclared FIT windows**, with valid transmission metadata and no stopped samples. Gaps are split before unchanged segment-local SG121 processing. Complete FIT actions also honor declared windows. Labels retain original action status and distinguish `PREDECLARED_SAFE_INTERRUPTED_FIT`. Incomplete VALIDATION, safety/transmission failures, and a clipped VALIDATION that did not reach its original requested target remain excluded. Interval membership uses integer physics-step offsets at the unchanged 240 Hz rate.

Example segment fields (a complete Agent command still requires its normal protocol envelope):

```json
{"role":"FIT_ACCEL_OPEN","target_deg":13,"profile":"constant","hold":true,
 "max_sim_s":45,"reference_speed_m_s":0.012,
 "reference_acceleration_m_s2":0.04,"fit_intervals_s":[[1,12]]}
```

There is no retroactive declaration on old trials. Old V4 still supplies **zero FIT bins**, strict or opt-in. The original failed action/model files are preserved.

## Verification results

| Test | Result | Interpretation |
|---|---|---|
| Deterministic ±13° lag model | 14.9083 s each; max lag 0.23854°; no lead braking | Numerical controller test, not Isaac performance |
| Deterministic stalled measurement | Reference settles at 0.500057°; velocity tends to zero | Finite lead, no uncontrolled advance |
| Archived first failed V3/V4 grasps | Existing transmission stall detected at 1.5 s | Genuine slip response retained |
| Native robot-only TCP arc | +13.1934° in 6.6583 s; return −13.2978° in 6.9542 s; final lag ≈0.213° | **Virtual geometric arc**, real arm actuators; actual door stayed closed |
| Native frozen-feedback fault | `TIMEOUT` at 2.8042 s of 3 s; reference 0.500123° | Fault injection, not a physically jammed door |
| Synthetic interrupted FIT | 90 →150 FIT bins; 90 VALIDATION bins | Known (I,b,C)=(0.3,0.12,0.04), recovered (0.300008,0.119943,0.0400014) |
| Validation mutation test | FIT parameters and covariance bit-identical | Complete holdout isolation retained |

**Actual native finger–handle contact**, one acquisition, original one-second hold, no regrasp:

| Target absolute angle | Measured angle after stop | Nominal / actual sim s | Median reference °/s | Final lag ° | Peak TCP mm/s | Peak relative slip mm |
|---|---:|---:|---:|---:|---:|---:|
| Open to 0.8° at 4 mm/s | 0.811829° | 1.533 / 1.658 | +0.540206 | 0.045741 | 5.079 | 0.375 |
| Close to 0.3° at 3 mm/s | 0.292442° | 1.304 / 1.408 | −0.405105 | 0.029185 | 4.735 | 0.405 |
| Reopen to 1.1° at 3.5 mm/s | 1.108560° | 1.754 / 1.850 | +0.472685 | 0.029806 | 5.916 | 0.412 |

No lead braking occurred during these three contact segments. Their reference rates agree with requested tangential speed at approximately 0.424 m radius. Raw TCP peaks are retained rather than filtered away. Pad-load peaks L/R were 1.351/0.563, 1.108/1.268 and 1.486/0.614 N. All three segments had continuous 240 Hz data (398/338/444 samples). This is short-range transmission/controller verification, **not ≥2° short-task success, 30° capability, or independent dynamics accuracy**.

The contact test safely released and parked in `native_rest`; unloaded pads 0/0 N, measured joint-cell error 0.003722 rad. Final door angle after release was 1.119720°. Total including initialization, original closure, hold and parking: **123.8 simulated seconds / 351.584 wall seconds**; the three actual interaction segments total 4.9167 simulated seconds. Robot-only successful verification cost 67.215 wall seconds / 18.321 simulated seconds, including guard tests and export. These are engineering timings, not a matched end-to-end speedup claim.

## Safety and estimator regression

The unchanged policy SHA256 is `9ef5bd91047d46b56d3ba61b5bb704272bbd302eccbd14fdb3673e05183a8ee9`. Native tests rejected an unsafe joint command, joint velocity command and a real collision route before physics advancement. Finite effort caps read back unchanged. A **robot-tool-only** applied 4.2 N test load produced 4.368 N wrist measurement and continued 48 steps; an 11 N tool-only fault stopped at measured 10.183 N in six steps under `EMERGENCY_WRIST_FORCE`. No test force was applied to the door; these samples never enter identification. Hard stop remained authoritative and the actor exported/closed. Offline tests additionally cover sustained overload, pad overload, actuator authority and phase TCP speed stops.

Maximum recorded command-joint acceleration was 0.8000028 rad/s² robot-only and 0.80000013 rad/s² contact (float32 quantization around the unchanged 0.8 command bound). Unfiltered free-space arc TCP peaks of 20.308 and 10.549 mm/s remain reported; this does not establish exact actuator tracking or hardware safety.

V3.1 mathematical sources, wrench compensation/transforms and SG121 are unchanged. Strict replay on the original V3 partitions reproduces the **entire corrected constant model bit-identically**: I=0.1121266433, b=0.2888320155, C=0.01794941969 (304 FIT /83 VALIDATION bins). This is that exact archived replay packet, not a replacement for any previous reported alternative analysis. The actual asynchronous corrected worker completed on short contact data in 2.957 s; only 12 FIT /9 VALIDATION bins exist, so no identification accuracy is claimed.

After the physical test, FIT-window boundary/label semantics were tightened and replayed into a **new** analysis directory: identical contact bins/parameters and strict V3 model, original physical online packet preserved. Protocol publication remains standard-library-only and passes malformed/concurrent-command regressions. No motion change followed the successful contact test. See [final input check](FINAL_INPUT_REGRESSION.json), [FIT test](FIT_SELECTION.json), [safety tests](SAFETY_OFFLINE.json), [continuity](COMMAND_CONTINUITY.json).

## Provenance, failures and cleanup

Three robot-only launches and two contact-fixture launches are retained separately. Robot-only launch 00 lacked a required physics-config job binding (no physics); launch 01 exposed the measured completion recoil and remains failed. Contact launch 00 lacked `visual_reobservations`, stopping before physical closure; the launcher now explicitly carries/preflights the original visual job fields. Only contact launch 01 executed a physical grasp. No valid physical failure was erased, and no full Agent episode was launched.

Public artifacts contain compact summaries only: [native outcomes and loaded hashes](NATIVE_SUMMARY.json), [final source hashes](SOURCE_HASHES.json), [archived estimator regression](ESTIMATOR_REGRESSION.json), [unchanged V4 selection](V4_QUALITY_REGRESSION.json), [deterministic tests](DETERMINISTIC.json). Loaded-operation/progress hashes match the delivered implementation; final numerical selection hashes are separately recorded rather than rewriting runtime provenance.

Private server evidence: `results/probing_progress_20261018/{native_short_00,native_short_01,native_short_02,contact_short_00,contact_short_01}`. Total task directory was **22,383,302 bytes** at audit; no raw archive, image, depth or video is published. All stored numeric chunks passed hash verification and F/T validity checks. All five owned actor PIDs were verified absent; no unrelated processes were terminated.

Reproduction (offline): run `scripts/verify_probing_progress.py OUTPUT.json`, `scripts/verify_interrupted_fit.py`, `scripts/verify_fast_runtime_v1.py --output OUTPUT_DIR` with repository and `scripts` on `PYTHONPATH`. Archive replay and native fixture scripts require the private recorded inputs; they are explicit engineering runners, not hidden online grasp fallbacks.

**Delivered:** reusable measured-progress clock, budget-aware segment contract and qualified interrupted-FIT selection, with short native contact verification. Large-range Agent probing remains a subsequent independent experiment; no further physics runs are part of this task.
