# Informative probing V3 — registered before physical outcomes

One fresh continuous development episode on **45936**, based on `6f7b74d`.
Label: AGENT_WRIST_RGBD / KNOWN_HINGE_STATE / IDEAL_LOAD_CELL / APPROXIMATE_ACTUATOR_SIMULATION.

Fresh wrist RGB-D and actual Codex image inspection; at most four view adjustments. Registered station and known articulation state remain dependencies. No archived grasp/view replay or hidden dynamics input. Maximum two acquisitions in one unchanged scene, with the existing safe release/park before any recovery decision. No physical retry after completion.

Freeze the shared implementation before launching. Preserve all original closure, one-second hold, Pink, grip authority, collision/joint/phase-aware force and motion policies. Existing LOADED reference caps 12 mm/s and 40 mm/s² remain ceilings. No object actuation.

## Excitation opportunity

V2 measured variable-opening velocity P10–P90 was only 0.01028–0.01068 rad/s. Successful FIT velocity/sign correlation 0.9897, scaled condition 72.21. Most opening acceleration occurred at excluded derivative edges. Release residuals depend strongly on pose; historical 0.004644 Nm is not a calibrated noise floor. Failed transmissions remain archived but are excluded from V3 parameter fitting.

Candidate menu, not a permanent object policy:

- Internal alternating low/high speed plateaus: 1.5–12 mm/s, 0.55 s plateaus, 0.45 s raised-cosine transitions; capped by original Cartesian acceleration and measured-state progress clock.
- Steady upper-speed pass (12 mm/s) in FIT.
- Independent closing validation with 3.5–10 mm/s smooth 3.1 s modulation.

Initial finite-block proposal: FIT opening toward 12°, FIT closing toward 3°, FIT reopening toward 15°, complete VALIDATION closing toward 4°. Codex may choose a safer bounded subset or other menu targets from observed reachability/contact, before execution. Entire blocks run continuously without loaded Agent waits; original stall/force/collision stops remain authoritative. No requirement to reach target for an identifiability claim.

Kinematic lag surrogate predicts internal acceleration RMS about 0.0057–0.0152 rad/s² over 0–150 ms contact lag and broader actual speed variation. This is an opportunity estimate, not Isaac contact certification. No hidden I/b/C used.

## Estimation and independent validation

Cached compressed columns loaded once. Each complete successful segment processed independently: SG121 and full121-edge exclusion; 100 ms nonoverlap bins. Fitting, regularization, uncertainty and segment-out diagnostics use FIT only. Complete reserved validation trajectory never enters fit or uncertainty. Original nonnegative constant model and regularization retained.

Report conditional lag-5 HAC/iid maximum standard errors, FIT-only leave-segment-out stability and projected regressor sensitivity. Three-standard-error support is diagnostic, not a calibrated physical accuracy guarantee or execution gate. Historical residual appears only as a sensitivity comparison. Existing final unloaded APPROACH is passively retained at240Hz for phase/pose residual characterization; no extra calibration episode/motion. RELEASE residuals reported separately. Neither is declared a loaded dynamic noise bound.

Forward validation initializes measured angle/native velocity once at the121-sample derivative-interior boundary, uses compensated measured F/T as input, and never corrects the predicted state. Report excluded edges, train/test angular domain and extrapolation. Truth available only to independent evaluator after all Agent choices and physical execution end.

## Budget / integrity

FAST_DEV: one compressed numeric copy; checkpoint RGB-D; capped tiny event clip; private evidence below1GB. Target around30min; owner timeout35min plus90s cooperative export, no indefinite waits. No full Agent video/archive backups. Preserve prior results. Publish only code and compact derived report. Close all owned actors at finalization.
