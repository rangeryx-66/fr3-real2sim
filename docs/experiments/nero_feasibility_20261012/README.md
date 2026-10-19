# NERO feasibility evidence

Read [REPORT.md](REPORT.md) or the [one-page summary](RESEARCH_SUMMARY.md). This round is **INCONCLUSIVE**, with promising offline geometry/IK and an unsuccessful no-object approximate-servo probe. No door-contact episode was executed.

The source changes are isolated in four new files:

- `scripts/nero_door_adapter.py`: native model mapping and two verified proxy pairs, with raw-triangle checks retained.
- `scripts/plan_nero_doors.py`: the existing shared planner with seven-joint seed adaptation and native hand convention; no object-specific control branch.
- `scripts/nero_actuator.py`: explicit finite-effort approximate PD/gravity servo. **Its present design is unstable in the probe; do not treat it as a validated execution controller.**
- `scripts/probe_nero_integration.py`: robot-only gravity/trajectory/jaw test and lossless export. No object or door episode.

Official source: AgileX `agx_arm_urdf` commit `983788b58d7eae76511177f768d85877534917fe`. License: `LICENSE_AGILEX`. The preparation and geometry/tool verification scripts, selected plans and exact source/config hashes are included. Runtime uses the existing Isaac Python 3.12 environment with Pinocchio 3.9 / Pink 3.3 / DAQP, xacro 2.1.1 in the previously isolated source-resolution dependency directory.

Server paths and commands remain explicit in the scripts for reproducibility. Preparation adds only a consistent TCP frame, folds the virtual jaw command into native mirrored sliders, and leaves official arm/gripper meshes and inertias unchanged. Model-only/robot-only evidence must not be relabeled as manipulation success.

`LOCAL_ARCHIVE_VERIFICATION.json` checks 128 files in the complete local raw archive. It contains the two probe-scene outputs (first API failure: zero physics steps), all 5,040 measured-state and torque-command records, source models, plan audits and original video. Missing self-contact forces or measured motor reactions are not fabricated.

The original video is misframed. `recorded_q_replay.mp4` is a separately labelled visualization of actual saved q using simplified hulls: **not original Isaac video, not new physics, not collision evidence**. No physics rerun was made to improve the camera view.
