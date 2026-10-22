---
name: agentic-real2sim
description: Run the versioned xArm7/native-G1 articulated interaction Skill for known-axis cabinet doors and horizontal drawers in Isaac Sim. Use for wrist RGB-D observation, automatic bar or compact-cap grasp planning, physical probing, contact-feedback recovery, dynamics identification, independent validation and guarded parking. Requires explicit private resource bundle and environment profile; does not authorize a new benchmark.
---

# Agentic Real2Sim V8.0

Read `references/RUNBOOK.md` and `references/API.md` before launching. Repository root is three levels above this skill directory. All CLI commands run from that root. No chat history or saved successful poses are required.

Use only fresh **wrist** RGB-D, SAM hypotheses, measured geometry, robot/contact/wrist-F/T feedback and current identification results. Known axis/rail, state, station and native collision checks are explicit priors. Never inspect the plant configuration, authored physics outputs, passive-camera package or archived successful grasps for online decisions. This is a simulated wrist mount and ideal virtual six-axis load cell, not verified hardware autonomy.

The common workflow is: observe → inspect → evaluate/move to useful wrist views → build geometry and automatic feasible candidates → inspect plan → validate execution → explicitly authorize finite grasp/probing block → inspect feedback → recover at an unloaded boundary → independent VALIDATION → release/park → finish.

You own observation selection, geometry hypothesis, grasp family/candidate, legal preload, probing/validation targets and recovery decisions. Deterministic frozen controllers own IK/collisions, actuators, real-time protection, finite budgets, release and parking. Do not prescribe manual grasp transforms or joint waypoints. Do not tune physical behavior during reproduction.

Actually inspect the returned RGB/mask/depth/point-cloud boards before choosing a view or approving a grasp. A filename or summary is insufficient. Record a short decision with inspected-image SHA256, missing information and expected benefit. Inspect current-generation evidence; stale observations or old attempts cannot justify current actions.

Do not wait for reasoning under unstable load. Physical commands are finite preapproved blocks that return to an unloaded boundary. HARD stop is authoritative; never clear it or continue manipulation. Missing single-pad positive-normal reports alone are not a separation measurement or a new veto.

Prismatic initial **requested** preload is 1.0 N. At verified unloaded boundaries choose among 0.5/0.75/1.0/1.5 N only when feedback supports it. This request is not measured gripping force. Existing bar and compact radial-cap families are geometry choices, never asset-ID lookups. Do not change drive authority, physics, score weights, protection thresholds or estimator coefficients.

Predeclare FIT trajectories and a distinct complete VALIDATION trajectory before execution. Use FIT only for parameter/model selection. A partial VALIDATION is not independent validation. Complete validation promptly after minimally informative bidirectional FIT; short-stroke support does not establish full-range dynamics. Report unsupported parameters as unknown.

Stop when safety requires it or no feasible useful recovery remains. Preserve failed attempts. Finish through validated release/clearance/parking, export compact essential 240 Hz data and close only owned processes. Do not launch a full paper benchmark from this skill without a separately authorized registration.
