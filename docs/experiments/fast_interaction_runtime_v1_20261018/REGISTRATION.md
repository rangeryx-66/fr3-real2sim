# Fast interaction runtime V1 — engineering validation

Reference: `eb80bf3`. Separate branch/namespace; frozen PiPER, native-G1 and Recovery V1.1 sources/results are retained.

This is a bounded engineering iteration, not a new research benchmark or full Agent manipulation episode. Regressions may be repeated only after a concrete shared software/infrastructure defect is identified. No asset substitution, grip-force increase, actuator gain change, friction change or object assistance is permitted.

Validation components:

1. Archived native-record replay: 5.0680359 mm/s pre-contact panel entry; 8.2487895 mm/s measured / 35.1942511 mm/s commanded-FK handover; 2.0002655 N isolated warning.
2. Deterministic controller, transport, force history, unsafe-condition and lossless-export checks; active-source audit.
3. Matched native unloaded endpoints, four round trips, same xArm7/G1/45936 scene; account for initialization and process exit. Separately report unchanged-command replay and faster time-law results when available.
4. Short native contact chain: automatic grasp from measured RGB-D engineering fixture → original one-second hold → approximately 1° pull → release/park → fresh actuator-executed wrist RGB-D → Agent-selected observed panel region → compact native-G1 physical closing → separation/park.
5. Native collision-route rejection, joint/velocity command rejection, finite effort readback, and independently applied **robot-tool-only** sensor test loads. These loads never assist the object and are excluded from manipulation/identification evidence.

The fixture caches measured wrist surfaces from the baseline to avoid repeating expensive active sensing during low-level runtime development. The grasp is automatically recomputed from those surfaces; no historical successful grasp transform is an input. This does not claim fresh visual generalization. Panel contact uses fresh wrist RGB-D inspected by Codex. Registered station, known hinge/state, native safety geometry, simulated wrist mount and approximate actuator assumptions remain.

FAST_DEV retains 240 Hz physics, raw contacts, wrench and motion. It disables default continuous video and redundant diagnostic queries. EVALUATION retains continuous diagnostic video. Raw evidence and images stay private; publish code and compact results only.

Interim valid failures remain evidence: the first contact regression acquired/held/released/parked but exposed a fixed-lead reference-clock migration defect; the next reached 1° and first real panel contact but exposed a pre/post-step contact-frame association error. Neither is relabeled as initialization failure or task PASS. Fixes must retain the same physical geometry, contact authorization and tolerances.
