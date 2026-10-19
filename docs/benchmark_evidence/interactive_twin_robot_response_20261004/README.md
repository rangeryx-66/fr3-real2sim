# Shared robot response experiment evidence

Base: 79b100c. Mode: SIM_TO_SIM_BLIND_SYSID.

Use `main_table.csv`, `delivery_summary.json` and the result report two directories above.
PASS means the frozen conditional prediction criteria passed, not unique friction identification or full-task success.
M3 was not triggered in any condition. Overall acceptance failed; no baseline was promoted.

Large JSON files are gzip-compressed without information loss. `evidence_index.json` records hashes of decompressed contents.
The complete 109-file delivery, including all new held-out observable trajectories, is also saved locally under
`results/interactive_twin_robot_response_deliverables/delivery` and on the server under
`results/interactive_twin_robot_response_20261004_v2`.
Native full logs/videos remain in those server trial directories. No existing successful baseline was changed.

The legacy native `first_failure_state` field also stores successful conditional replay terminal frames.
The compact native audit distinguishes this from an actual `failure_phase`; all 308 physics replays completed.
