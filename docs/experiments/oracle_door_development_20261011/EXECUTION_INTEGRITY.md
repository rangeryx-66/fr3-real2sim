# Execution integrity exceptions

1. **45176 archival failure.** The recorded owner-end file was written before the stop request and records an intended pause/export, not proof that it happened. Native Isaac SIGINT handling exited before Python pause/export. The actual surviving evidence is listed in PARTIAL_EVIDENCE_REPORT.json. No 240 Hz contact buffer or events were exported. No scene restart or state stitching occurred, but this episode does not satisfy the complete raw-evidence requirement.
2. **45415 extra-trial configuration error.** development_02 used the same original development runner, as confirmed by /proc cmdline and prelaunch source_config_hashes.json. The submitted closure-continuity repair was not exercised. This trial consumes the single extra-trial budget. Corrected owner code is for future reproduction only; its correction does not change the recorded outcome.

The report uses no substituted sensor values, invented precise peak or unexecuted control result. These are operator/execution mistakes in this round, not intrinsic object mechanical limits. Frozen code, original assets and prior results were not modified to conceal them.
