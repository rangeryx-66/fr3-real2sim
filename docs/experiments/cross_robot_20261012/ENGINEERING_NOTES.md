# Engineering preflight corrections

No physical episode was launched in any engineering preflight.

1. The first dimension adapter accidentally queried the home seed again for the alternate IK branch. It was corrected to a second explicit deterministic random seed, uniformly across robots. Exact owned offline worker PIDs were stopped; metadata/partial results remain under `engineering_prebranch_*`.
2. The initial input resolver assumed every case had `door_transfer4_20261012/ID/cooked_initial.json`. 45176 and 45415 instead reference their existing `benchmarkA_30deg_20261011/ID/setup/cooked_initial.json`. Observed error: `FileNotFoundError` for those two absent new-folder paths. The saved job path is now resolved uniformly. The previous 16-process batch completed with eight input errors and is retained under `ENGINEERING_INPUT_PATH_*` / `engineering_input_path_batch`.
3. The final comparison was rerun with one corrected source and all 16 process return codes were zero. No controller/physics tuning was applied to individual robots or assets. Only this final batch contributes to `COMPARISON.json`.

The earlier per-case stderr files were reused by the final owner, so the original full engineering stderr is not preserved. Batch metadata, partial outputs and the observed input error above are preserved; no missing physics/contact data is invented. Final planning logs are complete in the raw archive.
