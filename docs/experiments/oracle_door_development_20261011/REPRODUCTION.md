# Reproduction — ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT

The experiment runs on the existing PiPER Isaac environment, with Pinocchio 3.9.0, Pink 3.3.0 and DAQP. The unchanged prepared Benchmark A assets, frozen speed/Pink controller snapshots and camera calibration are required. Original asset files and frozen controllers are never edited.

1. `preparation.py` creates a separate output/asset-metadata directory, symlinks immutable geometry, registers oracle meshes and reads the exported robot base reference. Save preservation hashes.
2. Run `scripts/plan_oracle_door.py --job CASE/job.json --output CASE/planning --wall-s 600` for each registered object. Preserve all failures. The draft closure audit missed thin local bars because it stopped at full mesh width minus 2 mm; its results are retained separately and were not physically executed. The final planner sweeps the full original aperture to zero.
3. `scripts/run_oracle_door_trials.py` executes each feasible selected candidate once from the normal closed state. Source/config/plan hashes and exclusive trial reservation files are saved before launch. The owner requests graceful cleanup only after report/video/raw export. There is no automatic retest loop.
4. The sole targeted retest owner reuses the exact 45415 plan with `scripts/run_oracle_door_closure_continuity.py`. The root additional-retest reservation is exclusive. This one-line continuity variant retains the closing aperture command during unilateral centering; original sources and thresholds remain unchanged. New outputs must be used for an independently labeled reproduction, never overwrite this trial ledger.
5. Run `scripts/oracle_door_metrics.py OUTPUT_ROOT`, `motion_diagnostics.py OUTPUT_ROOT`, and `plot_results.py OUTPUT_ROOT` only after trial export. These files are not imported by the actor. All measured differences are passive evidence.

Normal bootstrap initializes a closed object once before manipulation. During execution there are no object-joint commands, attachments or external object forces. The robot/cabinet stay fixed after setup. The physical sequence has one grasp and no releases or regrasps.

A planning pass is not a physical pass. A raw closure excursion is not credited as opening. The target is actual >=30 degrees after valid correct-handle grasp, through real contact and original protections. Results are development outcomes on three selected diagnostic cases, not a rerun or replacement of the original Benchmark A score.

Raw telemetry uses 240 Hz gzip/pickle chunks with index/metadata, including native impulses, points, normals and collider pairs. Continuous videos and raw data remain local/on the lab server; GitHub contains source, plans, hashes, metrics, static figures and reports.
