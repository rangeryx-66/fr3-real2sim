# Reference validation and scope

The following values were obtained on the lab Isaac host. They are evaluation results, not estimator inputs or target thresholds selected after seeing GT. Source runs are retained on that host under `results/fr3_qcomp_cross_object`, `results/fr3_qcomp_cross_object_infrastructure_retry`, `results/fr3_payload_capture_recovery` and `results/fr3_payload_fresh_gate`. Large traces, GT asset exports, checkpoint files and license files are deliberately excluded from this GitHub repository.

| Object | Batch | Paired static poses | Mass error | COM error | Reported COM uncertainty | Confidence accepted |
|---|---|---:|---:|---:|---:|---|
| soup | prior frozen baseline retry | 8 | 0.099% | 0.86 mm | 4.23 mm | yes |
| mustard | prior frozen baseline | 8 | 0.119% | 1.02 mm | 2.85 mm | yes |
| raisin | capture recovery | 7 | 1.293% | 33.30 mm | 12.84 mm | no |
| banana | capture recovery | 7 | 0.828% | 5.97 mm | 18.58 mm | no |
| mug | fresh PayloadID start gate | 5 | 0.360% | 17.44 mm | 14.55 mm | no |
| sugar | fresh PayloadID start gate | 8 | 0.072% | 2.55 mm | 1.20 mm | yes |

The correction to the PayloadID start gate uses recent 0.8 s free-space state instead of rejecting current stable captures because of historical settling. It still requires sustained two-finger contact, measured clearance and stable recent relative pose, and the recording/cross-pose slip guards remain active. With this change, sugar seed 1026 produced eight pairs and mug seed 1036 produced five. Those comparisons were replays of the same seeds, not a deterministic paired physics A/B that isolates a single cause. Force stayed at a total 60 N and friction at μ=1 in the calibration simulator. No grasp executor or USD was modified.

Four of six evaluated objects happened to have COM error below 10 mm, but only three passed the estimator's confidence check. Banana's low one-run GT error did not justify measured-COM production use because its reported uncertainty was high. Raisin and mug still have substantial residual/model or grasp-state limitations. Global measured COM is therefore **disabled**; use explicit geometry fallback for COM/inertia when confidence fails.

The stationary scan/MV-SAM3D stage used 1800 RGB-D frames across two rings and an official 8-view input subset, with 2 and 4 views as ablations. MV-SAM3D produced visual GLBs. No benchmark in this repository validates direct conversion of those raw GLBs to metric USDs. The documented metric and physics checks must be run separately.
