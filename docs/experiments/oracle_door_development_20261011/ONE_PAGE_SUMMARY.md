# PiPER reusable door opening: focused development result

**ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — no new physical 30° PASS.**

The archived standard-door benchmark identified two distinct failure families: incorrect grasp-region selection and insufficient contact/workspace execution. This round investigated only 45176, 45203 and 45415, preserved the original benchmark, and used three new physical episodes within the fixed trial budget.

| Object | Before | This round |
|---|---|---|
| 45176 | Wrong trim grasp, 0.36°, TCP speed stop | Correct-handle grasp, observed 1.31°; unilateral contact and ~10 mm drift prevented useful continuation |
| 45203 | Wrong trim grasp, 0.82°, contact loss | Correct handle identified, but generated TCP targets exceed the unchanged robot's height reach; no physical trial |
| 45415 | Correct handle, 28.99°, joint margin stop | Better predicted 30° station with 0.122 rad worst margin; two new unchanged-closure episodes failed before valid grasp |

**What improved.** A single shared planner now uses registered handle meshes, audits the actual convex collision solids during closure, and ranks base/wrist alternatives using contact geometry and minimum joint margin across approach plus opening. It replaces neither assets nor controllers with per-object scripts. The physical 45176 trial confirms that automatic planning can select and acquire its intended handle rather than trim. Geometric planning passed for two of three cases, but this does not establish physical task success.

**What the evidence revealed.** The old 45415 failure was a genuine joint-5 workspace boundary, not poor transmission. The new 45415 failures instead exposed repeated aperture-command reopening during unilateral centering: 188 increases, up to ~0.112 mm, unloaded contact before bilateral preload. A one-line shared repair retains the prior closing command. Recorded-input replay removes those increases, with gains, effort and thresholds unchanged.

**What remains unresolved.** That repair was not physically validated: the sole extra trial accidentally launched the original runner. Its actual source hashes and identical failure are reported. 45176 still transmits motion poorly despite the correct handle, illustrating that semantic correctness and nominal pad contact are insufficient. Convex bow-handle collision proxies fill visual rear cavities; no collision geometry was removed. 45203's handle-height constraint cannot be solved by base x/y/yaw optimization alone.

**Evidence limits and execution integrity.** Both 45415 episodes preserve continuous video and complete 240 Hz native/raw records. Native SIGINT shut down 45176 before buffered raw export; only video, 1 Hz states, metadata and the acquired transform remain. Exact velocity/contact peaks are therefore unknown. All three actors are closed, 434 saved files are verified locally, and 11,247 frozen/old-result files are unchanged. No physical parameters or protections were weakened, and no attachment, object actuation or scene stitching was used.

**Next autonomous-skill steps.** First validate closure-command continuity physically and make passive archival/cooperative shutdown reliable. Then improve sustained-transmission candidate ranking, replace oracle handle ROI with perception, and evaluate a newly registered batch. This round supports a concrete engineering diagnosis; it does not support claiming reliable generalized opening or an improved benchmark score.
