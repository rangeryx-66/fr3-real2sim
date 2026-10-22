# V8.0 — Frozen Agentic Interaction Skill

**Release:** `agentic-interaction-v8.0.0`. **Assessment:** packaging and tested short-stroke lifecycle reproduction PASS, with identification and portability limitations. No new benchmark was launched.

## Package and behavioral freeze

The repository-discoverable [Skill](../../../.agents/skills/agentic-real2sim/SKILL.md), [runbook](../../../.agents/skills/agentic-real2sim/references/RUNBOOK.md), [API](../../../.agents/skills/agentic-real2sim/references/API.md) and [schemas](../../../agentic_interaction_v8/SCHEMAS.md) expose:
observe → inspect measured geometry → generate/inspect feasible candidates → validate execution → contact/probe → inspect feedback → recover unloaded → fit/independent VALIDATION → release/park.

The common CLI delegates to unchanged `RuntimeV7/serve_registered` for revolute and `PrismaticRuntimeV751/serve_radial_v751` for prismatic. Geometry selects existing bar/observed compact-cap families; no asset-ID pose rules were added. Fresh smokes test bars. Cap physical evidence remains V7.5.1 prismatic evidence, not a universal knob/revolute claim.

No motion, ranking, actuator, collision, safety or estimator algorithm/coefficients changed. The diff from `0885127` under `interaction_runtime`, `scripts`, `configs` and `config` is empty. Agent decisions remain separate from deterministic native safety/execution.

- Original revolute **71 source/config +22 external** hashes match.
- `BEHAVIOR.json` retains 606 matching current execution-directory files and 35 exact legacy/diagnostic snapshots. Earlier A/A1/B/C execution variants have original registrations/source snapshots; they are not attributed to the final report commit. Legacy snapshots cannot execute as recovery fallbacks.
- Transitive comparison found 303 matching Python files and 48 matching configs. Two differences were separately archived: a package docstring and unused R1/AnyGrasp backend. A broad directory snapshot is not a minimal dependency graph.
- Final release lock covers **956 files**; the Git tag freezes the complete tracked package. All 104 historical report/registration/derived commitments and six previous private complete-manifest commitments match. No prior result/failure is overwritten or added to a new denominator.

## Explicit setup and dependencies

[ENVIRONMENT_LOCK.json](ENVIRONMENT_LOCK.json) records Linux/NVIDIA, Isaac Sim 6.0.1.0, Python 3.12.14 and tested package versions. The private environment profile hashes 485 SAM source/overlay files and its checkpoint, and declares interpreter/ffmpeg/GPU/bundle/output/lock paths. This is not a claim that pip alone provisions Isaac or that a clean OS was tested.

An explicit one-time migration creates a private 379-file bundle of exact native robot/asset/closure/uncertainty/calibration bytes and minimal station placement/base metadata. It strips saved grasps and excludes old observations, outcomes and USD caches. Relocation changes filenames only, comparing every other XML value/topology and mesh byte. Bundle SHA256: `70127bd0e2f5868aa72ad8aa16e81d95ed46f5b621741f43a3f28b18496ae042`.

Robot IO is bound before planning imports. The entry forbids Python reads under old result roots and execution of excluded legacy programs. This is trusted research IO separation, not adversarial OS isolation. Licensed resources are explicit operator prerequisites; missing inputs fail. New scenes still require declared station/asset metadata and frozen-owner-compatible templates.

Known priors: registered station, hinge/rail and current state, native safety colliders, and coarse cabinet bounds/front orientation for the initial survey. Subsequent Agent views and executable grasp/contact geometry use fresh wrist RGB-D. The diagnostic overview RGB camera supplies no Agent geometry; neither passive reconstruction camera is instantiated. Hidden dynamics remain evaluator-only. Wrist mounting and virtual six-axis load cell are idealized simulation, not hardware calibration/safety.

## Reproduction method and software checks

**49 tests pass:** package 7, V7.5.1/radial 12, V7.5 lifecycle 18, RuntimeV7 safety 6, sliding FIT qualification 6. Skill format validation passes. See [SOFTWARE_REPRODUCIBILITY.json](SOFTWARE_REPRODUCIBILITY.json).

A fresh-context subagent (`fork_turns=none`) received only [handoff](CLEAN_ROOM_HANDOFF.md) and explicit checkout/environment/spec paths. It discovered the Skill, inspected actual fresh RGB/SAM/depth/cloud, geometry/plan/contact boards and chose atomic actions. No winning views, grasps, hidden coefficients or contact recipe were supplied. Separate fresh clones ran each scene; drawer inherited no cabinet USD cache. SSH-transferred Git bundles preserved exact commits after server GitHub TLS failed. This tests conversation/fresh-checkout independence on the installed host. Another Codex installation, clean OS and **cross-machine reproduction remain UNVERIFIED**; alternate host checks timed out/refused.

Preserved pre-contact packaging failures: symlinked assets initially omitted by the exporter, fixed with traversal/dependency checks; missing read-only plan-renderer IO adapter, added without actor/controller changes. No acquisition was hidden or retried for these errors.

## Fresh physical-contact smokes

Original one-second holds, actual arm/native-G1 contact and guarded measured-state parking were retained. These use existing revealed engineering physics, not unseen-dynamics evaluations. [Detailed Agent trace/segment table](CLEAN_ROOM_REPRODUCTION.md).

| Measure | Revolute 45936 | Prismatic 45290 |
|---|---:|---:|
| Executed source | `22510aa` | `e453c7f` |
| Acquisitions / physical recovery regrasp | 1 /0 | 1 /0 |
| Agent view adjustments | 2 | 1 +fresh generation |
| Outward FIT endpoint | 0.100166 rad (5.74°) | 25.067 mm |
| Reverse endpoint / travel | 0.024580 rad /4.33° | 5.881 mm /19.186 mm |
| Motion actions | 4/4 TARGET_REACHED | 4/4 TARGET_REACHED |
| Complete physical VALIDATION durations | 4.617 /4.329 s | 5.342 /4.938 s |
| Retained prediction horizons | 3.604 /3.317 s | 2.825 /1.921 s |
| Free position RMSE | 0.002097 /0.002186 rad | 0.0917 /0.0256 mm |
| Unloaded SAFE_PARK /actor exit | yes /yes | yes /yes |
| Actor wall time, initialization through cleanup | 1096.1 s | 1458.7 s |
| Recorder completion bytes | 44,296,283 | 45,132,608 |

Drawer same-scene recovery addressed perception/planning: upper-bar association failed all 16 collision branches before acquisition; fresh lower-bar SAM included a shadow. The Agent inspected frozen depth proposals, selected the measured protrusion and generated another feasible grasp. No collision veto was waived. Initial requested preload stayed 1.0 N; measured hold normals were approximately 1.13/0.93 N. No physical regrasp was required or manufactured.

**Identification limitations:** cabinet inertia remains unsupported. Drawer reverse motion completed, but its predeclared FIT interval had only 240 records and failed `insufficient local filter support`; only 838 pull samples qualified. This is not complete eligible bidirectional estimator FIT, trajectory cross-validation or position-dependent resistance evidence. Conditional constant-model/short-horizon RMSE does not establish mass/damping accuracy. Some validation is outside FIT support. Validation never entered fitting; no intervals were relabeled or physical episodes added to improve results.

## Finalization, evidence and cleanup

Both contact scenes lack `V8_LOADED_SOURCES.json`: Isaac fast shutdown skipped the atexit writer. Their prelaunch/job/core provenance is preserved; no loaded ledger was reconstructed. Explicit session-return finalization now writes it before frozen Isaac shutdown. A forced-exit software test and separate **zero-acquisition, zero-probing** scene (`ad18717`) verified the fix: 98 imported Python sources match tracked Git bytes, safe parking/exit in 165.673 s. An unchanged package-docstring file absent from the earlier lock was added to the final lock. This is an audit component check, not a third manipulation result.

[EVIDENCE_INDEX.json](EVIDENCE_INDEX.json) records private full-file-manifest commitments, source/job/result hashes and category storage. Contact recorder summaries total 89.43 MB; final sealed remote contact files total 89.67 MB, plus 3.77 MB for audit-only checking. Raw RGB-D, compressed 240 Hz numeric streams and inspected-image traces remain private on `test-X640-G50`. No continuous high-resolution video/duplicate raw archive was published. Temporary source transport bundles were removed after clone verification. All three owned actors/workers exited; unrelated processes were untouched.

## Paper evaluation and stopping boundary

[PAPER_PROTOCOL_DRAFT.md](PAPER_PROTOCOL_DRAFT.md) proposes 12 genuinely held-out objects ×2 unseen physics ×3 matched arms =72 slots, feedback-recovery/active-probing ablations, all-failure denominators, parameter availability/errors, complete held-out prediction, safety/time/cost and separate short-stroke/full-range endpoints. Exact objects/history eligibility, seeds, arm implementations, analysis code and budgets require separate registration/authorization. No full benchmark or fresh hidden configuration was created here.

V8 establishes the pinned Skill and tested Agent-to-physical-action lifecycle. It does not establish full-range dynamics, complete three-parameter accuracy in these smokes, universal cap support, hardware deployment or cross-machine portability. Stop with this release and protocol draft.
