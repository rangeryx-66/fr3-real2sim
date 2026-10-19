# Reproduction and evidence

Read [REPORT.md](REPORT.md) and [SCOPE_AND_MODEL_STATUS.md](SCOPE_AND_MODEL_STATUS.md) before interpreting any range as physical capability.

Runtime: existing Isaac arena Python 3.12 environment, Pinocchio 3.9 / Pink 3.3 / DAQP. The source preparation additionally uses xacro 2.1.1 with existing PyYAML. It was installed into an isolated `cross_robot_sources_20261012/python_deps` directory; the frozen experiment environment was not changed.

Official sources are pinned in `ROBOT_MODELS.json`. Their license notices are included as `LICENSE_AGILEX`, `LICENSE_UFACTORY`, `LICENSE_FRANKA`. Meshes remain in the pinned source repositories and verified raw archive, rather than being relabeled as new robot models. The generated URDFs preserve the official arm and exact old PiPER hand parameters; FR3 joint-name aliases and the unvalidated foreign fixed-tool proposals are explicit.

The scripts preserve the actual server paths in their reproducible configuration:

```sh
# Generate resolved official descriptions and copied frozen hand.
python prepare_models.py
# Independent URDF-chain FK/Jacobian and reduced-model FK verification.
PYTHONPATH=.:scripts python verify_kinematics.py
# Model-only Isaac import: no World, object or physics integration.
python inspect_isaac_models.py
# Register/freeze and run the bounded 16-case offline study once.
PYTHONPATH=.:scripts python run_offline_batch.py
# Read-only diagnostics and aggregation.
PYTHONPATH=.:scripts python audit_physical_models.py
PYTHONPATH=.:scripts python audit_source_parameters.py
PYTHONPATH=.:scripts python audit_collision_semantics.py
PYTHONPATH=.:scripts python diagnose_pure_ik.py
PYTHONPATH=.:scripts python summarize_study.py
```

The two added runtime files are `scripts/cross_robot_adapter.py` (names/dimensions/reduced IK/model mapping) and `scripts/cross_robot_offline.py` (calls the unchanged shared planner with dimension-only branch adaptation). They do not implement or claim validated foreign-arm low-level contact execution. No physical run can be reproduced by pretending the offline tool/actuator proposals are validated.

Full raw study archive path, checksum and 311-file verification are in `ARCHIVE_SUMMARY.json` / `LOCAL_ARCHIVE_VERIFICATION.json`; final registered plans, q trajectories, collision blockers and 44 variants are under `offline/`. Model-only USD drive/name mapping is `ISAAC_STRUCTURAL_MAPPING.json`. Old real-contact 240 Hz streams and videos remain at the frozen [81ab4f5 evidence index](../door_transfer4_20261012/EVIDENCE_INDEX.md).
