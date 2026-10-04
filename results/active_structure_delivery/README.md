# Frozen active structure experiment evidence

Start with [REPORT.md](REPORT.md), [frozen_test_manifest.json](frozen_test_manifest.json) and [active_summary.json](active_summary.json).

Implementation: `b339503`, based on `eb0fefa`; original successful baseline files unchanged. Native run completed before 2026-10-05 05:00 Asia/Shanghai. All 12 fresh TEST configurations remain in the denominator. No physics fitting or TEST-driven method changes.

The compact archive was downloaded and all 365 indexed files verified against SHA-256. Full measured observations, command tapes, complete videos, meshes and native process logs remain in the source directory recorded by `evidence_index.json`. Three representative continuous videos were also copied locally; their exact source, label and hash are in `media_index.json`. Videos are not stored in Git.

`postprocess/` contains evaluation-only scripts used to append figures and diagnostics after all native episodes stopped. They do not participate in control, fitting, model acceptance, asset selection or method freezing.

## Single experiment entry (server repository)

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/run_active_structure_benchmark.py --config configs/active_structure.yaml
```

For a later replication, copy the config and change only output location/deadline. Existing run outputs are immutable.
