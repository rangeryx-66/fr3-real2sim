# Reproduce the NERO-only checks

Read [REPORT.md](REPORT.md) first: free-space stabilization is observed; official inertia fidelity, complete camera framing and owner-SIGTERM verification are not fully cleared.

Pinned reference `b9c8166`, manufacturer CAD source `983788b58d7eae76511177f768d85877534917fe`, Isaac Sim 6.0 / Python 3.12 / Pinocchio 3.9 / Pink 3.3. Reuse the existing NERO model output and four saved plans. No PiPER/shared-controller edits.

On the lab server:

```sh
cd /data1/home/rangeryx/fr3_real2sim_known_whole_20261007
PYTHONPATH=.:scripts OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/probe_nero_implicit.py --run 3
```

The output directory is created with `exist_ok=False` so prior evidence cannot be overwritten. Do not rerun this command into the existing archive. Loaded snapshots retain exact historical run variants and hashes. The final runner has the audited link6/flange correction and TGS gravity substeps; historical run 02 lacks the substep flag. Metadata/rate/unit and finite-effort details are in each run's JSONs.

This documentation is a reproduction recipe, **not authorization to exceed the completed three-session budget**. Fix camera focal units and confirm native inertia interpretation before another physical validation. Full original evidence is attached to the [diagnostic release](https://github.com/rangeryx-66/fr3-real2sim/releases/tag/nero-actuator-diagnostics-20261012).
