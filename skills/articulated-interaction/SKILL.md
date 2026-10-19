---
name: articulated-interaction
description: Collect RGB-D observations at several physically reached states of a revolute or prismatic object, record breakaway and moving effort, and export ART-style and Ditto/HouseDitto reconstruction inputs.
---

Run from the repository root using `scripts/articulated_interaction_skill.py`.
Choose `open_for_multistate_capture` or `probe_articulation_and_effort` and a
prepared dataset episode. Read the configured targets and motion budget before
execution. This skill runs simulation; it does not authorize physical hardware.

```bash
python scripts/articulated_interaction_skill.py --job existing_job.json \
  --joint-type revolute --goal open_for_multistate_capture --output results/capture \
  --ffmpeg /path/to/ffmpeg
```

Use `--joint-type prismatic` for linear joints. `--stage prepare --asset-root`
reuses the frozen reference job's deployment policy and bounded mobile search;
then use the emitted `execution_job.json` for `--stage run`. Export an existing
capture with `--stage export --output`. `--manifest` runs a frozen list of jobs
and keeps every failure in `batch_summary.json`.

Use the existing PiPER real-contact approach, closure and safety checks. Do not
use attachments, direct object force or execution-time object joint commands.
Targets are requests: only measured/estimated reached states become captures.
Preserve partial captures and the first physical stop reason.

For repositioning, release/retreat safely, use the existing collision-checked
mobile recovery, stop/lock, reobserve and regrasp before continuing. Never move
the base while gripping. If safe release or a current handle observation is
unavailable, return `REPOSITION_REQUIRES_SAFE_REGRASP`; do not teleport or reset.

Return `multistate_capture.json`, state folders, effort logs and backend bundles.
Distinguish contact-measured force, command proxy and estimated-radius torque.
Repeated starts at open states are restart effort, not closed-state breakaway.
Do not call any of these measurements precise hinge friction.

Validate backend bundles using the exporter; ART-style grouping is not proof
that an unpublished official ART loader or model has been executed. Twin updates
must retain source/provenance, observed range and uncertainty; do not overwrite
the successful baseline or claim reconstruction from an input bundle alone.
