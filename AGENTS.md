# Current server handoff

For Agentic Real2Sim, first read `HANDOFF_SERVER_CODEX.md` and
`README_SERVER_CODEX.md`, then both repository-local Skills:
`.agents/skills/agentic-real2sim/SKILL.md` (including RUNBOOK/API) and
`.agents/skills/abc-continuous-v84/SKILL.md`.
These are included in Git; no personal Mac Skill or old chat is required.
The handoff is provenance and task context, not permission to start an
experiment, clear a latch, or treat a saved object state as a live World.
At transfer the latest R02 episode is closed, NOT PASS, with parking
unconfirmed. Read actual completion/safety records before taking action.

# Experiment process cleanup

For xArm7/native-G1 Agentic Real2Sim reproduction and interaction, discover and read `.agents/skills/agentic-real2sim/SKILL.md`. V8 packaging uses explicit resources and the frozen physical owners; the older `skills/articulated-interaction` is the separate PiPER workflow. Do not infer a current grasp, camera pose or action from prior chat or archived success.

The user requires cleanup after every experiment. Save the report, continuous video and raw telemetry, then close all owned Isaac actors for completed or abandoned trials. Do not leave finalized successful or failed runs paused indefinitely.

Preserve a scene while actively recovering or continuing the same episode. Once the trial is declared finished, release its processes and GPU resources. Frozen controller sources must remain unchanged.

Prefer the runner's graceful shutdown marker. If shutdown hangs, verify the exact owned PID and command line before SIGTERM and, if necessary, SIGKILL. Never terminate unrelated server jobs or use a broad process-name kill.
