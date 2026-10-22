# Experiment process cleanup

For xArm7/native-G1 Agentic Real2Sim reproduction and interaction, discover and read `.agents/skills/agentic-real2sim/SKILL.md`. V8 packaging uses explicit resources and the frozen physical owners; the older `skills/articulated-interaction` is the separate PiPER workflow. Do not infer a current grasp, camera pose or action from prior chat or archived success.

The user requires cleanup after every experiment. Save the report, continuous video and raw telemetry, then close all owned Isaac actors for completed or abandoned trials. Do not leave finalized successful or failed runs paused indefinitely.

Preserve a scene while actively recovering or continuing the same episode. Once the trial is declared finished, release its processes and GPU resources. Frozen controller sources must remain unchanged.

Prefer the runner's graceful shutdown marker. If shutdown hangs, verify the exact owned PID and command line before SIGTERM and, if necessary, SIGKILL. Never terminate unrelated server jobs or use a broad process-name kill.
