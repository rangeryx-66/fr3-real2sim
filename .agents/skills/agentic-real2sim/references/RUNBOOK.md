# Reproduction runbook

## Explicit prerequisites

Linux NVIDIA GPU + Isaac Sim **6.0.1.0**, Python **3.12.14**, torch 2.11.0+cu128, numpy 2.3.1, scipy 1.17.0, pin 3.9.0, pin-pink 3.3.0, python-fcl 0.7.0.11, trimesh 4.11.1, daqp 0.8.5, Pillow 12.2.0. Full tested environment is recorded in `docs/experiments/skill_v80_20261023/ENVIRONMENT_LOCK.json`. These are tested versions, not a claim that pip alone provisions Isaac/Omniverse or every GPU driver. A clean OS/container installation is not verified by reusing this installed stack.

Obtain licensed assets, official robot meshes, SAM3 source/overlay/checkpoint and existing F/T calibration from the operator. Large/private resources are not in Git. The environment profile gives absolute paths, exact source/checkpoint hashes, Python/SAM executables, ffmpeg directory, shared physics/perception GPUs, bundle/output/lock paths. A missing resource is an explicit error, not permission to recover a pose from old results.

The preserved initial survey additionally uses the library cabinet's broad static bounds/front orientation to propose its first coarse view. Later Agent views use fresh wrist evidence. This is an explicit known-model initialization dependency, not handle GT for grasp scoring or an image-only initial station estimator.

One-time migration (operator, not the online Agent):

```
python3 -m agentic_interaction_v8.cli export --descriptor EXPLICIT_INPUTS.json --output NEW_PRIVATE_BUNDLE
```

The descriptor declares native robot config, active closure, uncertainty, wrist/camera/F/T calibration and each asset source plus minimal station report/job template. `resources.py` exports original mesh/URDF bytes, removes old poses and caches, records all hashes. It preserves 22 historical external files. `materialize()` changes filenames only and checks every other XML field and mesh byte. Do not regenerate the physical assembly using newer xacro or unofficial meshes.

Fresh checkout and source audit:

```
git clone --branch agentic-interaction-v8.0.0 REPOSITORY FRESH_CHECKOUT
cd FRESH_CHECKOUT
python3 -m unittest discover -s tests -p test_agentic_interaction_v8.py
python3 -m agentic_interaction_v8.cli audit --bundle NEW_PRIVATE_BUNDLE
```

Read `agentic_interaction_v8/SCHEMAS.md` for environment/job formats. Each new smoke has a new ID/output directory and explicit plant-file commitment. A committed spec never embeds hidden coefficients. Development/reproduction physics is not a new blind benchmark. The launcher checks free VRAM without requiring exclusivity, and shares the documented owned-actor lock across checkouts.

```
python3 -m agentic_interaction_v8.cli launch --job SMOKE_SPEC.json --environment ENVIRONMENT.json
```

Run launcher in a managed background process. The same process stays alive throughout Agent view/recovery commands. Inspect `WAITING_FOR_AGENT.json`, `DECISION_PACKET.json`, current observation files and completed responses. Publish atomically through the CLI, never a partial redirected JSON file. Every physical approval specifies finite, predeclared segments. Never count process rc=0 alone as success: inspect episode result, attempts, completed validation and measured final parking.

```
python3 -m agentic_interaction_v8.cli command --episode OUTPUT --sequence 0 --json COMMAND.json
```

Watch acknowledgments and completion before the next command. Keep decisions concise within the finite unloaded window. If interaction cannot continue, publish `finish`; if transport fails use the owned scene's `STOP_AND_EXPORT` marker. Allow guarded exit reserve. Inspect process completion and GPU/PID release. Do not kill unrelated jobs.

The release writes `V8_LOADED_SOURCES.json` synchronously when the session returns, before the frozen runner closes Isaac. Do not depend on `atexit` for simulator evidence. `V8_PRELAUNCH.json`, `SOURCE_PROVENANCE.json`, the release source lock and resource commitments remain separate checks. Missing provenance is an audit failure to report, not a reason to recreate a purported loaded-module ledger after process exit.

## Outputs and claims

Keep original 240 Hz compressed F/T, motion/contact signals and sparse fresh RGB-D private. No continuous video by default, no duplicate archives. Public derived report includes attempt failures, current model, FIT support, full validation duration versus derivative-supported prediction horizon, safety/parking and timing/storage. A short physical smoke proves interface reproducibility for that tested scenario, not population robustness, full-range reconstruction or hardware safety.

Never inspect `HIDDEN_AUTHORED_PHYSICS.json`, `PRIVATE_NATIVE_*`, plant configuration, old episode directories or passive-camera outputs during online decisions. Role separation is a research protocol, **not an OS security sandbox**. Seal actions/model before independent post-hoc evaluation.
