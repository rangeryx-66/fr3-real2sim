# V8 explicit schemas

Environment JSON (operator-private, absolute paths; no magic working directory):

```json
{"python":"/installed/isaac/python","sam_python":"/installed/isaac/python",
 "sam_pythonpath":"/pinned/sam_overlay:/pinned/sam_source",
 "sam_checkpoint":"/licensed/weights.pt","sam_checkpoint_sha256":"SHA256",
 "sam_source_sha256":{"/pinned/sam_source/file.py":"SHA256"},
 "ffmpeg_dir":"/installed/ffmpeg/bin","bundle":"/private/versioned_bundle",
 "output_root":"/private/v8_episodes","actor_lock":"/shared/owned_actor.lock",
 "physics_gpu":0,"perception_gpu":1}
```

Commit all SAM Python source files/overlay files actually available on PYTHONPATH in `sam_source_sha256`; checkpoint hash is checked. Environment lock additionally records package versions/driver/GPU. Source availability/licensing are operator responsibilities. Only tested Linux/Isaac stack is currently supported; a fresh checkout on the same host is not cross-machine validation.

Public job spec (no hidden parameter values):

```json
{"schema":"agentic-interaction/v8","episode_id":"v80_drawer_smoke_00",
 "scene":"drawer_bar","purpose":"reproducibility_smoke",
 "physics_file":"/private/scene_generator/configuration.json",
 "physics_sha256":"SHA256","identification_case":0,
 "wall_budget_s":2400,"sim_budget_s":1400,"acquisition_budget":2,"agent_wait_s":360}
```

Only registered bundle scene names are accepted. Known station/kinematics and baseline home state are explicit scene inputs. No executable pose overrides, arbitrary safety/controller fields or passive camera inputs are accepted. New output IDs cannot overwrite prior episodes. `prepare()` records job, resource and exact-source commitments, path-only robot equivalence and explicit frozen-owner compatibility fields. Plant file is for scene generator only; no general claim of OS-level blinding.

Resource bundle BUNDLE.json: per-file SHA256, original-source SHA256, external frozen-path→bundle mapping, robot URDF paths, explicit calibration paths and scene asset/station/template paths. No RGB-D, learned model, previous attempt/pose, USD import cache or dynamic outcome is imported. The native asset mesh and manifest are permitted safety/kinematic resources; observed wrist geometry alone supplies executable grasp generation/contact ranking.

One-time operator migration descriptor:

```json
{"native_configuration":"/declared/original/NATIVE_CONFIG.json",
 "active_closure_source":"/declared/original/active_closure.py",
 "robust_uncertainty_config":"/declared/original/UNCERTAINTY_FROZEN.json",
 "wrist_calibration":"/declared/original/wrist.json",
 "camera_calibration":"/declared/original/camera.json",
 "ft_calibration":{"bias_sensor":[0,0,0,0,0,0]},
 "scenes":{"registered_scene_name":{
   "asset_root":"/declared/asset",
   "station_report":"/declared/station/report.json",
   "job_template":{"experiment":"V7","asset_id":"LIBRARY_ID",
     "articulation_v7":{"joint_type":"prismatic","limits_native":[0,0.15],
       "units":{"coordinate":"m","effort":"N","inertial":"kg","damping":"N s/m"}},
     "remaining_frozen_owner_fields":"supply exact committed execution template, not this abbreviated example"}}}}
```

The bias shown is a shape example, **not permission to zero a real or simulated sensor under contact**. Supply the complete previously verified calibration provenance and frozen execution template. Migration is explicit operator work: it needs the committed external files at their original declared locations once, verifies their exact hashes, copies licensed source geometry and strips runtime/history fields. Later runs use only the completed bundle. Do not call the legacy model `prepare()` functions to regenerate assets from an undeclared xacro tree.

Adding a scene requires an independently prepared source asset/manifest, current known-articulation limits, a declared base/asset station and a frozen owner-compatible template. This package does not autonomously solve station placement, discover an unknown hinge/rail, acquire missing licensed assets, or calibrate a new hardware mount. Validate a new scene's metadata offline before authorizing contact. Existing historical manifests remain provenance; they are not per-object online view/grasp recipes.
