# R1-7a + Dex1-1 grasp candidate comparison

This framework only generates and screens 6-DoF grasp candidates. It does not
publish robot commands, plan trajectories, or execute a grasp. All providers
read the same calibrated camera frame. Native model scores are kept as-is;
their numerical scales are **not** directly comparable.

## Input and frames

`--scene` accepts either the existing frozen `.npz` (`points`, optional `rgb`,
`mask`, `K`, `T_B_C`) or a directory with `depth.npy/png/tiff`, `camera.json`
(`K`, `depth_scale_to_m`, `T_B_C`), optional `rgb` and `mask`. Points are in
camera optical coordinates. `T_B_C` maps camera optical to `r1a7_world`.

Every saved `GraspCandidate` has `T_C_G`, `score`, optional `width_m` and
`depth_m`, `T_G_TCP`, `T_C_TCP`, `T_B_TCP`, native rank, checks, and provenance.
The relationship is `T_B_TCP = T_B_C @ T_C_G @ T_G_TCP`. GraspNet-format
providers use the existing `src/r1a7_frames.py` +X approach convention;
GraspGenX uses its +Z approach/+X closing convention and the project Dex1
pad-centre TCP. The R1 Link7→Dex1 mount and TCP values remain provisional
until physical calibration; see `config/end_effector_calibration.yaml`.

Clone each official predictor outside this repository and keep its checkpoint
in that checkout. Set `GRASPGENX_ROOT`, `GRASPNESS_ROOT`, and
`ECONOMICGRASP_ROOT` if those checkouts are not at the lab server's default
paths. GraspGenX needs its official `release/gen` and `release/dis` checkpoint
directories; EconomicGrasp's checkpoint is the official
[`economicgrasp_realsense.tar`](https://github.com/iSEE-Laboratory/EconomicGrasp/releases/download/v1/economicgrasp_realsense.tar).
The [GraspGenX release weights](https://huggingface.co/adithyamurali/GraspGenXModel/tree/main/release)
are kept outside Git because they total more than 1.6 GB.

## Run

On the lab server, from the repository root and with the geometry Python
environment active:

```bash
python compare_models.py \
  --scene /path/to/frame.npz \
  --models graspgenx graspness anygrasp zerograsp rngnet economicgrasp \
  --graspgenx-python /path/to/GraspGenX/.venv/bin/python \
  --graspness-python /path/to/anygrasp/env/bin/python \
  --economicgrasp-python /path/to/anygrasp/env/bin/python \
  --output-dir results/grasp_compare/frame_001 --visualize
```

GraspGenX uses the official released generator/discriminator checkpoints in
`GraspGenX/ext/graspgenx_checkpoints/release/{gen,dis}` and constructs its
sweep volume from the official Unitree Dex1-1 URDF, not a Robotiq mesh.
Its release does not return a per-candidate opening width; the shared
collision check evaluates Dex1 at its maximum opening, which is conservative
for nearby obstacles but does not prove finger contact.
Graspness uses the supplied `minkuresunet_realsense.tar`. The licensed
AnyGrasp SDK is still blocked on this host by `feature id mismatch`; use a
frozen AnyGrasp result with `--predictions anygrasp=/path/to/grasps.json`.
This keeps the original model and output untouched.
The Graspness source checkout uses
`patches/graspness_torch25_inference.patch` for the current PyTorch runtime;
the patch does not change its trained weights or grasp poses.
Apply it with `git -C "$GRASPNESS_ROOT" apply --unidiff-zero
/absolute/path/to/patches/graspness_torch25_inference.patch`.
EconomicGrasp uses its official RealSense release checkpoint and runs on
the same sampled scene cloud without its native collision filter. Its
MinkowskiEngine, PointNet2 and KNN extensions must be built in its Python
environment.

ZeroGrasp now has a native single-frame adapter. It uses the official Mirage
checkpoint, RGB-D, mask and supplied camera intrinsics. Install its octree
extensions in a separate environment and run:

```bash
python compare_models.py --scene /path/to/frame.npz \
  --models graspgenx zerograsp economicgrasp graspness \
  --graspgenx-python /path/to/GraspGenX/.venv/bin/python \
  --zerograsp-python /path/to/ZeroGrasp/.venv/bin/python \
  --economicgrasp-python /path/to/EconomicGrasp/python \
  --graspness-python /path/to/graspness/python
```

The released ZeroGrasp model calls an older OFE signature than the submodule
revision pinned by its repository. The adapter bridges that signature for one
target in one frame without changing model weights. Depth is passed in
millimetres, as required by its loader, and results are converted back to
metres. The official GraspNet GraspGroup pose convention is retained.

RegionNormalizedGrasp and any precomputed ZeroGrasp result can also be
compared from original GraspNetAPI `N x 17` `.npy` files or the same JSON
contract:

```bash
python compare_models.py --scene /path/to/frame.npz \
  --models zerograsp rngnet graspgenx \
  --predictions zerograsp=/path/to/zerograsp.grasp.npy \
  --predictions rngnet=/path/to/rngnet.grasp.npy \
  --graspgenx-python /path/to/GraspGenX/.venv/bin/python
```

RNGNet native inference is not yet wired to the one-command runner. If no
prediction is supplied, it is reported `UNAVAILABLE`, never counted as zero
successful grasps. RNGNet has a checkpoint but its demo pins older
PyTorch/CUDA and assumes GraspNet camera calibration. Feeding it the current
frame without a verified camera adapter would be an invalid comparison.

`--approach-camera x y z --approach-max-angle-deg 30` applies the same camera
frame direction filter to all providers. `--target-radius-m` controls the
target region near the mask. The full scene cloud is checked at
pregrasp, mid-approach and grasp against **official Dex1-1 URDF collision
STLs** at 1 cm intervals along an 8 cm approach. The non-watertight base-link
mesh is evaluated by surface distance; watertight finger meshes use signed
distance. Target points are exempted only inside the aperture defined by
the official terminal pad meshes (3 mm tolerance in depth/height);
target-mask pixels on a nearby door panel
remain collision obstacles. This is a point-cloud check, not a continuous
swept-volume proof, and RGB-D cannot reveal occluded surfaces.
An absent mask yields
`UNKNOWN_NO_TARGET_MASK`, not a false collision-free result.

For R1 IK, launch `src/r1a7_moveit.launch.py` with the chosen
`R1A7_BASE_POSE`, then pass `--run-ik --ik-python /path/to/ROS/python`.
The read-only IK checker uses eight seeds, the 7-joint MoveIt group,
`r1a7_tcp`, `/check_state_validity`, and records the minimum J5/J6/J7 margin.
It does not move the robot. Point-cloud collision covers Dex1 and the scene;
MoveIt currently checks robot self collision but does not insert the point
cloud as collision geometry for the whole arm.

Outputs include `shared_input.npz`, each native prediction JSON, inference
logs, `candidates.json`, optional `candidates_with_ik.json`, and optional
`candidates.html`. The top 100 saved candidates per model retain the native
score/rank plus common checks. Inference time is recorded separately from
filter time; supplied predictions have null inference time.

## First same-frame smoke test

Scene: original FR3 arena `seed_1000` mustard frame, same frozen point cloud,
target mask and calibration. R1 base: `(0.329, -0.175, 0.237 m; yaw 56.3°)`.
These are diagnostic candidate counts, **not** grasp success rates.

| Provider | Native candidates | Dex1 scene collision-free | R1 IK + self collision | Margin J5/J6/J7 > 0.05 rad | Fresh inference wall time |
|---|---:|---:|---:|---:|---:|
| Frozen AnyGrasp | 20 | 1 | 0 | 0 | unavailable: frozen output |
| Graspness | 11 | 0 | 0 | 0 | 4.7 s |
| EconomicGrasp | 25 | 1 | 1 | 1 | 5.0 s |
| GraspGenX | 200 | 67 | 27 | 17 | 8.1 s |
| ZeroGrasp | 140 after native NMS | 0 | 0 | 0 | 9.5 s |

The five-provider full candidate record is
`docs/grasp_compare_seed1000_five_models.json`;
`docs/grasp_compare_seed1000_five_models.html` shows the same scene in camera coordinates
(requires Plotly CDN to load).
The earlier `docs/grasp_compare_seed1000_candidates.json` retains the
MoveIt IK details for the other four providers. ZeroGrasp had no candidate
passing the common scene collision gate, so no ZeroGrasp IK query was made.
Inference wall time includes interpreter/model startup and excludes shared
Dex1 filtering and MoveIt IK. GraspGenX produced 200 proposals; this single
scene cannot establish a grasp-success ranking. AnyGrasp was a previously
frozen top-20 baseline; the other three predictors generated fresh candidates
from the same frame. ZeroGrasp also generated fresh candidates from that
frame. Scores remain on each model's own scale. The 140 ZeroGrasp candidates
all passed the target-region filter but collided with the official Dex1
geometry during pregrasp or approach; this may reflect its generic gripper
pose distribution as well as the current mount/TCP calibration and strict
depth-cloud collision approximation.

The target-aware collision rule materially changes the result. If the entire
target mask is exempted, the same source proposals yield 74/8/18/17
collision-free candidates (GraspGenX/Graspness/EconomicGrasp/AnyGrasp).
Restricting contact exemption to the official Dex1 finger aperture gives the
67/0/1/1 counts above. Most newly rejected generic-gripper poses intersect
Dex1's palm or finger links. This strict test can also reject plausible
contacts when the depth cloud or provisional TCP is inaccurate; validate
across further shared frames before selecting a model for hardware work.

Pinned sources in this run: GraspGenX `b942909`, ZeroGrasp `152f67c`,
EconomicGrasp `4119bdc`, Graspness `b5abf5a`. Official checkpoint SHA256:
ZeroGrasp Mirage `0460111a60b31b7b35f6a256e17a95be96ff31bc4abfec3f9a341df93213bc65`,
GraspGenX generator
`8b55f31cdb8340a573b4df27b027c15cff326bd6debcb389bf631d2aaab7ac44`,
discriminator `cbf3f3bdb2e4c03fca8486ed24de0e6a8a859e6bd22bce2f1434a610335abd3e`,
EconomicGrasp `33c99bf43d599bd259c53c090badd83771ca34b8bebdb4a27a9a45a80df32223`,
Graspness `a0b9d85b300e40c76b66d092de62138d74757d6b7c2037f0d364f33960c54aa9`.

Sources: [GraspGenX](https://github.com/NVlabs/GraspGenX),
[ZeroGrasp](https://github.com/sh8/ZeroGrasp),
[RegionNormalizedGrasp](https://github.com/THU-VCLab/RegionNormalizedGrasp),
[EconomicGrasp](https://github.com/iSEE-Laboratory/EconomicGrasp),
[Graspness](https://github.com/graspnet/graspness_unofficial).
