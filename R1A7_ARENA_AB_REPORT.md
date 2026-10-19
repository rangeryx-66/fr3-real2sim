# FR3 / R1-7a + Dex1 Isaac Lab Arena matched grasp benchmark

## Protocol and provenance

The cohort is `ARENA_COMPLEX_PROTOCOL.json`: eight official Isaac Lab Arena
assets (`mustard`, `raisin`, `hidden_tuna`, `bowl`, `banana`, `sugar`, `soup`,
`mug`) with five seeded tabletop layouts each. The assets are the official USD
references recorded in `assets/arena_complex/inventory.json`; no procedural
benchmark objects are used. The same seeded layout includes the target and five
physical clutter objects. R1 uses the current recommended base pose
`(0.329, -0.175, 0.237 m; yaw 56.295°)` and a `0.10 × 0.10 × 0.20 m` mounting
pedestal.

The FR3 baseline was previously executed on all 40 layouts under
`/data1/home/rangeryx/fr3_moveit_grasp/results/arena_complex40`. That run
captured a new RGB-D frame, instance mask and point cloud, and ran AnyGrasp
once for **each** scene. This paired R1 run uses those exact per-scene captured
inputs and top-K files. It checks SHA-256 for the cloud and grasps before each
R1 trial. The archived inputs have 40 distinct cloud hashes and 40 distinct
grasp hashes. The FR3 baseline is a prior run, not a simultaneous FR3 rerun;
this choice fixes perception for the execution comparison.

The original protocol changes x/y and yaw across the five poses, but its x/y
range per asset is only about 1–3 cm near `(0.5, 0) m`. The results test
cross-asset and local pose robustness, not the full R1 tabletop workspace.

## Methods

Three arms of the comparison use the same top-K AnyGrasp output: the original
FR3 pipeline, R1 with the raw AnyGrasp pose, and R1 with a Dex1-specific local
manifold. The manifold treats each raw grasp as a surface-region seed. It
samples bounded local offsets plus poses translated toward the closest
official target-mesh surface point, depth offsets, and roll/pitch/yaw changes.
The same 24-variant budget is used for every raw candidate and every asset.
Each proposed pose is screened by Dex1 pad surface coverage and opening width,
table penetration, exact IK, joint margin, self/scene collision, and
pregrasp/approach/micro-lift/lift paths. Successful screening only admits a
candidate to full MoveIt planning and Isaac physical execution; it does not
count as grasp success.

All R1 executions use pregrasp, approach, Dex1 close, contact verification,
micro-lift, lift of at least 8 cm, and stable hold of at least 2 seconds.
One bounded re-close and depth correction, then a different adapted candidate,
can be attempted after bad contact or contact loss. Raw AnyGrasp poses,
scores, ranking, and model inference files remain unchanged. FR3Backend is
unchanged. No articulated-object stage is run.

The target collision object uses the official asset triangle mesh. The table
uses the Arena asset footprint; clutter has physical USD collision in Isaac
and conservative axis-aligned boxes in MoveIt. The latter can reject feasible
poses and is a limit on scene-collision fidelity. The pedestal dimensions are
parameters, not a measured physical mount.

## Reproduce and audit

Run `scripts/build_r1a7_arena_manifest.py` on the labserver to audit the
per-scene FR3 inputs and create reference manifests. The orchestrator
`scripts/run_r1a7_arena_ab.py` accepts separate `--objects`, `--gpu`, `--port`,
and `--ros-domain` arguments for isolated parallel batches. It stops stages at
05:00 Asia/Shanghai. Run `scripts/summarize_r1a7_arena_ab.py` after collecting
the trial JSON files to generate `results/r1a7_arena_ab/analysis.json` with
paired denominators, per-object and per-position outcomes, candidate coverage,
recorded approach trajectories, contact loss, candidate ranks, and adaptation magnitude.

On the labserver, from the `r1-7a` checkout:

```bash
export CONDA_PREFIX=/data1/home/rangeryx/fr3_moveit_grasp/ros_env
source "$CONDA_PREFIX/setup.sh"
export AMENT_PREFIX_PATH="$CONDA_PREFIX" PATH="$CONDA_PREFIX/bin:$PATH"
"$CONDA_PREFIX/bin/python" scripts/build_r1a7_arena_manifest.py
"$CONDA_PREFIX/bin/python" scripts/run_r1a7_arena_ab.py --objects mustard --gpu 1 --port 18781 --ros-domain 217
"$CONDA_PREFIX/bin/python" scripts/summarize_r1a7_arena_ab.py
```

For parallel asset batches, assign a different GPU, port, and ROS domain to
each process. Do not run the same asset from two batches concurrently. The
runner calls the Isaac Sim Python environment at
`/data1/home/rangeryx/isaaclab-arena/.venv/bin/python` and starts the R1
MoveIt launch and ROS bridge itself.

## Results

40/40 个场景均完成三组配对。每组使用对应场景的同一份 AnyGrasp top-K；80 份 R1 trial 的 cloud/grasp SHA-256、scene seed 和目标物体均通过校验。全部 28 次 R1 成功还逐条通过了原始 trial 的 ≥8 cm 抬升、≥2 s hold 及 hold 高度稳定检查。适配组成功抬升范围为 8.98–11.83 cm，hold 采样跨度至少 2.067 s。

| 指标 | FR3 原 pipeline | R1 Raw | R1 Dex1-adapted |
| --- | ---: | ---: | ---: |
| 最终抓取成功 | 19/40 (47.5%) | 5/40 (12.5%) | 23/40 (57.5%) |
| 筛选通过的 candidate | 191/687 (27.8%) | 21/687 (3.1%) | 168/687 (24.5%) |
| R1 pregrasp 规划成功 | 不按同一字段记录 | 9/40 (22.5%) | 33/40 (82.5%) |
| 记录了 approach 轨迹 | 38/40 | 9/40 | 33/40 |

“记录了 approach 轨迹”不等于整段执行成功。例如 FR3 的 `APPROACH_FAIL` trial 也可能保存 approach 轨迹。最终成功率只取物理抬升和 hold 判据。FR3 的 `VALID` 与 R1 的 `KINEMATIC_PATH_VALID` 都表示各自机器人约束下的候选筛选结果，但两个 pipeline 的具体筛选实现并非逐项相同。

| Arena 物体（每类 5 个 pose） | FR3 | R1 Raw | R1 Adapted | R1 Raw→Adapted 路径可行 candidate |
| --- | ---: | ---: | ---: | ---: |
| mustard，高瓶 | 5/5 | 1/5 | 4/5 | 2→30 / 90 |
| raisin，小盒 | 4/5 | 2/5 | 3/5 | 13→43 / 99 |
| hidden_tuna，低矮罐 | 0/5 | 0/5 | 0/5 | 0→0 / 97 |
| bowl，低矮宽口 | 0/5 | 0/5 | 0/5 | 0→6 / 100 |
| banana，长条 | 1/5 | 0/5 | 4/5 | 0→29 / 100 |
| sugar，大盒 | 4/5 | 1/5 | 4/5 | 4→24 / 100 |
| soup，圆柱罐 | 5/5 | 1/5 | 5/5 | 2→30 / 51 |
| mug，带把手 | 0/5 | 0/5 | 3/5 | 0→6 / 50 |

| 同一物体的 pose 序号（各含 8 类物体） | FR3 | R1 Raw | R1 Adapted |
| --- | ---: | ---: | ---: |
| 1 | 4/8 | 3/8 | 4/8 |
| 2 | 3/8 | 0/8 | 5/8 |
| 3 | 4/8 | 1/8 | 5/8 |
| 4 | 4/8 | 1/8 | 5/8 |
| 5 | 4/8 | 0/8 | 4/8 |

原 Arena 协议的 yaw 覆盖较广，但每类的 x 跨度仅 1.2–2.4 cm、y 跨度仅 0.8–2.6 cm。因此上表支持“跨物体和 yaw、局部桌面位置”泛化，**不能**证明整个 R1 工作区的大范围位置泛化。所有物体的姿态/位置精确值保存在 `analysis.json` 的逐场景记录中。

### 失败链与剩余差距

R1 Raw 的 35 次失败中，31 次终局为 `NO_EXECUTABLE_CANDIDATE`、3 次 `CONTACT_LOSS`、1 次 `BAD_CONTACT`。候选级 687 个 raw pose 的筛选状态中，碰撞 346、接触几何不合格 217、`NO_IK` 96；原 pose 的主要瓶颈并非单纯 IK。

R1 Adapted 的 17 次失败中，终局为 `NO_EXECUTABLE_CANDIDATE` 6、`NO_PLAN` 6、`COLLISION` 2、`BAD_GRASP_GEOMETRY` 2、`LOW_JOINT_MARGIN` 1；无终局 `DROP`。适配过程在 11 个场景出现 12 次 `CONTACT_LOSS` 事件，其中 4 个场景最终通过恢复或切换候选成功。终局类别不足以描述完整因果链：bowl 的 5 次都先发生 micro-lift 接触丢失，随后 re-close 的返回 Cartesian 路径只完成 50%，终局记为 `NO_PLAN`。

hidden_tuna 的 97 个 raw candidate 无一通过适配筛选。多次 raw 姿态使 Dex1 pad 穿入桌面约 27–32 mm；抬高或改变姿态后常失去双侧 pad 接触。其变体失败主要是 table/scene collision 与接触几何，而不是 `NO_IK`。FR3 在这 5 个场景也全是 `APPROACH_FAIL`。bowl 的 6 个适配后路径可行候选均未转化为稳定接触；FR3 的 5 次也全是 `BAD_CONTACT`。这两个物体不支持“当前适配已覆盖所有低矮/宽口物体”的结论。

对于 FR3 原本稳定的 mustard、raisin、soup、sugar，R1 Adapted 分别为 4/5、3/5、5/5、4/5，对应 FR3 为 5/5、4/5、5/5、4/5。剩余差距主要是个别碰撞或几何筛选；不是多数场景的 `NO_IK`。banana 和 mug 的适配结果分别为 4/5、3/5，均高于这套 FR3 原 pipeline 的 1/5、0/5，但这些是同一小样本协议内的结果，不能推断所有长条物或杯子都更适合 R1。

### 是否依赖固定偏移或 rank

23 次成功所选适配位移为 0–30.2 mm，中位数 13.1 mm；只有 2/23 落在 25±2.5 mm。成功位移分布为 `<10 mm: 10`、`10–20 mm: 7`、`20–30 mm: 4`、`≥30 mm: 2`。成功旋转为 `0°: 16`、`10°: 3`、`20°: 3`、`30°: 1`。成功所选 raw candidate 的 rank 覆盖 0–19，其中 8/23 的 rank ≥10；没有固定 top-1 依赖。

全部 168 个路径可行适配 candidate 的最小 joint-limit margin 中位数是 0.284 rad，最小值 0.051 rad；161/168 大于 0.08 rad。23 次最终成功的 margin 均大于 0.08 rad，最小为 0.147 rad。搜索没有通过普遍贴近 joint limit 来换取成功率。banana 的 1 次失败仍是 margin 不足。

### 结论与限制

在这套 8 个官方资产、40 个匹配输入的协议内，Dex1-specific adaptation 从 Raw 的 5/40 提升到 23/40，并超过 FR3 原 pipeline 的 19/40。提升跨越盒、瓶、圆柱罐、长条和带把手物体，且不依赖固定 25 mm offset 或固定候选 rank。当前主要未解决的是低矮/宽口目标的 finger/table 几何与真实接触；已筛选可行的候选在接触恢复阶段仍可能因 Cartesian 规划失败。当前结果足以支持继续研究普通刚体抓取泛化，但不足以声明整个桌面工作区或所有物体都已稳定抓取，也不应据此进入铰链交互。

本轮没有重新采集一套新的 FR3 相机数据；使用的是原 FR3 Arena 40 场景中**每个场景独立采集、独立运行 AnyGrasp**的历史输入，以固定感知差异。要验证更大 x/y 工作区或不同成像条件，需要新建更宽位置的配对场景，并对 FR3/R1 共同重跑感知与物理执行。现场结果和对应 top-K JSON 均随本报告保存；原 RGB-D/点云继续保存在 manifest 指明的服务器路径，未纳入 Git（约 106 MB）。

Git 中的 `results/r1a7_arena_ab/trial_records.tar.gz` 保存了 40 份 FR3 基线、80 份 R1 trial 和 16 份阶段 summary；`analysis.json` 可直接查看逐场景统计。解包原始 trial 可运行 `tar -xzf results/r1a7_arena_ab/trial_records.tar.gz -C results/r1a7_arena_ab`。
