# Existing dataset handle selection — PiPER fixed base

2026-10-02，分支 `codex/piper-mobile-door`。这次没有获得合法真实 grasp，也没有完成 articulated motion；47686 是被否决资产的诊断样例，不是已验证优于 microwave 的替代品。

## 数据与自动筛选

- prepared 中有 7 个有效 manifest，对应 6 个不同 asset；35059 有两个版本。
- 同服务器已解压的完整 PhysX-Mobility 包有 25 个 URDF，活动关节全部为 revolute。没有 prismatic drawer，因此没有运行 drawer blind probe。
- `rank_prepared_handle_assets.py` 从实际 URDF 推导活动子树，应用 visual/collision origin、scale、FK，估计面板法线、横杆轴、两侧面片面积及三个中央截面的突出量/后方间隙。没有 object-ID 分支。
- 这些射线间隙是有符号轴向测量，**不是完整指体空间或全局 graspability 证明**。尺寸初筛来自官方 100 mm 开口、25.43 × 10.02 mm 指垫；局部后方间隙只作保守筛选，最终以完整几何和实际 closure 为准。
- 本轮没有资产通过全部几何初筛。35059/46744 的突出部件主要为短小、非长横杆候选；7236 的暂定 handle 标签不能直接作为可夹把手；7320 和 47686 的视觉空隙未被 prepared 凸碰撞体保留。
- 默认预检入口在无合格资产时返回 `NO_SCREEN_QUALIFIED_ASSET`。`--diagnose-rejected` 明确启用负例诊断，自动挑视觉/碰撞后方空间差异较大的横杆。本次为 **47686 / original-29 / revolute**。

47686 视觉横杆约 44.4 mm 长、18.4 mm 宽，中央横杆厚约 6.20 mm、后方视觉间隙约 7.85 mm。但实际碰撞模型没有保留这片空隙。不能仅凭视觉形状声称它更适合 PiPER。

## 模型、控制与放置

PiPER base 保持 `(0.5, -0.55, -0.10 m; yaw=150°)`。新诊断柜体摆放固定为 `(0.41, 0, yaw=-90°)`，沿用 40 mm 参数化支撑。未搜索或移动 base。

资产 wrapper 只补充从 URDF 推导的 `moving_link` / `door_link`，其余文件通过 symlink 指向原 prepared 目录。没有改 asset URDF、mesh、尺寸、joint、mass 或摩擦。执行沿用现有 loader 的 COM/inertia 近似和 PiPER drive profile，不是新增物理调参；仍不是经过真实硬件标定的模型。

指垫中心和坐标由官方 ownership surface 与 FK 得到。局部搜索三个横杆位置、深度 ±3 mm、四个绕接近轴姿态及 ±15° 方向扰动。closure 始终实际从打开状态向零命令闭合，不把点云估计 aperture 当闭合终点。原生 metal 接触即停；actual joint margin 必须 >0.05 rad。

## Candidate funnel / 实测

| 阶段 | 数量 |
|---|---:|
| 搜索姿态 | 324 |
| Exact IK 可解 | 78 |
| IK + 原生碰撞 + approach + pregrasp 路径 | 3 |
| 实际 PhysX closure 尝试 | 3 |
| 合法双侧 pad-only closure | **0** |
| 合法 1–2 mm pull | **0，未执行** |
| 1° / 5° 开门 | **未执行** |

| 候选 | 停止时实际 aperture | 两侧 pad force (N) | metal contact 数 | 全程最小 margin |
|---|---:|---:|---:|---:|
| dataset_205 | 44.52 mm | 0.0057 / 0 | 2 | 0.24709 rad |
| dataset_313 | 71.14 mm | 0.7037 / 0 | 2 | 0.26272 rad |
| dataset_314 | 71.25 mm | 0.1146 / 0 | 2 | 0.25599 rad |

三例均在 CLOSE 阶段 `NATIVE_FORBIDDEN_CONTACT`。最大记录门角约 6.7e-9°，是数值噪声，没有真实开门。未建立合法 grasp reference，因此 slip **不可测**，视频 overlay 的默认 `slip=0` 不能当作“无滑移”的证据。

## Native cooked / raw 核对

实际 native handle collider 来自 `original-29.obj.convex.stl`，通过唯一 authored mesh identity 映射为 pad 目标；没有允许整扇门接触。点云最近表面投票仅 73.8%，因此记录显式映射，未放宽投票阈值或用运行时 contact point 猜 ownership。

本次导出的 authored raw collision 与实际 PhysX cooked shape：

- 体积均为约 `2.36952647e-5 m³`；bounds 相同。
- 中心射线厚度均约 **32.096 mm**；视觉横杆厚约 **6.203 mm**。
- 因而空隙填充已存在于 prepared 单凸体，不能归因于这次 PhysX cooking 膨胀。

对三例全部已记录 CLOSE samples 再做 cooked + official raw finger audit，分别检查 510 / 271 / 270 samples；最终统一拒绝，违反项均为 `METAL_CONTACT`，native owner mismatch = 0。

**几何相交与接触带必须区分：**在停止采样点，FCL 硬相交检查仍为 FREE；PhysX 已在既有 contactOffset 的正间隙范围产生 metal 接触（205 首次约 +0.041 mm separation）。综合 validator 使用原生 collider ownership 一并拒绝。这不是“pad-only 成功 / offline metal penetration”矛盾，也不能把它描述成 raw mesh 已发生穿透。没有修改 contactOffset/restOffset 或新增容差。

## 结论与边界

本轮没有找到满足全部条件的更合适资产。可执行入口、自动初筛、负例物理试验和审计已运行，但目标“合法抓取后产生真实 articulated motion”未达成。

证据支持优先审计 **asset 侧 prepared convex proxy 是否忠实保留把手空隙，以及候选闭合轨迹是否先碰安装脚/主体**；不能保证仅做 decomposition 就会成功。此次 3 个闭合失败不是后段 IK 或 joint-margin 失败。没有据此启用 mobile base，也没有扩大开门角。

这只是当前初筛及 324 个局部姿态的负结果，不证明所有资产/所有姿态在数学上不可抓。若后续允许修正资产 collision，应保留真实视觉表面和尺寸，对非凸把手做等价分解后重新导出 cooked shapes，再重复同样的严格 closure gate；本轮没有擅自修改该几何。

Articulation identification **未执行**：URDF 的 revolute 标注仅是 GT；没有合法交互轨迹可以报告 blind identification 准确率。

## 复现与证据

服务器结果：`results/dataset_handle_selection/`；本地完整 curated delivery：`results/dataset_handle_selection_delivery/`。其中保存 overview / 三例真实 closure 视频、native SDK shapes、局部搜索 pose、全 closure observations、审计和摘要。详细指标另见本目录 `dataset_handle_selection_results.json`。

```bash
python scripts/rank_prepared_handle_assets.py \
  --prepared /data1/home/rangeryx/datasets/physx_mobility/prepared \
  --source-dataset /data1/home/rangeryx/datasets/physx_mobility/extracted/PhysX_mobility \
  --output results/dataset_handle_selection/ranking.json

python scripts/dataset_handle_preflight.py \
  --ranking results/dataset_handle_selection/ranking.json \
  --fixed-base-report results/piper_fixed_center_verified/report.json \
  --output results/dataset_handle_selection/provisional --diagnose-rejected

# Export the actual Isaac geometry using piper_owned_grasp_trial.py --overview-only
# with --target-mesh-stem from the selected dataset mesh (original-29 in this run).
python scripts/dataset_handle_preflight.py \
  --ranking results/dataset_handle_selection/ranking.json \
  --fixed-base-report results/piper_fixed_center_verified/report.json \
  --output results/dataset_handle_selection/provisional \
  --native-export results/dataset_handle_selection/overview_verified/cooked_initial.json \
  --association results/dataset_handle_selection/overview_verified/target_collider_association.json
```

每候选独立 fresh episode，使用原 `run_piper_owned_trials.py`，新增可选 `--target-mesh-stem` / `--opening-goals 1 5`；默认参数保持原实验。runner 与 Isaac trial 都检查上海时间次日 05:00 截止。本轮在当日约 18:06 完成物理测试，已停止模拟。
