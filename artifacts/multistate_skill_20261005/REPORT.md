# Articulated interaction skill：实际多状态采集

2026-10-05；基于 `codex/piper-mobile-door` 的 `47b8d42`。
保留 f5dcc6f / cd61660，原控制、接触、几何及安全文件未修改。
本轮为仿真实接触采集，没有 physics fitting、attachment 或物体直接驱动。

## 一个运行命令

在服务器 `/data1/home/rangeryx/fr3_real2sim_piper_mobile` 执行：

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/articulated_interaction_skill.py \
  --manifest configs/articulated_interaction_capture.json \
  --output results/articulated_skill_reproduce --gpu 0 \
  --ffmpeg /data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg
```

该 manifest 复用服务器上已保存的实际物理 job；不是独立于资产/环境的安装包。
微波炉请求默认多状态采集；抽屉请求短程 effort probe（0/1/2 cm）。
接口另支持默认 0/2/4/6/8 cm 采集；所有目标均受原安全及运动预算限制。

## 实际结果

| 数据集对象 | 保存的实际状态（事后独立评估） | 状态数 / 相机数 | 最终实际运动 | 最小 joint margin | 停止原因 |
|---|---|---|---|---|---|
| 7320 微波炉，revolute | 0/2.68/5.21/7.73/10.23/12.76/15.29° | 7 / 每状态 2 | 16.69° | 0.06349 rad | 冻结的 0.12 m EE 路程预算；未到 20° |
| 45746 抽屉，prismatic，短程复测 | 0/1.007/1.994 cm | 3 / 每状态 2 | 1.996 cm | 0.17157 rad | 请求目标完成 |
| 45746 抽屉，长程采集 | 0/1.007/1.994/2.965/4.007/5.007/6.011 cm | 7 / 每状态 2 | 6.885 cm | 0.049994 rad | LOW_JOINT_MARGIN；未到 8 cm |

长程抽屉保护在离散物理子步检测到余量略低于 0.05 后停止，不能称全程满足 >0.05。
短程复测达到三个物理不同的状态，未触发该问题。
三个实验的最终 gripper–moving-link 相对平移漂移分别为 0.102 / 0.458 / 1.789 mm。
在线 contact-plane drift 不是完整滑移；真实相对变换只由事后独立日志评估。
实际关节/物体轨迹没有进入在线轨迹生成。

每状态保存 1280×960 RGB、米制 optical-Z depth、instance mask、moving-part mask、
内参/相机世界变换、世界点云、object/part ID、估计状态、base/TCP/grasp 变换及 hold 标记。
mask 是仿真渲染实例观测，不是实际 SAM 或独立真机感知。
当前 grasp pose 为实测官方 TCP proxy；没有声称测出了接触中心。
终止原因记录到最后一个已安全采集的状态，故不表示该状态拍摄瞬间已经失败。

## Effort：测量与 proxy 分开

| 实验 | 初始静止启动 effort | 首个低速运动段均值 | 来源 |
|---|---:|---:|---|
| 7320 短程 effort 回归 | 0.00899 N | 0.00180 N | command-equivalent proxy；不是实测切向接触力 |
| 45746 短程复测 | 0.21771 N | 0.06686 N，标准差 0.08283 N | PhysX 法向＋摩擦缓冲区，双侧覆盖验证通过 |

“初始启动”数值是运动起始之前的有效 effort 峰值，起始判据为 EE 行程 ≥0.25 mm。
它包括机器人/抓持瞬态，不证明最小 breakaway 力，不等于精确铰链摩擦。
45746 独立物体日志的同阈值起始时刻，接触力投影为 0.01892 N，之前峰值为 0.21771 N；
两者均保留，不能将峰值自动解释为静摩擦。
7320 尚无经验证的完整法向＋摩擦测量，按用户允许的 proxy 分支记录，不能与抽屉测量直接比较。
开状态停留后再启动与初始闭合启动分开；连续 1 mm 段不伪装成 breakaway。
逐段 moving effort、标准差、起止状态、样本数、来源见 `*_effort_summary.json` / `*_effort_vs_state.csv`。
所有原始子步日志留在服务器，短程抽屉 `effort_samples.jsonl` 也已下载。
没有拟合 tau_c/b/tau_s，没有宣称精确力矩或真机 friction measurement。

## Reconstruction / twin 接口

- ART-style：按状态分组的 RGB、mask、校准相机及可选估计状态标签。官方项目未给出可验证的公开执行 loader/checkpoint，未宣称 ART inference 已运行。
- Ditto：相邻及 closed–final pair；共享世界坐标/归一化；`pc_start/pc_end` float32 `(1,8192,3)`。实际 torch adapter 读取验证通过。
- HouseDitto：before/after RGB-D/点云观测 manifest；未声称实现全部后端专用 preprocessing/inference。
- URDF-Anything+：可选图像/米制几何输入。
- `twin_update.json`：估计结构、已观测范围、effort 和 provenance。轴在 world frame，真正 URDF 写回须转到 joint parent frame；未覆盖源资产，不推断完整 joint limits，不宣称 geometry reconstruction 完成。

输入包已齐备，后端生成器尚未实际运行；这证明交互采集和输入组织，不证明 ART 重建质量。

## 保留的失败与能力边界

19179 三个原 family 内候选都未建立稳定 bilateral hold；45746 的另外两个候选也失败。
它们全部保留，成功候选没有增加夹紧力、改变对象尺寸或接触阈值。
早期预检中 27044/29921/48169 不可达，102085 不符合 bar/aperture 筛选；原日志留存。
GPU OOM 与新资产继承旧 proxy fingerprint 是 setup/infrastructure 失败，不算物理 grasp 结果。
已修复新资产 job 元数据，仍保留 fingerprint 校验；没有修改执行碰撞几何。

两个成功对象均未使用 mobile recovery。已有初始 SE(2) recovery 可复用；
中途自动 release/retreat/reobserve/regrasp 尚未实现，绝不抓持中移动底盘。
后续不可达返回 `REPOSITION_REQUIRES_SAFE_REGRASP`，或由原余量保护停止。
本轮未达到完整 20°/8 cm，也不是跨资产泛化统计。

## 文件与验证

统一入口：`scripts/articulated_interaction_skill.py`。
Agent skill：`skills/articulated-interaction/SKILL.md`（仓库内，未全局安装）。
文档：`docs/ARTICULATED_INTERACTION_SKILL.md`。
7 项单测通过，skill 格式校验通过，保留的 baseline hashes 见 `frozen_baseline_hashes.json`。
主表：`summary.csv`；完整结果及失败见本目录 JSON。
连续视频和图像包在本地 `results/multistate_skill_delivery/`，大文件不纳入 Git。

结论：revolute 和 prismatic 均已在真实接触仿真中采到至少三个稳定多状态观测。
多状态数据、effective effort/proxy 与 reconstruction 输入已交付；
中途换位续抓和后端重建执行仍未证明。
