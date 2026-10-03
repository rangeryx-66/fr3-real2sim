# Mobile recovery 与 physics oracle 分解

本入口独立于 `f5dcc6f` / `cd61660` 的成功抓取与未知铰链实验。实验模式是
`SIM_TO_SIM_BLIND_SYSID`，不是真机 Real2Sim 验证。

## 运行

在服务器仓库根目录执行（配置中的数据与 Python 路径对应 labserver）：

```bash
env -u PYTHONPATH OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_interactive_twin_recovery_benchmark.py \
  --config configs/interactive_twin_recovery.yaml --stage full
```

阶段可选 `mobile / oracle / prior / full / summary`。`oracle` 和 `prior` 要求
12 个冻结配置全部有结果。时间截止为配置中的上海时间；当前为 2026-10-04 05:00。
结果位于 `results/interactive_twin_recovery_20261003_v2`。

## Mobile recovery

- 固定底座先复测原 12 个 handle-local 抓取候选；仅 IK、关节余量或机械臂路径失败触发恢复。
- 从初始视觉 handle frame 生成最多 27 个 SE(2) 安装位姿，z 不变；最多 4 个位姿进入完整路径检查。
- 检查底盘、环境、机械臂 home、pregrasp、approach、grasp、10 mm 局部未知关节 probe 空间。
- 排序使用最小关节余量、底盘环境间隙、行程和机械臂平移 Jacobian 的最小奇异值。
- 检查平移/旋转顺序的两种路径；没有实现通用绕障导航。
- Isaac 中底盘按 0.01 m/s、5°/s 上限重定位，机械臂保持 home。停止并稳定后才开始原物理抓取流程。
- 平台是 **kinematic SE(2) 模拟**，未模拟轮胎、底盘动力学或真机导航。底盘 box 为 0.34 × 0.30 × 0.20 m。
- 接触失败、滑移、辨识失败都不会再次移动底盘。

官方夹爪、interaction proxy、IK 参数、0.05 rad 余量、摩擦、夹紧力和危险接触检查不变。
移动阶段没有物体状态设置、直接物体力或 attachment。

`cross_object_mobile.csv` 区分规划可行、真实抓取、辨识、实际 5° 操作。
仅通过底盘/抓取预检不计为 manipulation success。抓取前自行开启的门也不计为成功。
所有失败保留在 12 个场景的分母中。

## Physics 分解

| 组 | 关节结构 | 阻力参数 | 用途 |
|---|---|---|---|
| P0 | GT | GT | 仅事后 oracle 对照 |
| P1 wrong prior | GT | 冻结错误 prior | physics 更新前 |
| P1 | GT | P1/P2/P3 训练动作拟合 | 隔离 physics sysID |
| P2 | EE 估计 | GT | 隔离 kinematic error |
| P3 | EE 估计 | 训练动作拟合 | 当前完整方法 |

GT 只用于诊断环境构建，不传给控制器或参数估计器。所有 replay 都向自主演化的
Isaac 物理环境重发同一条 command tape；不回放观测 q、门角或物体轨迹。
P4 是未参与参数选择的 held-out 动作。

沿用冻结的 robot-only calibration、LOW/MED/HIGH sensitivity 数据和 noise scales。
P1 使用最多 9 次原生 train rollout。P3 仅在配置、命令、初始 estimate 和几何 hash
一致时显式复用此前的 9 次原生训练日志；文件保留 import provenance，不能称为新执行。

如果 P1 不能改善错误 physics prior 的 held-out 预测，停止 physics 扩展。
仍输出 P0/P2 诊断，并明确标记未运行/未接受的 P1/P3。不会继续 45621 optimizer。
参数只能称为 **effective simulator resistance parameters**；当前没有经标定电流或外部 torque scale。

## 独立 visual prior

`visual_prior` 使用初始可见 moving-part bounds、handle、默认竖直轴、冻结 5° 轴扰动和
20 mm axis-line 偏移，不把数据集 GT URDF 叫作视觉重建结果。它是构造的 imperfect
visual/default prior。独立 `T_prior` 的关节 stop 使用已冻结 DEV ±10° 默认窗口，
不继承数据集 GT stops；该窗口不代表真实完整关节范围。仅当 P1 gate 通过后比较其 held-out 响应与 P3。
不宣称几何精度改善或完整关节限位已识别。

## 可追溯性与恢复

- `frozen_manifest.json`：保留首次冻结的 12 个场景、输入和代码 hash。
- `implementation_amendments.json`：仅记录通用实现修复的 hash 链；不能改变参数、资产或控制 baseline。
- 每次原生运行保留 `job_private.json / cache_identity.json / process.json / report.json`。
- 失败原始报告中的 NaN 不被覆盖；汇总文件将不可用指标写为 null。
- 断点续跑保留已有候选和已消耗预算，不通过重启刷新失败资产预算。
- `base_reposition.mp4` 与 `contact_baseline.mp4` 分别记录重定位和接触操作。

只读生成当前报告：`python scripts/summarize_interactive_twin_recovery.py --output results/interactive_twin_recovery_20261003_v2`。

已修复的通用问题包括：移动阶段 clock callback 释放、相同 FCL solid 的缓存、失败
汇总 NaN 处理。缓存检查曾与原 PhysicalScene 在 12 个路径/q 状态逐一对照，判定一致。

## 证据边界

控制与拟合不读取 GT hinge，但当前安全 supervisor 仍使用 PhysX 接触信号；原厂
PiPER SDK 的可用信号不能等同于独立双指 tactile 或腕部 F/T。完整相对滑移不是在线
可观测量，事后 final relative transform 不能标成全程最大滑移。跨物体成功率、辨识误差
和 physics held-out 改善必须分别报告，不能互相替代。
