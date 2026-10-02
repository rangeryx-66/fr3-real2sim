# PiPER 真实接触、未知 articulation 实验（2026-10-03）

**本轮未跑通未知铰链开门。最终为 UNOBSERVABLE，不能继承 known-model baseline 的 4/4 成功率。**

`f5dcc6f` 的执行入口、接触检查、闭合/保持控制、机器人描述生成器、IK 和力/滑移配置逐字节未修改，见 `frozen_baseline.json`。同一 base、proxy 和 nominal grasp；官方整根 finger collider 仍各一个。新代码只有独立实验入口与观测/探索模块。没有 attachment/weld、门外力或执行期门 joint state 设置。

## 可复现命令（服务器）

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
env -u PYTHONPATH -u CUDA_VISIBLE_DEVICES \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python \
  scripts/run_piper_unknown_interaction.py \
  --source /data1/home/rangeryx/fr3_real2sim_r1a7/results/microwave_7320_table_edge_preflight_full_rgb \
  --asset-root results/semantic_interaction/asset \
  --plan results/semantic_interaction/nominal_minimal/plan.json \
  --output results/unknown_contact_reproduction --gpu 6 --repeat 3
```

这条命令复现当前实验流程，**不是成功开门命令**。默认到下一个上海时间 05:00 停止；也可用 `--deadline-shanghai` 显式指定。只有首次成功才重复三次；失败后运行同初始状态、实际抓取的零探索对照。没有搜索其他 grasp。

## GT 隔离与流程

- 数据集 joint 信息仅供仿真场景组装；运行控制器的机器人模型不加载 object URDF/joint。
- 对对象 joint、axis/origin 及 link pose 查询设置运行期访问保护。控制器仅使用机器人状态、EE、左右接触载荷和实际 RGB-D。
- 实际完成原 approach、慢闭合和双侧保持后，使用受力限速、有限 Cartesian lead、受限 jaw-balance 顺应和观测姿态跟随；无预生成 hinge arc。目前是位置命令上的有限 lead/接触反馈，**不是已验证的六轴 F/T admittance controller**。
- 保留闭合保持时的命令/实测偏差，避免把重力下沉重复积分成下降命令。既有 IK、robot drive gains、force limits 不变。
- 拟合仅接收实测 EE SE(3)，复用现有 `fit_articulation`。只有高质量 revolute fit 才保存估计并使用其轴线生成后续切向控制；**该后续分支本轮未获得实际接触验证**。
- 所有试验先暂停仿真、保存 fit，再读取一次最终门角及静态 GT axis/origin 做评估。没有执行期 GT moving-link trajectory。

## 实测结果

| 试验 | 探索 EE 位移 | EE 转角跨度 | 事后实际门角 | 结果 |
|---|---:|---:|---:|---|
| v4：保持偏差修正后 | 0.293 mm | 0.073° | 0.0384° | 持续单侧失载，停机 |
| v5：视觉滤波与 jaw 顺应 | 0.543 mm | 0.113° | 0.0518° | 持续单侧失载，停机 |
| v6：角向跟随试验 | 0.380 mm | 0.834° | 0.0516° | 相对旋转漂移、观测丢失，停机；此控制未保留 |
| **v7：当前 RGB-D/optical PnP** | **0.055 mm / 0.10 s** | **0.00255°** | **0.00764°** | **RGBD_TRACKING_LOSS → UNOBSERVABLE** |
| v7：zero-probe，实际保持 25 s | 约 3.1 µm；fitter travel 4.3 µm | 0.00047° | 约 0° | UNOBSERVABLE，无自开门 |

最终 v7 停机时，左右载荷仍 **0.503 / 0.542 N**，没有持续失载；最大记录的观测相对平移为 **0.0771 mm**。全流程最小 joint margin **0.06349 rad**，探索阶段最小 **0.18233 rad**；没有 IK、joint margin 或危险碰撞停机。

所有探索均未达到可辨识运动跨度，axis error / axis-line error **不可报告（N/A）**，没有基于估计继续到 5°，没有首次成功后的三次重复。

## 第一次实际阻碍与对照证据

当前 v7 在 130 个有效图像对应、0.195 px 重投影误差时，深度一致性 RMSE 达到 **1.17 mm**，观测被判无效并停机。更关键的是：**zero-probe 的机器人/门基本静止、双侧接触稳定，但 PnP 的后抓取有效观测帧只有 5.35%**，深度一致性误差最高约 4.87 mm。因此不能把视觉失效或相对位姿估计直接解释为真实滑脱。

当前瓶颈在 **小运动的可观测性及 compliant following**；未进入 articulation fitting 的有效辨识阶段，不是已证明的 IK/reachability 限制。早期视觉跟随还会造成单侧失载，说明观测误差能够破坏已建立的抓取。

下一步应先在冻结 baseline 的已有记录/独立观测诊断中验证 moving-part RGB-D 跟踪的深度边缘、静/动部分混合及姿态不确定性，再验证未知模型的角向顺应；不继续改 geometry、grasp、friction、preload 或 margin，也不靠继续加大动作获得 fit。

## 记录与限制

- `unknown_probe_full.mp4`：最新真实接近、闭合、短探索、停机连续视频；**不是成功开门视频**。
- `unknown_probe_closeup.mp4`：原视频末段，正常速度，无合成运动。
- `probe_trajectory.png`：v5 与当前 v7 的机器人 EE、native 双侧载荷及 RGB-D 估计滑移。
- `verified_summary.json`：逐轮结果、GT 评估、观测有效率；原报告的 observer 文本标签沿用了旧 ICP 名称，实际观测方法以记录的 `method` / summary 为准。
- slip 是 **RGB-D 观测估计，不是 simulator-GT 的真实最大滑移**；失效/冻结观测段不提供可靠完整滑移界。没有记录运行期 GT 对象轨迹。
- q/EE/force 及观测变换完整 240 Hz 日志、实际 PhysX 子步、cooked export 和原始视频保存在服务器 `results/unknown_contact_v1...v7`。公开 control trace 下采样至 30 Hz，probe trace 保留 240 Hz。
- v1 的重力跟踪偏差累积 bug、v2 的 OpenCV 大数组错误已修正；它们不算成功/可靠性重复。中途取消的调试/重复对照也不计入完整试验。

检查：原 baseline 文件哈希不变；运行期 GT 访问保护测试通过；robot-only model 无 asset 定义；full-image RGB-D 反投影与 PnP 已知位移检查通过；现有圆弧/直线/激励不足 fitting 回归通过。截止入口检查通过：过期 deadline 下不加载场景、不启动试验/对照，输出零次试验。

**这些检查不替代真实闭环成功，当前仍未达到 5°。**
