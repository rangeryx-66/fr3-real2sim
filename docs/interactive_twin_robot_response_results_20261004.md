# 共享机器人响应与物体被动阻力分离

基线：`79b100c`。模式：**SIM_TO_SIM_BLIND_SYSID**。

## 实验边界

本轮只增加独立校准、误差传播、有限连续参数搜索和结果汇总入口。
原 grasp/contact/proxy/mobile/probe/fitter/controller/安全判据均未修改；
物体仍只使用 M2（`tau_c,b`）或经原 adequacy gate 触发的 M3
（`tau_s,tau_c,b`）。同一配置、参数范围和计算预算应用于三个条件。

以下结果属于**从已建立抓持的共同可观测状态出发的条件响应预测**。
接触在原生物理中重新建立，之后自主积分；没有 attachment、物体直接驱动、
逐帧 q/门角重放。本轮不能替代从接近开始的完整任务验证，也不是实机摩擦测量。

## 共享 robot calibration

只使用无物体接触日志拟合一次：

- 额外延迟 `Delta_t ≈ 0 s`（落在冻结下界）；
- 等效笛卡尔响应时间常数 `T = 0.0204201 s`；
- 原控制器 K、D、速度与力上限保持不变；
- 首次启动/停止用于拟合，后一次启动/停止用于验证；
- 7320、45621 的已有部署姿态各增加两次无接触原生验证，不重新拟合。

| 独立验证 | probe 方向位置 RMSE（mm） | normalized residual | 冻结门槛 |
|---|---:|---:|---:|
| 原 robot-only 日志验证段 | 0.001089 | 5.660 | 10 |
| 7320 部署姿态 | 0.001072 | 5.579 | 10 |
| 45621 部署姿态 | 0.001053 | 5.476 | 10 |

参数局部 covariance（顺序为 Delta_t、T，单位 s²）：

```text
[[ 2.86916e-9, -7.85123e-10],
 [-7.85123e-10, 4.17107e-9 ]]
```

这仅描述确定性仿真中两个接近部署姿态下的等效响应与条件校准不确定性。
不能解释为 PiPER 固件延迟、真机传感精度或任意姿态下都成立的电机模型。
原生 PhysX 已包含机器人响应，因此未额外串联第二个响应滤波器。

## 误差与冻结协议

保留原 normalized RMS=10 门槛，使用
`Sigma_repeat + J_structure Sigma_structure J_structure^T + J_robot Sigma_robot J_robot^T`。
尺度只由校准/训练数据计算，不用测试残差膨胀噪声。

传播后的三轴位置标准差（局部 EE 坐标，单位 μm）：

| 来源 | 7320 | 45621 |
|---|---|---|
| 确定性 repeat | 0.177 / 0.225 / 0.131 | 相同 |
| 结构不确定性 | 30.884 / 2.603 / 176.021 | 31.756 / 14.460 / 199.866 |
| 共享机器人参数 covariance | 0.0008 / 0.0011 / 0.0203 | 0.0009 / 0.0017 / 0.0209 |

这里的机器人项是**参数估计不确定性**，不是机器人模型残差本身；
约 1 μm 的无接触验证残差没有被当成可随意放大的噪声。
结构传播采用冻结错误物理 prior 附近的局部原生有限差分，
它有局部近似的限制，不能因 covariance 变宽就认定物理模型准确。

45621 旧结构候选只提供二维响应跨度，原结构 covariance 为四维。
测试前已补充两个缺失方向的正负有限差分，共 4 个导数模型、12 个训练回放。
这些模型只用于误差传播，没有进入候选选择，没有调用或修改 fitter。

连续优化沿用原范围：每个条件最多两个不同结构起点、每起点 12 次参数评估。
训练动作决定优化方向，完整 stop/dwell 动作参与原来的批量验证选模。
不顺序硬删候选，不用测试选择参数。support 是有限候选似然支持集，
不是经过覆盖率标定的连续置信区间。

两个预冻结新动作：

| 测试 | 速度（mm/s） | 任务驱动时间窗口（s） | 总时长 |
|---|---:|---|---:|
| 新速度脉冲 | 0.400 | 1.25–5.25 | 10 s |
| 新启动/停止/重启 | 0.450 | 0.75–2.75、5.25–8.75 | 12 s |

最初草案的 0.375 mm/s 已出现在旧测试，因此在拟合和新测试之前改为
0.400 mm/s；两版注册均保留。结构响应跨度的补齐也发生在任何新测试之前。
测试开始后未改选模方法、权重、阈值、候选结构或参数。

物体预测的 EE 位置 RMSE 沿用旧实现：对时间和三个位置分量的平方误差共同取均值再开根号。
stop/dwell 指驱动关闭时相对于关闭瞬间的增量误差，不是绝对位置偏差。
除旧评分器的首次启动指标外，交付表还单独记录每个重启脉冲的 onset error；
这些新增诊断不参与模型选择。

## 运行

单一服务器命令见 [运行说明](interactive_twin_robot_response.md)。
完整原生结果保存在服务器
`results/interactive_twin_robot_response_20261004_v2`。

## 实测主表

共完成 **312 次原生仿真**：4 次无接触 robot transfer，308 次物体响应回放。
以下 P / R 分别表示新速度脉冲 / 新启停重启测试；位置与 dwell 单位为 mm，延迟单位为 ms。
A 是规范结构候选加错误物理 prior；B 只在现有结构候选中选模型，物理 prior 相同；
C 联合选既有结构与 M2 参数。代表模型在新测试执行前冻结。

| 条件 | 模型 | validation normalized residual | P RMSE | R RMSE | P / R 首次启动误差 (ms) | P / R stop/dwell RMSE |
|---|---|---:|---:|---:|---:|---:|
| 7320 原阻力 | A_wrong_prior | 0.3586 | 0.091148 | 0.100828 | 45.83 / 229.17 | 0.016550 / 0.005166 |
| 7320 原阻力 | B_structure_only | 0.2772 | 0.060427 | 0.070477 | 12.50 / 41.67 | 0.036983 / 0.018428 |
| 7320 原阻力 | C_M2 | 0.1092 | 0.031001 | 0.034890 | 8.33 / 45.83 | 0.003409 / 0.003349 |
| 7320 非零阻力 | A_wrong_prior | 0.2375 | 0.056116 | 0.063276 | 183.33 / 175.00 | 0.014625 / 0.008093 |
| 7320 非零阻力 | B_structure_only | 0.1198 | 0.037965 | 0.042441 | 241.67 / 41.67 | 0.022619 / 0.011469 |
| 7320 非零阻力 | C_M2 | 0.0693 | 0.003407 | 0.002684 | 220.83 / 137.50 | 0.003543 / 0.003178 |
| 45621 原阻力 | A_wrong_prior | 0.3977 | 0.050265 | 0.031362 | 116.67 / 16.67 | 0.068755 / 0.035382 |
| 45621 原阻力 | B_structure_only | 0.3231 | 0.025286 | 0.015635 | 104.17 / 16.67 | 0.044962 / 0.026074 |
| 45621 原阻力 | C_M2 | 0.2971 | 0.035910 | 0.026108 | 116.67 / 16.67 | 0.046509 / 0.028000 |

三种条件的 M2 均通过未变的 validation RMS=10 gate，**M3 均为 NOT_TRIGGERED**；
不能将未运行的 M3 写成成功或失败。
新测试没有用于调整 whitening、候选结构、权重、参数或门槛。
也没有因为看见 45621 的新测试失败再启用 M3；那会违反测试隔离。

## 参数支持集与连续优化

| 条件 | 冻结代表点 tau_c / b | M2 支持候选数 | 可辨识性 |
|---|---|---:|---|
| 7320 原阻力 | 0.004 / 0 | 105 | UNIDENTIFIABLE |
| 7320 非零阻力 | 0.0043 / 0.3 | 105 | UNIDENTIFIABLE |
| 45621 原阻力 | 0.0108 / 0 | 65 | UNIDENTIFIABLE |

**每组支持集仍覆盖 tau_c=[0,0.012]、b=[0,1] 的完整冻结范围。**
M2 中 tau_s=tau_c，不是第三个独立参数；本轮没有 M3 支持集。
支持候选数包含原网格与连续评估的标签，不应当作独立重复实验数。
所有优化都遵守 2 起点 × 12 次函数评估预算，预算耗尽不代表全局收敛。
这些是 effective simulator parameter support，不能称为真实铰链摩擦值或精确置信区间。

## 验收结论

| 条件 | 结论 | 原因 |
|---|---|---|
| 7320 原阻力 | **BEHAVIORAL_ONLY** | 两个新动作优于 B，但旧动作回归失败 |
| 7320 非零阻力 | **PASS（条件行为预测）** | 两个新动作优于 B，validation 与旧动作回归通过；参数仍不可辨识 |
| 45621 | **MODEL_MISMATCH** | 虽通过包含结构不确定性的 validation gate，两个新动作均劣于 B |

**整体验收未通过，不将本轮代表模型提升为成功 baseline。**

7320 原阻力旧 P4 的 RMSE 从 79b100c / 3e5e856 的 0.011816 mm
增至 0.032215 mm，超过预冻结的 +0.010 mm 容限。
非零阻力旧 P4 为 0.001668 mm，优于 79b100c 的 0.005366 mm
及 3e5e856 的 0.010841 mm。
原成功基线及其结果均保留。

45621 的两个新动作分别比 B 恶化约 **42% / 67%**，因此不能宣布跨资产
physics calibration success。该结论没有通过测试后的参数重选改变。

## 剩余误差究竟是什么

1. **共享、无接触的启动/停止响应可以被最小 robot model 解释。**
   同一 Delta_t/T 在两个现有部署姿态独立验证通过，无需按资产拟合。
   这只排除了该范围内明显的共同延迟/单极点响应错误，不能排除所有受载执行器误差。
2. **45621 的主要残差是停驱动后的持续漂移，不是约 20 ms 的衰减瞬态。**
   验证 stop 段，C 的增量 RMSE 为 0.05648 mm；停后 250 ms 误差约 0.00592 mm，
   末端累积误差约 0.17510 mm；末尾两秒误差增长速度约 0.03695 mm/s。
   新速度测试 stop/dwell RMSE 为 0.04651 mm，也没有优于 B 的 0.04496 mm。
   图中可见停止后持续偏移，重启驱动又把部分位置偏差拉回。
3. **这仍不能唯一归因于 hinge physics。**
   结构传播尺度远大于确定性 repeat，且参数支持集没有收缩；
   结构引入的重力/装配响应、受载机器人响应和被动阻力仍可能混淆。
   当前模型选择在宽支持集内选出的代表点不具备稳定的跨动作优势。
   不再增加 spring、mass、Stribeck 或 contact damping 等参数。
4. **本轮停止在这里。**
   保留完整失败、曲线与支持集；共享响应校准可复用，但当前物理代表模型不替换旧成功结果。
   若开展下一轮，应先取得能区分这些解释的独立证据，而非继续加摩擦网格或用已见测试重选参数。

## 安全与复现范围

全部 308 次物体回放建立 bilateral hold，完成原生 physics protocol；
最小 joint margin **0.16923 rad**，高于原 0.05 rad 门槛。
记录的最大单指/把手接触力约 **0.59493 N**，未触发原安全限制。
最大受监测的接触面法向漂移约 **0.10427 mm**，它不是完整六维相对滑移测量。
切向滑移的不可观测性、simulator-only safety supervisor 和近似 interaction proxy 的限制继续保留。

所有回放均记录：只在开始初始化、无 robot/object state replay、无直接物体驱动、无 attachment。
原生 runner 的历史字段 `first_failure_state` 对正常条件回放也保存末帧；
本轮报告按真实 `failure_phase/status` 区分安全失败与这类末帧记录，不改执行代码。

源码对照 79b100c 仅增加独立文件；27 项已冻结源码 hash 校验通过，
312 次原生运行中的 baseline 源码 hash 一致。
本轮不能宣称完整接近/抓取/开门复现成功、真机摩擦辨识成功或四资产泛化成功。

## 数据与曲线

- [完整主表 CSV](benchmark_evidence/interactive_twin_robot_response_20261004/main_table.csv)
- [机器可读结果与判定](benchmark_evidence/interactive_twin_robot_response_20261004/delivery_summary.json)
- [M2 支持集](benchmark_evidence/interactive_twin_robot_response_20261004/physics_support_sets.json)
- [共享机器人校准曲线](benchmark_evidence/interactive_twin_robot_response_20261004/robot_response.png)
- [7320 原阻力：两个新动作](benchmark_evidence/interactive_twin_robot_response_20261004/heldout_dev_7320_00_original.png)
- [7320 非零阻力：两个新动作](benchmark_evidence/interactive_twin_robot_response_20261004/heldout_dev_7320_00_additional_nonzero.png)
- [45621：两个新动作](benchmark_evidence/interactive_twin_robot_response_20261004/heldout_test_45621_00_original.png)
- [45621 停止后的持续漂移](benchmark_evidence/interactive_twin_robot_response_20261004/stop_residual_test_45621_00_original.png)
- [有限候选参数 profile](benchmark_evidence/interactive_twin_robot_response_20261004/physics_support_profiles.png)

完整交付另存于本地 `results/interactive_twin_robot_response_deliverables/delivery`，
含所有新 held-out 的 observable trajectory；服务器保留每次原生日志和视频。
Git 中保存约 5.3 MB 的紧凑证据，大 JSON 使用无损 gzip；解压后 SHA256 见 evidence_index。
