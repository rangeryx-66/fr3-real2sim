# Production Estimator V3.1 — consistent SG121 dynamics

**交付：可供下一次独立物理验证的正式异步估计器。** 只使用已有 V3 数据；没有启动 Isaac、机器人运动或新交互，没有修改物理控制、Agent 策略、抓取、传感器装配或动力学。`b75d5b8` 和 `fe184ce` 保留。

## V3 / V3.1 结果

完全相同的三段 FIT、304 个 bin；完整 VALIDATION 段独立保留，83 个 bin。窗口固定为 121，不按 authored 参数或验证误差选择。

| 指标 | 冻结 V3 | Production V3.1 |
|---|---:|---:|
| I_eff kg·m² | 0.165298434 | **0.112126643** |
| b N·m·s/rad | 0.302644563 | **0.288832015** |
| C N·m | 0.017725733 | **0.017949420** |
| FIT 力矩 RMSE | 3.05894 mN·m | 0.08606 mN·m |
| 独立 VALIDATION 力矩 RMSE | 1.16059 mN·m | **0.06717 mN·m** |
| 独立前向角 RMSE，8.1667 s 内部区间 | 0.057855° | **0.039691°** |
| 同区间末角误差 | +0.126657° | +0.087905° |
| V3.1 原生速度预测 RMSE | — | 0.0005017 rad/s |
| 完整 9.175 s，含边界外推的角 RMSE | 0.122532°¹ | 0.076177° |
| 归一化回归条件数 | 25.56 | 26.95 |

¹ V3 完整段结果沿用 `fe184ce` 的独立离线重放。原 V3 的 0.057855° 指标原样保留。V3 速度 RMSE 使用 SG 速度参考，不能直接与 V3.1 的 native 速度参考比较。

力矩误差处于各版本自己的处理空间，不能把滤波后的 RMSE 降低解释为同等幅度的传感器精度提升。前向预测采用各版本的腕部 F/T 处理结果、一次相同 native 初始状态，不做后续状态校正或 GT 轨迹跟随。V3.1 另用未平滑、但已正确工具补偿的力矩输入，内部角 RMSE 为 **0.042617°**。完整段指标包含不可靠的导数/滤波边界，仅作敏感性检查。

## 实际接入方式

- [xarm_ft_v31_estimator.py](../../../scripts/xarm_ft_v31_estimator.py)：新生产数值处理；不导入离线动量积分或隐藏物体参数。
- [update_probing_v31_model.py](../../../scripts/update_probing_v31_model.py)：缓存列式读取、分段处理、FIT-only 回归/HAC、独立预测、原子模型发布及紧凑更新历史。
- [estimator_v31.py](../../../interaction_runtime/estimator_v31.py)：`OperationsV31` **仅覆盖 `_update`**。观察、规划、抓取、开关门、恢复及停车全部继承原操作。继续使用原单线程异步队列、30 s 导出等待和 60 s 子进程预算。
- [run_informative_probing_v31.py](../../../scripts/run_informative_probing_v31.py)：正式 V3.1 Agent runtime 入口，明确绑定 `RuntimeV3 + serve_v31 + OperationsV31`，冻结新 worker 和数值模块的源码哈希。

**旧 `run/launch_informative_probing_v3.py` 有意保留为 V3 复现入口，不会被静默改写成 V3.1。** 下一次经授权的独立物理测试应使用新 `run_informative_probing_v31.py --job <fresh job.json>`；本轮没有执行它。

## 数学约定

1. 在每条实际测量时间戳，旋转 `raw−bias` 到世界，计算工具重力及 Newton–Euler 惯性项；保留 COM 力臂、世界惯量、角加速度和陀螺项。没有调整工具质量、惯量、力矩增益或补偿倍数。
2. **先**将所有力矩移至固定世界铰链原点。不能先平滑移动传感器原点的六维数据，再乘当前力臂。
3. 工具 COM 加速度用中心二阶差分；角速度用居中的旋转增量，角加速度用中心差分。先形成完整瞬时工具惯性力矩，再与测得力矩共同处理：`tau = K(raw_world_hinge + gravity_hinge − inertia_hinge)`，工具补偿恰好一次。
4. K 是 SG121/order3 二阶导对应的、单位 DC 增益的**等效加速度平滑核**，不是再套一个普通 SG0。回归项均为 `K(d native_qd/dt)`、`K(native_qd)`、**`K(sign(native_qd))`**，不是 `sign(K(native_qd))`。
5. 只在单个完成段的连续有效记录内处理；缺测、采样间隙分块，不插值跨越；没有数据从 VALIDATION 进入 FIT。每端沿用原 121 点剔除，保持 304/83 个 bin。单 pad 无正法向载荷报告不作为新条件。

`fe184ce` 的 0.112231 是诊断近似，使用公共 211 点边界且保留了原工具旋转级联。生产版本将所有瞬时 Newton–Euler 项统一后处理，并保留原 FIT 边界，因此不要求两者逐位相同。没有按参考值校正这个差异。

## 验证证据

[VALIDATION.json](VALIDATION.json) 保存以下实际运行结果；[verify_estimator_v31.py](../../../scripts/verify_estimator_v31.py) 可重复执行，无 Isaac actor。

| 检查 | 结果 |
|---|---|
| 已知参数合成恢复 | 非平凡传感器旋转、运动 COM、工具重力/惯性、正反向速度，已知 I/b/C=`0.3/0.12/0.04`。工具质量 0 / 0.75 kg、61/121/181 窗全部通过；最大相对误差 I≈0.0033%、b≈0.117%、C≈0.0094%。 |
| 原 V3 精确复现 | 在只读 symlink replay 上调用冻结 worker；参数与原结果误差 <1e−12，原角 RMSE 一致。 |
| 独立验证段不泄漏 | 仅在内存大幅修改 VALIDATION 的 F/T、q、qd、工具位姿；FIT 参数、协方差、信息矩阵及 FIT 留一段结果**逐位一致**。 |
| 窗口稳定性 | 固定 61/121/181：I=`0.112045 / 0.112127 / 0.112198`；b=`0.291510 / 0.288832 / 0.287101`；C=`0.017911 / 0.017949 / 0.017975`。默认始终为 121。 |
| 独立动量积分核查 | 0.5 / 1 s 得到 I=`0.111237 / 0.111322`。只在验证脚本调用，不被生产 worker 导入、混合或用作校正。 |
| 分段/采样边界 | 混合 FIT/VALIDATION 和跨时间间隙的直接处理请求被拒绝；worker 自行分块，不跨边界平滑。 |
| 低激励报告 | 常速合成数据标记秩不足，三个参数不被错误标为独立支持。 |
| 正式异步调用 | 直接调用继承的 `update_async`，执行真正的 V3.1 `_update` 和 Python 子进程；两次更新串行完成，均 rc=0，报告正确版本及实际加载 SHA256。 |
| 入口接线 | 验证新 runner→serve factory→OperationsV31；物理操作方法与冻结类是相同函数对象。没有构造或启动 Isaac runtime。 |

## 不确定度、时延与开销

V3.1 条件标准差：I **0.0002531 kg·m²**，b **0.0008123 N·m·s/rad**，C **0.00001130 N·m**。使用仅 FIT 的 Bartlett HAC（SG121 时 lag=5），避免跨段/间隙的相关乘积，并采用 iid 对角下限。报告的协方差是 PSD，**其对角线与标准差平方一致**；修复了旧 worker 中二者可能不一致的报告约定。

三个参数当前均通过 rank/三标准差的**条件数值支持**。这不是已校准的物理置信区间：窗口敏感性、约 0.0008–0.0009 kg·m² 的独立方法差异以及工具/接触/求解器偏差不包含在 HAC 内，`physical_accuracy_validated=false`。约 5.35% 的验证内部采样角度超出实际 FIT bin 范围，已明确记录；配置的 0–20° 域不再被误标为“已观察覆盖”。

| 时间项目 | 实测/含义 |
|---|---|
| SG121 未来数据支持 | 最大 **62 点 / 0.2583 s** 的保守支持上界；每端仍剔除 121 点。不是零时延因果状态估计。 |
| 发布时机 | 完成段导出后异步更新；实际可用时间还要加子进程耗时，不能宣称 0.258 s 即返回完整模型。 |
| 本次同机 V3 同步完整重放 | 9.286 s，含绘图；原历史记录 worker 内部计时为 8.276 s。 |
| V3.1 正式异步子进程 | **4.375 / 4.475 s**，含启动、读取、计算、输出和绘图，均 <60 s。 |
| V3.1 worker 内部计时 | 3.619 s；其余为启动/进程开销。 |
| 全套验证 | 35.83 s，包括合成、V3 重放、两次异步更新、泄漏、窗口和动量核查。不是物理交互运行时间。 |

这些是一次同机数值回放的时间，不是长期吞吐/硬件性能标定。生产 worker 只发布当前模型、紧凑参数历史及一张覆盖更新的反馈图，不输出重复原始数据。

事后才读取既有公开 authored 参考：I/b/C 误差分别约 **−0.0472% / +4.281% / −0.729%**，见 [POSTHOC.json](POSTHOC.json)。尤其阻尼残余远大于条件标准差，不能据此宣称物理 twin 或传感器标定完成；authored 惯性也不等于独立测得的 PhysX 有效惯性。

## 复现、保存与停止

```bash
# 纯数值核查，无 Isaac
PYTHONPATH=.:scripts python scripts/verify_estimator_v31.py

# 完整 archived replay，NEW_DIR 必须独立于冻结结果
PYTHONPATH=.:scripts python scripts/verify_estimator_v31.py --episode PRIVATE_V3_EPISODE --output NEW_DIR

# 生产 worker 独立处理旧记录时也必须指定新输出目录
PYTHONPATH=.:scripts python scripts/update_probing_v31_model.py PRIVATE_V3_EPISODE --output NEW_MODEL_DIR
```

原始 7 个 NPZ 块、index、旧 estimate/updates 哈希保持；尝试覆盖 legacy model 会返回明确错误。物理代码、传感器及旧估计器与 `b75d5b8` 一致，见 [INTEGRITY.json](INTEGRITY.json)。回放只有指向原数据的只读使用 symlink，未复制 raw。私有派生结果约 **0.30 MB**；公开只有源码、紧凑数值和本报告。没有新进程占用 GPU，没有待清理 Isaac actor。

**V3.1 已通过现有记录上的生产数值与异步接线验证；下一步是独立物理测试，不是本轮已完成的物理验证。本轮到此停止。**
