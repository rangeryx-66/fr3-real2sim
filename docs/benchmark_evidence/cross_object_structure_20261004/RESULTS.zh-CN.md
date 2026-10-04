# 跨物体 articulation structure benchmark：实测结论

本轮实际完成 8 个未用于开发的 PhysX-Mobility 资产、每个 2 个配置，共 16 场景。资产在执行前冻结；失败没有被替换。7320、45621 仅作回归对照，不计入主集。没有重新拟合物理参数，也没有修改原有抓取、接触、控制器或安全门槛。

## 主结果

| 指标 | 实测 |
|---|---:|
| 抓住并产生 ≥5 mm 实测 EE 运动 | 6/16，37.5% |
| 产生上述交互的资产 | 3/8，三个资产均为 2/2 配置 |
| 接受 refined articulation / 达到位移门槛的交互 | 0/6 |
| 通过 discovery 模型接受门槛 | 0 |
| 以已接受 discovery 为分母的 refinement 成功率 | N/A，分母为 0 |
| estimated/refined model 端到端成功 | 0/16 |
| 至少一个配置端到端成功的资产 | 0/8 |
| 两个配置均端到端成功的资产 | 0/8 |
| 固定底座可达 | 5/16 |
| Mobile recovery 恢复的固定底座失败场景 | 3/11，使总可达数达到 8/16 |

这里“≥5 mm 运动”只是统计门槛，不代表角度激励、噪声与拟合残差已经满足结构可观测性要求。不能将 provisional revolute 分类计作成功结构识别。

| 资产 | 类别 | 有效交互 | 首个失败层 |
|---|---|---:|---|
| 9128 | Door Set | 0/2 | 部署可达性 |
| 12480 | Dishwasher | 0/2 | 部署可达性 |
| 7130 | Microwave | 0/2 | 抓持 |
| 10797 | Refrigerator | 0/2 | 部署可达性 |
| 45600 | Cabinet | 2/2 | Discovery 未通过 |
| 38516 | Dressing Cabinet with Mirror | 2/2 | Discovery 未通过 |
| 45403 | Storage Cabinet | 2/2 | Discovery 未通过 |
| 48721 | Tall Storage Cabinet | 0/2 | 部署可达性 |

## 失败首先发生在哪里

- **部署：8/16。** 冻结 SE(2) 搜索未找到满足 IK、路径碰撞及余量条件的部署。没有据此宣称物体全局不可操作。Mobile recovery 成功恢复了 7130 的两个配置和 38516 的第二配置；只有后者进一步建立抓持并产生有效运动。
- **抓持：2/16。** 7130 是事后确认的水平轴翻门。预热阶段已有被动偏转，初始把手定位随后失配；实际失败分别是背部/根部加载和未建立双侧保持。报告中的约 3.97° / −4.86° 初末角度差包含被动运动，不能算机器人开门成功。CSV 分开保存预热漂移和预热之后的运动。
- **Discovery：6/16。** 三个新柜体均建立真实抓持，EE 移动约 20 mm；实际门只转约 0.84–1.68°。位置重构残差为 0.38–0.88 mm，超过冻结的 0.30 mm discovery 门槛；部分场景角度激励也不足。达到既有探索预算后停止，没有降低门槛。

因此主集没有进入 robust refinement，也没有进入 T0/T1/T2 留出预测。**当前证据尚未证明结构重建跨物体泛化，不能把失败归因于从未被调用的 robust 精化阶段。**

## 对照与留出预测

原 5.5° 回归对照：7320、45621 均成功，实测开门分别为 5.62°、5.58°。

补充完整结构对照不计入主集：

- **7320 成功**：实际开门 8.29°；轴方向误差 0.551° → 0.0243°，轴线误差 35.70 → 0.332 mm。T2 完整完成留出预测，EE 位置 RMSE 0.0181 mm。T1 在留出段之前触发既有安全停止，没有完整 RMSE。T0 为数据集结构，确定性重放误差为 0，接近 oracle，不能称为视觉先验。
- **45621 扩展阶段安全停止**：实际达到 5.77°，精化未完成。原日志名为 `SUSTAINED_RELATIVE_SLIP`，但触发量是约 1.007 mm 的模型一致性误差；接触平面漂移仅约 0.104 mm。该标签不能直接解释为真实夹爪滑脱。

GT 仅在执行停止、估计保存后用于误差和实际位移评估。在线成功/停止结果与事后任务成功分开保存。没有 attachment、直接物体驱动或执行中物体状态回放。

## 资产与证据

资产来自已有 `PhysX-Mobility.zip`，SHA256 为 `88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908`。视觉网格保留原数据集；接触使用冻结的近似 semantic interaction proxy，并不宣称真机几何精确一致。总览部分高物体被固定相机裁切，不能由该图误判其只有一个悬空把手。

- [自动报告](REPORT.md)、[逐场景](per_episode.csv)、[逐资产](per_asset.csv)、[失败分类](failure_breakdown.csv)
- [冻结资产清单](frozen_asset_manifest.json)、[信息边界与代码核对](input_and_information_boundary_audit.json)
- [资产总览](asset_overview.jpg)、[Discovery 可观测性](discovery_observability.png)、[DEV 留出预测](control_7320_heldout.png)
- `infrastructure_history/` 保留基础设施修复、旧清单及受影响重跑的 provenance。科学配置未改变。

完整服务器输出：`/data1/home/rangeryx/fr3_real2sim_piper_mobile/results/cross_object_structure_20261004_v1`。原始轨迹、命令和连续视频保留于此；Git 保存紧凑证据，省略大体积路由逐步日志等，见 `git_evidence_index.json`。本地完整证据及代表视频位于工作区 `results/cross_object_structure_delivery`。

统一运行方式见 [入口说明](../../cross_object_structure_benchmark.md)。重现实验需复制配置并使用新的输出目录和截止时间，避免覆盖本轮不可变结果。

**本轮结论：真实接触运动已覆盖三个新资产；完整跨物体结构识别尚未成立。首先受部署覆盖率限制，已抓持场景则受 discovery 的短角度激励和重构残差限制。本轮到此停止，没有继续修改参数或进入 physics fitting。**
