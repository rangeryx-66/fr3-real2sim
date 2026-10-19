# 3af4cd3 → 实际 ArtGS 接通与分段采集结果

## 结论

**官方后端、真实数据适配、推断 mesh/URDF、独立 Isaac 导入已运行；重建质量没有通过实用验收。换位后续抓也未完成。**

本轮结束于 2026-10-06 凌晨，早于冻结的 05:00 截止。没有追加抓取搜索、碰撞重建或摩擦优化。失败保留，原参考资产没有被重建模型覆盖。

## 实际物理采集与续抓

| 数据集资产 | 实际最终操作量（事后评估） | 持握阶段最大 gripper-part 相对平移 | 最小关节余量 | 释放/退臂/换位 | 重抓与续拉 |
|---|---:|---:|---:|---|---|
| 7320，revolute | 18.6737° | 0.1301 mm | 0.06349 rad | 已实际完成 | 未完成 |
| 45746，prismatic | 60.1627 mm | 1.6589 mm | 0.09486 rad | 已实际完成 | 未完成 |

相对平移包含夹爪顺应，不等同于纯滑移；释放和底盘移动阶段不计入持握漂移。底盘使用既有 kinematic SE(2) 平台，非轮式导航动力学。

两者均在 `OBJECT_MOVED_DURING_REPOSITION` 停止：在线重新观测/配准无法满足已冻结的 1 mm 一致性检查。**独立事后日志显示释放结束到换位结束，物体平移和转动均为 0。** 此次阻塞是重新观测的不一致，不能宣称物体回弹，也不能宣称安全续抓成功。未降低检查门槛或通过 GT 位姿补重抓。

本轮修复了新编排的模式切换问题：切换位置控制时清除上一阶段未清零的 actuator effort；位置 gains、夹紧力上限、抓取控制器和物理参数不变。此前约 1.02 mm 释放开口偏差与残留约 0.524 N 力命令一致。所有早期失败、原生 Isaac 崩溃及有预算的重跑均保留。

## 实际重建

官方 ArtGS：`7c1f41be2cb8b96abca13c6a9668dcf7e06d8c2a`。
独立环境；官方 storage_45503 checkpoint 只跑作者自己的 demo。
目标数据实际运行 **3000 coarse + 3000 type prediction + 5000 joint iterations**，seed 61；没有用作者示例权重、数据集 GT mesh 或 GT axis 替代目标输出。此为有限预算的逐场景优化，不是通用预训练直接推理，也不是官方完整训练预算评测。

复用已有实际采集数据，增加的 orbit views 在物体保持同一状态时暂停物理渲染。
重建主输入来源：7320_recovery_v2 和 45746_recovery_v3_infrastructure_retry；最新 v4 续抓验证数据及失败报告独立保留，没有混写历史训练输入。

| 模型 | 小状态对，留出 RGB RMSE / IoU | 大状态对，留出 RGB RMSE / IoU | 推断/导出评价 |
|---|---|---|---|
| 7320，原生自动类型 | 0.20234 / 0.69632 | 0.21052 / 0.74842 | 错判为 prismatic，mesh 大面积畸变 |
| 7320，EE 识别的 revolute 类型先验 | 0.20935 / 0.70733 | 0.21190 / 0.74449 | 轴/部件仍由 ArtGS 学习；mesh 畸变，没有获得可靠模型 |
| 45746，原生自动类型 | 0.10644 / 0.81750 | 0.10552 / 0.81136 | 类型为 prismatic，外壳较完整，但前部/活动部件明显缺失 |

留出 state 1 的图像/点云没有参加双状态重建。其 EE-estimated 状态标签仅提供渲染相位；这是**条件几何预测，不是自主演化/物理预测**。RGB/IoU 来自 Gaussian 表示，不能冒充导出 mesh 的碰撞精度。大开幅没有带来一致的预测改善。新增多视角版本并不全面优于旧两视角数据。

已导出实际学习的两个 part meshes、米制 URDF、父坐标轴/原点、世界 root 变换、来源、不确定性和 observed range。三个代表模型均在独立 Isaac 场景实际导入并录制开合视频；该视频通过驱动**重建模型**的关节检查装配，不是机器人真实接触成功。质量差的模型不应替换工作参考 twin。质量/关节分类失败没有用 GT 修补。

输入核对：光学 Z 深度；米制统一归一化；ROS-optical → OpenGL；共同内参重采样；每个状态独立点云；保留共同 camera IDs，并留出两个物理视角。自身点云重投影 mask 命中率最低 99.48%；320 像素重采样后的最大视角中位深度误差约 2.00 mm，非真实标定精度声明。机器人遮挡由 simulator instance mask 排除，非真机分割能力声明。

## effort

保存 `effort_samples.jsonl`、`effort_segments.json`、`effort_profile.json`、独立事后 `effort_profile_object_evaluation.json` 和时间曲线。

经验证的原生 normal+friction contact buffers 提供把手上的净力；世界坐标，N，沿日志拉动方向投影。不是两指夹紧力之和。估计轴可信时仅保留既有轴参考 torque 附件，未映射为精确 friction。

启动窗口为运动超过 0.25 mm 周围 ±0.2 s；低速窗口为开始后 ≥0.5 s、0.05–0.75 mm/s，输出平均、波动、范围和实际速度。启动阻力包括接触瞬态/密封/重力等；**不声称精确最小 breakaway friction**。启动窗口的力波动较大。没有可靠测量的样本保留 command proxy；独立物体运动标记只在结束后用于区分 EE 顺应与真实物体启动，不反馈控制。

## 一个统一入口

服务器已安装环境：

```bash
cd /data1/home/rangeryx/fr3_real2sim_piper_mobile
PATH=/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:$PATH \
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  environments/artgs/bin/python scripts/run_articulated_system.py \
  --config configs/articulated_system.json --stage full --gpu 1
```

采集与后端分别保存状态。本次按该入口的阶段和独立 episode job 实际执行，不声称单次 full 命令已完成所有目标。
支持 `collect/prepare/reconstruct/twin/preview/validate/report`；已有数据优先重建。截止时间过后，要开展新实验必须明确更新日期配置和新输出目录，不能静默延长旧 run。

## 交付

本地 `results/articulated_system_delivery/`：

- `artgs_actual_loader_inputs.tar.gz`：官方实际 loader 可读的四个双状态输入。
- `artgs_inferred_twins_and_observations.tar.gz`：三个实际推断模型、米制 URDF、多状态 RGB-D/相机数据、effort 和独立导入视频。整包解压，内部 hard links 自包含，无服务器 symlink 依赖。
- `artgs_target_checkpoints.tar.gz`：真实目标场景 5000-step Gaussian/deformation checkpoints。
- `microwave_release_reposition_blocked.mp4` / `drawer_release_reposition_blocked.mp4`：连续实际接近、抓持、分段拉动、释放、换位及停止视频。
- `microwave_artgs_EE_type_prior_import.mp4` / `drawer_artgs_native_import.mp4`：独立重建模型导入检查。
- `reconstruction_value_check.png`、两个 effort 时间图。

本目录保留主 CSV、checkpoint 来源/校验值、官方权重来源、baseline 哈希及最终报告。服务器完整原始日志在 `results/articulated_system_20261005/`；没有删除早期失败。

## 尚未完成及首个阻塞

1. 20°/8 cm 连续任务与换位后重抓：当前重新观测/配准阻塞。尚未证明完整换位续抓。
2. 可用 articulated reconstruction：当前部件分离、推断关节及导出 mesh 质量不足。没有证明新增状态观察改善重建，也没有生成可信的替代工作 twin。
3. 精确最低开启阻力：已保存可核验的 measured/proxy 记录，但瞬态与波动不支持精确最小摩擦结论。

本轮停止继续试参。后续应分别处理重抓定位与重建部件/可见性问题，不回到原抓取、碰撞、friction 或 EE-only 精确轴线调参。
