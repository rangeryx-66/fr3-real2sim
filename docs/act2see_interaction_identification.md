# Act2See-style 最小交互辨识实验

## 结论与边界

2026-10-01 在 Isaac Sim 中完成 microwave 7320 的 revolute 探索实验，两次非零力运行得到相同结果，并完成零力对照。机器人 base、资产 geometry/joints/物理参数、摩擦、Dex1 夹紧力和既有抓取 pipeline 均未修改。

这是 **ideal attachment 的仿真 proof of concept，不能算 Dex1 真实抓取或完整开门成功**。质量为零的理想 TCP 固定在初始 moving-link 局部 frame；把小幅探索力传给该 rigid body，R1 以反馈跟随观测到的 link pose。它不是两个 articulation 之间的物理双向固定约束，实际机械臂存在跟踪误差。没有按 GT hinge 生成圆弧，没有设置物体关节位置完成运动。

## 实测结果

每次运行 6.5 s，记录 1560 个物理步。最大输入力 0.05 N，方向仅取初始 grasp approach 的反向，先拉、再推，控制器附加速度阻尼并限制力的模长。力作用点是当前理想 TCP；关节约束决定实际运动方向。关节余量门槛保持 0.05 rad，实测最小值 0.141951 rad。

| 指标 | 理想连接 TCP | 实际 R1 EE |
|---|---:|---:|
| 辨识类型 | revolute | revolute |
| 最大位移 | 21.331 mm | 21.234 mm |
| 观测转角跨度 | 2.918° | 2.897° |
| 估计半径 | 418.805 mm | 419.055 mm |
| 拟合位置 RMSE | 0.00833 mm | 0.16309 mm |
| GT 轴方向误差（符号无关） | 0.000145° | 0.011010° |
| GT 轴线距离误差 | 0.00543 mm | 0.28896 mm |
| GT 约束轨迹重建 RMSE | 0.00833 mm | 0.16321 mm |

实际 R1 EE 估计轴方向约 `(0.000126, -0.000145, -1.000000)`；轴上点为世界坐标 `(0.245888, -0.180290, 0.139492) m`。这里选择轴与初始 EE 轨道平面的交点作为 origin gauge，**沿轴方向的 URDF origin 不可由这段运动唯一辨识**。直接比较两个 origin 点有约 200 mm 的 gauge 差异，不能当作轴位置错误；轨道平面内 origin 误差约 0.29523 mm。

实际 EE 对理想 TCP 的最大位置跟踪误差为 0.70943 mm。模型得分间隔置信度为 0.999，只是归一化得分差，不是经过校准的成功概率。小角度、无视觉噪声的仿真结果不能替代真机精度验证。

零力对照保持同一场景和初始化，6.5 s 内 moving-link/理想 TCP 位移约 `2.8e-17 m`，输出 `UNOBSERVABLE`，置信度 0。非零力实验最初 1 s 的无输入段也没有门运动。该对照支持运动由外力激励造成，而非门自行偏转或脚本驱动门关节。

## 实现

- `scripts/run_act2see_probe.py`：独立入口、未知关节 point-force probe、理想 attachment、观测反馈跟随、记录和事后 GT evaluation。
- `interaction_identification/fitting.py`：仅接收 SE(3) 轨迹。比较恒定姿态直线模型与单轴旋转圆弧模型；输出类型、axis、轴上点、半径、残差和得分差。
- `scripts/plot_act2see_probe.py`：仅对已记录数据生成图，不向控制器提供轨迹。
- `scripts/test_act2see_fitting.py`：带噪小角度圆弧、直线与无运动辨识测试，均通过。
- `docs/act2see_probe_summary.json`：非零力与零力实验摘要。

复用已有 scene server 的初始化段，以唯一 marker 截断；不启动旧 HTTP server 或执行旧抓取/开门命令。启动段 SHA256 记录在 report 中。若该段结构改变，必须检查 adapter；不能把这个最小加载方式视为长期稳定的公共 API。

R1 使用当前观测 Jacobian 的阻尼反馈跟随，加入控制输入形式的重力补偿；drive gain 不变，不调用新的 IK 配置或优化 base。指爪保持打开，不尝试接触抓取。未获取 TCP measured wrench，记录字段为 null；不能把 applied wrench 当作测得的 wrench。

GT type/axis/origin 只在两份轨迹 fit 文件已保存后读取，用于 evaluation。仿真加载资产必然需要其 joint 定义，但 probe controller 和 fitter 不读取这些定义。初始化的机器人状态来自先前场景记录，未读取其已知 hinge trajectory。

## 复现（服务器）

```bash
cd /data1/home/rangeryx/fr3_real2sim_r1a7
env -u PYTHONPATH -u CUDA_VISIBLE_DEVICES \
  /data1/home/rangeryx/isaaclab-arena/.venv/bin/python -u \
  scripts/run_act2see_probe.py \
  --source results/microwave_7320_table_edge_preflight_full_rgb \
  --initial-state results/r1_contact_consistency/physical_local_161/report.json \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --output results/act2see_microwave_final --gpu 6

env -u PYTHONPATH \
  /data1/home/rangeryx/.conda/envs/anygrasp/bin/python \
  scripts/plot_act2see_probe.py results/act2see_microwave_final
```

零力对照使用同一命令，添加 `--force-n 0`，输出目录换为 `results/act2see_zero_force`。不要覆盖之前的结果目录。现有 GPU 初始化要求不设置 `CUDA_VISIBLE_DEVICES` 并使用 `--gpu 6`。入口默认在下一个上海时间 05:00 截止，也支持显式 `--deadline-shanghai 2026-10-01T05:00:00+08:00`；已过指定截止时间会拒绝启动。

输出 `observations.json` 包含每步两种 EE pose、机器人 q、moving-link pose、applied wrench、空 measured wrench、joint margin 和 tracking error；另有 `estimated_articulation.json`、`report.json`、`trajectory.png` 和 `probe.mp4`。视频为 1280×960、195 帧、6.5 s，明确标注理想连接。

## 剩余问题与下一步

本轮最低目标 revolute 已完成，无需跑到截止时间。现有已准备资产清单中的七个 active articulation 均为 revolute，未发现可直接复用的 prepared prismatic drawer；prismatic 目前只通过合成轨迹测试，不宣称通过物理实验。

当前没有导致 probe 或 fitting 失败的阻塞，但图中 applied force 存在速度阻尼反馈带来的高频波动，最大 EE 跟踪滞后约 0.71 mm；因此不能据此称力控制已经稳定。attachment 是仿真理想化接口，moving-link pose 来自 simulator observation。下一步先核对并稳定力/导纳控制的离散反馈，再加入观测噪声和不同通用 probe 方向，检验短圆弧可辨识性；随后验证物理双向 attachment / 力反馈版本，再考虑替换为真实 Dex1 contact。真实接触的非指垫碰撞和滑移问题仍未由本实验解决。
