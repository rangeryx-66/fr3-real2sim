# PiPER 能力、信号和仿真边界

本审计读取官方源代码及本地安装信息，不连接 CAN、不实例化 SDK，也不改变成功的抓取/接触 baseline。官方字段存在、当前电脑安装 SDK、当前真机能够可靠提供该字段，是三个不同结论。当前未连接机器人，**硬件 firmware、serial、反馈频率和控制延迟均 UNKNOWN**。

## 已核对的官方版本

AgileX 官方 `piper_sdk`：[`c9e8a28174e71eeaac448593cb65f8ab258a92fe`](https://github.com/agilexrobotics/piper_sdk/tree/c9e8a28174e71eeaac448593cb65f8ab258a92fe)，源代码版本 0.6.2。该版本仅作为有固定 commit 的 API 证据，不冒充服务器或真机正在使用的版本。报告中的 `local_sdk` 单独记录实际探测结果、源码 SHA 和版本；不存在则保留缺失状态。

| 信号 | 来源及单位 | 必须保留的限制 |
|---|---|---|
| q | `GetArmJointMsgs`，0.001° → rad | 6 轴分在 3 个 CAN frame，不是严格同步快照；首帧默认零值不能直接使用 |
| qdot | 高速反馈 `motor_speed`，0.001 rad/s | 官方名称是 motor speed；必须与时间戳化 q 差分核对传动和正负方向 |
| EE/FK | `GetFK('feedback')` 为 mm/degree；EndPose feedback 为 0.001 mm/0.001° | 编码器/模型导出的 pose，不是独立笛卡尔位置传感；必须确认 DH/zero/TCP/base |
| current | 高速反馈 `current`，0.001 A | 总执行器响应含重力、惯性、servo、机器人摩擦，不能直接当外力矩 |
| SDK effort | current × 固定系数，J1–3: 1.18125；J4–6: 0.95844 | 不是独立 torque sensor；官方注释的 `0.001 N/m` 量纲写法含混，保存 raw，不自行宣称外部 N·m |
| gripper stroke | `grippers_angle`，0.001 mm | 需要零点和实际开口映射标定；不是两指独立关节观测 |
| gripper effort/status | 整体 effort 文档为 0.001 N·m，另有 enable/homing/error 状态 | 不能转换成左右 pad force，也不能独立证明双侧接触 |
| time | 审计版协议复制 python-can receive timestamp；应用另记录 monotonic send/receive | 不是机器人执行时刻；组消息时间戳随单帧更新，必须记录 frame age、同步误差和实际延迟 |

上述单位依据官方 [joint feedback](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_joint_states.py)、[motor feedback](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_high_spd.py)、[gripper feedback](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_gripper.py) 和 [interface](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/piper_sdk/interface/piper_interface.py)。

两个需要实际验证的版本细节：

- Current 字段文档写 uint16，但审计版本的 [parser](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/piper_sdk/protocol/protocol_v2/piper_protocol_v2.py) 按 signed int16 解码。保存原始 CAN bytes；未核对固件前不能根据电流符号推断受力方向。
- 官方 [FK](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/piper_sdk/kinematics/piper_fk.py) 含可切换的 2° DH offset；[README](https://github.com/agilexrobotics/piper_sdk/blob/c9e8a28174e71eeaac448593cb65f8ab258a92fe/README.MD) 说明与 S-V1.6-3 前后固件有关。不能仅因默认参数存在就认定本机模型一致。

## 模型辨识可使用什么

主数据接口只传时间、归一化 q/qdot、编码器/FK TCP SE(3)、gripper state 和真实下发 command。初始视觉 handle frame、attempt history、已估计 articulation 属于另行明确的策略输入。没有 six-axis wrist F/T、左右 tactile array 或精确外关节力矩的已验证能力。

`project_observation()` 从混合仿真日志显式白名单导出估计器输入，**不输出 GT joint、moving-link trajectory、PhysX contact manifold、法向、分离距离或仿真接触力**。命令字段也使用白名单，不能把 GT 藏进 metadata。源适配器负责单位转换；该函数不会猜单位。6 个 arm q 与 gripper 数据分开。

原成功 baseline 的双侧载荷、安全碰撞和接触面漂移仍来自 PhysX。可以原样保留为本轮 **simulation safety oracle**，但它们不属于 PiPER 硬件传感信号，不能给系统辨识器，不能据此宣称真机可运行。沿接触面的完整 slip 仍不可观测。若实验继续依赖这些检查，报告必须明确这一范围。

现有 Isaac 控制器使用机器人模型重力补偿和直接 joint effort。SDK 具有 MIT 风格命令 API，并不证明其时延、torque scale、servo 行为与 Isaac 相同；本轮没有连接/运行 MIT 控制。未来部署前必须独立完成无物体机器人响应校准，冻结后再辨识物体。

## B3 和物理参数命名

默认 **B3 = UNAVAILABLE**。增加 `calibrated: true` 字段不能解锁。标定证据至少记录机器人 serial/firmware/SDK、单位、偏置、同步验证、冻结的 robot-only baseline、held-out 验证及证据 SHA。即使证据齐全，本报告也只标 `EVIDENCE_DECLARED_REVIEW_REQUIRED`；不自动启用未复核的模型。

未获得可信 torque scale 时，只称 **effective simulator resistance parameters / identifiable interval**。不能将 SDK effort、仿真手指力或命令力直接称为真实 hinge friction N·m。current/effort 原始数据可独立记录用于以后标定；未标定时不进入加权 fitting objective。

## 独立运行与集成

```bash
python -m interactive_twin.capability --output results/interactive_twin/capability
# 可选：读取已存在的 SDK checkout；仍不会打开 CAN。
python -m interactive_twin.capability --sdk-root /path/to/piper_sdk --output results/interactive_twin/capability
python scripts/test_interactive_twin_capability.py
```

程序接口：

- `build_capability_report(sdk_root=None, calibration=None) -> dict`
- `write_capability_report(output_dir, sdk_root=None, calibration=None) -> dict`
- `project_observation(row, mode='SIM_TO_SIM_BLIND_SYSID', allow_calibrated_effort=False, report=None) -> dict`
- `validate_controller_inputs(signal_names, report=None)`：出现不在策略信号白名单内的字段即报错。

输出 `capability_report.json` 和 `capability_report.md`，包括固定官方证据、本地探测、未知项、单位、B3 状态和 claims boundary。`SIM_TO_SIM_BLIND_SYSID` 与 `REAL_LOG_TO_SIM` 共用归一化日志接口；没有真机日志时，前者不构成 Real2Sim 实验证明。
