# Mobile recovery / physics decomposition 实测记录

实验：`results/interactive_twin_recovery_20261003_v2`。模式为
**SIM_TO_SIM_BLIND_SYSID**。这份记录不代表真机实验。

## 冻结 12 场景：mobile 部分已完成

| 指标 | Fixed base | 含 mobile recovery |
|---|---:|---:|
| IK / approach / 路径可行 | 1/12 | 6/12 |
| 真实 bilateral grasp | 1/12 | 4/12 |
| articulation identification | 1/12 | 3/12 |
| 实际开门增量 ≥5° | 1/12 | 3/12 |

新增恢复了 5 个规划可行场景、2 个真实开门成功场景。
全部成功仍集中于 **45621 的三个配置，涉及 1/4 个资产**。
不能将配置覆盖率改善写成已证明跨物体稳定操作。

| 成功配置 | 实际开门增量 | min joint margin | axis angular error | axis-line distance | final relative slip |
|---|---:|---:|---:|---:|---:|
| 45621_00 fixed | 5.585° | 0.052935 rad | 1.467° | 15.312 mm | 0.501 mm |
| 45621_01 mobile | 5.518° | 0.576946 rad | 1.896° | 1.027 mm | 0.185 mm |
| 45621_02 mobile | 5.488° | 0.496481 rad | 1.387° | 1.955 mm | 0.206 mm |

角度为相对初始状态的增量。slip 是事后最终相对变换，不是全程最大值。
axis-line distance 不把轴向不可辨识的 origin gauge 当成误差。

### 未恢复场景

- **45130，3/3**：目标 TCP 高度距固定 base 至少 0.985708 m，
  URDF 链长度三角不等式给出的保守最大 reach 为 0.893242 m。
  SE(2) 搜索不能改变高度，因此无法补救。没有升高底座。
- **45166，3/3**：存在部分 exact IK，但冻结 grasp family 仍有手指与门板/把手
  的几何冲突，部分 base 的 home 也不可用。没有放宽碰撞规则。
- **7138_00**：静态路径恢复，但真实双侧抓持未建立。实际门在抓取前已偏转约 81°；
  route 的机器人 loaded contact 为零。未做该初态的无操作对照，不能仅凭这一点
  断言唯一原因是重力，更不能把自行开门算操作成功。
- **7138_01**：实际 bilateral grasp 已建立，在 `COMPLIANT_SETTLE`
  触发 `PROBE_CARTESIAN_SPEED_LIMIT`，未进入可靠辨识。没有修改控制增益或速度上限。
- **7138_02**：首先发生 `BILATERAL_HOLD_NOT_ESTABLISHED`，之后消耗完配置预算。
  汇总保留首个物理失败与最终预算终止，避免用 timeout 掩盖抓持失败。

### 基础设施修复与重试

移动 callback 生命周期、等价 FCL shape 缓存和 NaN 汇总问题仅在新入口修复。
7138_00 做过一次原姿态、原控制、原参数的基础设施重验证；旧运行和额外运行
均保留 provenance。没有把重启当成新的搜索预算。

底盘是 collision-checked **kinematic SE(2) reposition → stop → manipulate**，
尚未验证轮式动力学或真实导航。抓持/辨识/滑移失败不会触发再次移动。

## Physics decomposition

P1 的 9 点冻结参数网格正在实际 Isaac 运行。其余组与 imperfect visual prior
须等待 P1 train / held-out gate；当前不能报告 physics 校准有效。

并发 visual import cache 初始化曾导致一个任务在开始物理测试前失败。
该失败已归档，缓存改为串行预热后用完全相同参数重试。
它不计作阻力不可辨识或真实接触失败。

LOW/MED/HIGH sensitivity 与 robot-only calibration 使用已有冻结数据，
带 import provenance，不称为本轮新执行。当前没有经过独立标定的 current/effort，
不报告 B3 或真实铰链 torque。

## 复现与证据

统一命令与阶段说明见 [运行文档](interactive_twin_recovery.md)。
`cross_object_mobile.csv` 包含初始/选择后的 base pose、行程、每阶段成败；
每个 episode 保存原生 report、视频和失败日志。
`scripts/summarize_interactive_twin_recovery.py` 从实际结果生成两张主表。

`f5dcc6f` / `cd61660` 成功 baseline 的抓取、接触、碰撞、proxy、friction、
force limit 和 joint margin >0.05 rad 保持冻结。
当前安全 supervisor 仍使用 PhysX 信号，未证明无 F/T 或 tactile 时完整在线 slip observability。
