# PiPER mobile wrist capture：2026-10-06 实际结果

**本轮未完成完整目标。** 已到冻结的上海时间 11:00 截止，不再启动物理实验。原成功抓取、接触、几何、摩擦、驱动力、结构 fitter 和安全阈值保留。所有失败与旧数据保留。

## 1. 实际结果

| 运行 | 实际 articulation motion（事后独立评估） | 腕部重建状态 | 恢复 / 停止原因 |
|---|---:|---|---|
| 7320，`run_v2_local_release` | 14.871° | 仅闭合状态，8 个已验证复用的真实腕部视角；没有合格打开状态 | 闭门释放→退让→实际底盘换位→新腕部观察→真实重抓→接触拉动已执行。开门后的释放被 RGB-D 注册误报中止 |
| 45746，`run_v2_drawer_checkpointed` | 约 0 m | 仅闭合状态，8 个视角 | 3 个实际重抓候选均未建立双侧保持；每次失败后已释放并退让，原预算用尽 |
| 7320，独立小角度观察验证 | 1.129° | **不计入 WRIST-CLEAN / 多状态分母** | q 自过滤的实际释放检查通过；后续回 home 触发 `gripper_link2 ↔ door` 几何检查停止。不是完整退让成功，也不是 15° 修复验证 |

7320 主运行的门由真实机器人接触传力运动；没有 attachment/weld、直接门外力或执行期门 joint-state 指令。使用的是此前 **EE-only 估计模型作为 operation memory**，不能计作本轮重新发现铰链。

### 原目标逐项核对

| 目标 | 当前证据 |
|---|---|
| 一个对象 mobile-assisted wrist scan | 既有闭门实际腕部采集及其 q/base/camera 记录已保存、验证并复用 |
| pull→release→reposition→reobserve→regrasp→continue | **未完成**。已证明的恢复周期从闭门释放开始；拉到约 15° 后的周期仍失败 |
| ≥3 不同状态，每状态 ≥8 clean views，且较大状态差 | **未达到**；两个对象仍各只有一个合格闭门状态 |
| 新 WRIST-CLEAN 的 ArtGS 10k/5k/20k | **未启动**，数据门槛不足。不能据此判断 ArtGS 本身的质量 |
| 新 mesh / segmentation / joint / URDF / independent Isaac import | **没有生成**；不使用参考资产或旧坏结果冒充新重建 twin |

## 2. 实际恢复数量

下表仅统计指定运行，区分调用次数和唯一底盘候选，不把 48 个配置中的视角池算成已执行视角。

| 对象 / 运行 | base+arm 评估 | mobile base 评估 / 唯一姿态 | view planning 请求 | retreat 候选累计 | 实际重抓尝试 |
|---|---:|---:|---:|---:|---:|
| 7320 / local release | 29 | 28 / 28 | 1 | 24（初始和开门后的预检累计） | 1，成功 |
| 45746 / checkpointed | 97 | 92 / 92 | 5 | 48（初始和失败清理累计） | 3，失败 |

覆盖视角池仍为有限的 48 个 proposal；不是无限 SE(3) 搜索。闭门照片来自以前真实 robot-reached wrist captures，复用前检查初始观察一致性，保存源文件/hash/实际 q 和相机位姿。**复用不是本次新拍，也不是新状态。**

7320 实际完成的初始换位终点：`(0.633977, -0.794716, -0.100 m; yaw=57.6587°)`，由通用 SE(2) search 得到，未写 asset-specific base heuristic。换位前已释放并收拢，期间接触载荷为零；停稳后才移动腕部相机和重抓。

## 3. 释放失败的证据与修改

主运行释放时，独立只写 GT logger 在 **episode 完全结束后** 得到：

- `SYSTEM_RELEASE` 530.167–531.167 s：物体平移/旋转变化没有可测值；EE 移动约 **0.503 mm**。
- 同期在线注册却报告 **1.679 mm**，超过保留的 1 mm guard。
- 开门阶段的 gripper–moving-part 相对平移变化约 **0.305 mm**，相对旋转变化约 **0.349°**（独立 GT 事后评价）。

因此这次停止不能归因为真实门回弹。原 unmasked local cloud 混入运动机器人表面，且全局短区域 ICP 的偏差不可直接作为物体运动。机器人点云影响是优先修正假设；没有保存原释放深度时，不能声称已经完成严格的 15° 同帧因果消融。

修改版本：

- `b612a12`：用当前稳态前后 RGB-D 直接注册，避免两次闭门参考注册误差相减。**实际 15° 重跑仍失败，不能宣称修完。**
- `8614f30`：只在 release observer 中，按 **实测 q + 官方机器人 envelope + 相机标定** 排除机器人自身投影。重建 RGB/depth/SAM mask 原样保留；没有 simulator instance-mask cleanup，没有几何缩放、contact/collision veto 绕过或阈值放宽。缺少足够观察点仍停止，不将缺失数据当安全。
- 后续 helper 修正：退让规划前将 `current_D` 更新为最新观测；原 helper 在独立小角度检查中使用旧场景变换，确属通用状态传递 bug。**该修正没有重新做物理退让验证，不能认为碰撞已消失。**

### 小角度实际验证的范围

独立入口执行了原固定底盘/名义抓取、真实闭合、小幅 estimated-model pull 和 release。

- 门事后实际转动 **1.129°**。
- 21 组 before/after 观察记录（含两相机原始 RGB-D、q、官方投影自过滤、保留点云）。
- 最大注册变化 **0.0878 mm**，原 **1 mm** guard 未变；接触清除后实际进入退让。
- 随后回 home 时被几何碰撞检查停止；原报告保留。11:00 的 SIGINT 请求不能覆盖这个真实终止原因。
- **不是 15° 条件验证，不是 multi-state capture，也不是完整任务成功。** 不将不同角度的 1.679 mm 与 0.0878 mm 直接当作公平的 before/after 改善比。

13 项非 FCL 回归、1 项 FCL 释放安全回归通过。真实 2 mm 观察位移仍触发重闭合；新自过滤保留位于机器人前方的物体深度。单元测试不替代上述实际执行或后续 15° 验证。

## 4. Effort 与指标边界

已保存 `effort_samples.jsonl`、`effort_segments.json` 及汇总。已有约 15° 运行的 106 个小段具有 PhysX **normal + friction tensor** 力测量；其投影切向力曲线已导出。不是两侧夹紧力之和，不拟合 tau_c/b/tau_s。

启动窗口的 `breakaway` 使用明确的 EE motion-onset 定义及事后 object-onset 评价。不是严格的最小摩擦证明。力矩关于 EE 估计轴计算，不能称真机精确 hinge torque。

视频上的 `tactile drift` 是原控制器代理量，不能作完整真实相对滑移证明。原 final-slip 字段的参考涉及 estimated `moving_initial`；本报告独立按阶段用 `inv(T_object) @ T_ee` 评价，不把不同抓持周期及估计参考误差混成一次真实滑移。

## 5. 可复查命令与数据

服务器：`/data1/home/rangeryx/fr3_real2sim_piper_mobile`。

本轮实际完整入口（历史运行命令，当前冻结截止已过；再次物理执行需要新预算/config/output，不覆盖此运行）：

```bash
PYTHONNOUSERSITE=1 environments/artgs/bin/python scripts/run_wrist_reconstruction.py \
  --config configs/wrist_reconstruction_v2_observed_memory.json \
  --output results/wrist_mobile_20261006/run_v2_local_release \
  --object 7320 --stage full --gpu 4
```

独立释放观察验证入口：

```bash
/data1/home/rangeryx/isaaclab-arena/.venv/bin/python scripts/run_wrist_release_validation.py \
  --job results/wrist_mobile_20261006/release_observer_validation/job.json
```

这些旧 job 的截止已过，不能用重置 ledger/原物体 joint-state 伪造续跑。下轮需按原规则通过真实动作恢复可继续状态，并保留累计预算与 provenance。

主要结果目录：

- `results/wrist_mobile_20261006/run_v2_local_release/capture_7320/`
- `results/wrist_mobile_20261006/run_v2_drawer_checkpointed/capture_45746/`
- `results/wrist_mobile_20261006/release_observer_validation/`

连续视频均为真实物理执行。原生参考资产视频、诊断图及近景画面不计作新 reconstructed twin 或 clean wrist input。旧失败目录保留。

## 6. 下一项必要工作（尚未执行）

先在新的有限预算中验证最新 release self-observation + current-scene retreat，确认 15° 松手后可安全退让；再补上开门后 scan/reobserve/regrasp/continue，获得第三状态及较大跨度。仅数据门槛真正满足后启动新 ArtGS，并按 coarse review → type → full → meshes/URDF/Isaac import 验收。

本轮没有重启 physics fitting、EE-only 精确轴线优化、ownership 分割或碰撞微审计。

腕部相机当前使用 D435 **simulation nominal** 参数。AgileX 官方示例支持 D435 或 Petrel，不是唯一 factory-default wrist camera；没有厂商通用 hand-eye 标定。不能把当前结果宣称为真机标定/部署完成。
