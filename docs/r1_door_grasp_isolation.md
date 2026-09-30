# 固定基座：门、理想抓取与真实夹持拆分诊断

2026-09-30。固定 base=(0.50,-0.55,-0.10 m; yaw=150°)，同一 raw rank 32 把手抓取参考。没有改变机器人/门尺寸、碰撞网格、摩擦、夹紧力或基座位置。本轮不优化最终开门任务。

## 1. 无机械臂的门本体测试

独立 Isaac 场景不加载机器人，保留原柜体、门、桌面、支架和垫块。通过关节 effort API 每个物理步施加已知力矩，回读实际 actuation torque；只在独立试次初始化时重置门角，不用脚本设置门角完成测试。

- URDF 对应的世界 hinge axis=[0,0,-1]；URDF/Isaac 最大位姿误差约 0.25 μm、0.000424 rad。
- 正向力矩打开门；负向 -0.1 N·m 保持在约 0° 下限；2 N·m 持续 3 s 达到 90.00001°，上限基本一致。
- 原关节 stiffness/damping/friction 均为 0，source limits 为 0–90°。这是当前模型定义，不是实测硬件参数。源质量和几何箱体惯量近似未修改，也尚未得到干净的动力学辨识结果。
- 0.75 N·m 下最大约 0.137°；1 N·m 下约 42.05°。用 0.413 m 的把手力臂换算，一秒内明显打开的观测阈值位于约 1.82–2.42 N 切向力之间；这是有限阶梯测试，不能当作精确静摩擦阈值。
- **异常：开始施加打开力矩后，门—cabinet_fixture 碰撞力峰值约 133–266 N。柜体和桌面对应过滤读数为 0。** 零力矩单独试次接触几乎为零。必须区分运动中垫块接触阻力与纯铰链摩擦。

结论：关节方向和限位基本正确，但门本体 sanity check **没有通过“环境碰撞无异常”**。当前开启力包含垫块干涉，无法据此标定真实 hinge friction/damping/inertia。

## 2. 理想 TCP→handle 关系下的连续运动学

数学上冻结 moving-link→TCP 变换，不创建伪造物理固定抓取。每 0.5° 以上一步解为 seed，求 exact 6D IK，并对关节插值逐样本检查 MoveIt 的机器人 self/table/support/cabinet/door 碰撞和全部关节 >0.05 rad 余量；没有临时放开门板碰撞来调用 OMPL。本测试使用 0.055 rad 优化边界，保留原有安全余量预留。

- 这条连续分支验证至 **21.5°**；22° 为 `NO_SAFE_CONTINUOUS_IK`。
- 22° 最近的优化试次主要受 **J2、J5** 约束，均到优化边界 0.055 rad；J6 仍约 0.483 rad、J7 约 1.340 rad。
- 失败附近加权 Jacobian 最小奇异值约 0.036，未见明显秩亏；求解中采用 ±0.25 rad 连续 trust region。没有证据支持“J7 卡限位”或“第一次拉动因 22° 规划失败”这类解释。
- 21.5° 是**该分支已验证的机器人运动学范围**，不是对所有 grasp/IK 分支的全局最优证明，也不是完整系统的物理无碰撞认证。门—垫块干涉由测试 1 单独证实，尚未排除，因此 `fully_collision_free_system_angle_deg=null`。

## 3. 锁门后的独立夹持载荷测试

到达同一把手 grasp 位置，全部关节起始余量约 0.232 rad，闭合后实际余量仍约 0.231 rad，排除近限位姿态。通过临时设置门 DOF limits 锁门，不改变摩擦、质量、几何或 gripper command。门角基本保持 0°。

- 闭合后的双指接触力约 7.33 / 11.11 N，零位移保持中双侧接触稳定。
- 零位移接触向量投影得到切向预载约 **2.34 N**。它不是最大承载力，也不能与开启阈值直接比较后宣称可开门。
- 尝试第一个 0.25 mm 切向加载步时，exact IK 可解、关节余量约 0.191 rad，但各试次都报告 **cabinet_door ↔ dex1_Link1_2** 碰撞；这是当前允许接触的 distal pads 之外的 finger body。
- 首轮 Cartesian 接口 fraction=0；随后使用 exact IK 和逐点状态检查复核，确认实际 blocker 为非指垫碰撞，非关节限位。没有关闭此碰撞约束来继续拉伸。
- 因为没有执行任何合规的非零加载步，**`max_stable_tangential_force=null`（未测得）**，不能把预载改名为最大承载力。
- 原始实验文件旧版本曾把零位移预载写进同名 max 字段；公开汇总已更正为 null，原始传感器数据保留审计。代码也已修正，只有成功的非零载荷步才能更新该字段。

## 诊断与优先级

当前既有抓取/接触几何问题，也有后段机器人运动学限制，还发现环境碰撞建模问题。0→1° 的失稳发生在余量充足的起始范围，不能归因于 22° 的关节限位。

应先审计门—垫块的几何/碰撞装配，以及 Dex1 指垫与 finger body 的真实接触位置；重新得到合规的锁门载荷曲线后，再处理固定 base 下后段 J2/J5 的冗余与抓取姿态可达性。不要先优化基座，也不要先增加摩擦或夹紧力。本轮未实施这些后续修正。

精简结果：`r1_door_grasp_isolation_summary.json`。服务器完整数据：`results/r1_door_grasp_isolation3/{isolation.json,door_no_robot.json,isolated_grasp.mp4}`。

复现入口（相同服务器环境、同一冻结 RGB/SAM3/GraspGen-X 输入）：

```bash
python scripts/preflight_articulated_saved.py \
  --source results/microwave_7320_table_edge_preflight_full_rgb \
  --output results/r1_door_grasp_isolation \
  --asset-root /data1/home/rangeryx/datasets/physx_mobility/prepared/7320_v1 \
  --variants-file results/microwave_raw32_probe.json --max-candidates 1 \
  --base-pose .50 -.55 -.10 150 --support-bottom-z -.76 \
  --isolation-diagnostics --record-overview --require-home-valid
```

不加载机器人单独测门：用同一安装参数启动 sim server，指定 `--door-sanity-only output.json`。这些 diagnostic RPC 默认关闭，正常 benchmark 不受影响。
