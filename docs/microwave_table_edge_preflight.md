# PhysX 7320 微波炉：桌边安装预检

日期：2026-09-30。状态：**尚未通过安全开门预检，未执行微波炉抓取或开门。**

## 安装基准

世界桌面为 z=0，桌面中心 (0.50, 0)，尺寸 0.90×0.90 m，前边缘 y=-0.45。

- R1 底座安装面：(0.50, -0.55, -0.10 m)，yaw=150°。
- 底座中心在桌边外 10 cm，安装面低于桌面 10 cm。
- 参数化落地支架：0.10×0.10 m，底部 z=-0.76，顶部 z=-0.10。
- 微波炉：(0.50, -0.05 m)，yaw=-90°，垫高 0.04 m。
- 该位置是仿真诊断起点，并非已验证的真机推荐安装位置。实际支架外形仍需实测。

资产取自已下载的 PhysX-Mobility 7320，保留原 URDF 关节结构、轴和限位，统一缩放至 0.30 m 高。预处理记录源文件和几何 hash；在副本中修复无纹理坐标/无效法线索引，生成分件凸包碰撞与几何惯量近似。占位质量和惯量尚不是实测硬件参数。

## 已验证结果

实际 Isaac/MoveIt 待机 smoke test：TCP FK 误差约 1 μm，待机无碰撞，门角约 4.4e-8 rad。旧待机姿态在降低底座后会碰门，因此改用新的无碰撞待机姿态。

实际 RGB 经 SAM3.1 生成把手 mask；使用全图，避免默认 256 像素中心裁剪截断把手。SAM3 score=0.961，事后真值评估 IoU=0.992。真值没有进入抓取输入。checkpoint 的四个 missing keys 保留为兼容性风险。

| 阶段 | 数量 |
| --- | ---: |
| GraspGen-X 原始候选 | 100 |
| 目标/approach 筛选后 | 91 |
| 原始候选 Dex1 点云碰撞通过 | 3 |
| 现有局部 adaptation 的碰撞通过候选 | 73 / 1500 |
| 本轮进入 MoveIt 诊断 | 20 |
| exact IK | 7 |
| collision-free IK | 5 |
| pregrasp IK / approach | 5 / 5 |
| pregrasp plan | 4 |
| 原诊断完整开门路径 | 0 |

这些是候选诊断数量，不是执行成功率；此处原先的 margin funnel 主要衡量 J5/J6/J7。

## 运动门板碰撞检查与余量修正

MoveIt 静态场景若直接放入下一角度的门板，会将前一角度的机械臂起点误判为碰撞。现在先验证真实起点；仅在生成 OMPL 路径提案时临时允许运动门板碰撞，随后恢复完整碰撞规则，对机械臂和门板同步运动的密集样本逐一检查，并限制 TCP 圆弧偏差。其他静态物体和 self collision 保持检查。此过程不驱动 Isaac 门关节。

修正后 raw rank 32 能规划到 22°，J5/J6/J7 最小余量 0.06896 rad，**但全关节最小余量只有 0.01474 rad，受 J2 限制，因此不准执行**。

代码新增所有七个关节 >0.05 rad 的密集路径检查和执行入口检查；优化边界预留到 0.055 rad，以覆盖 MoveIt goal tolerance。旧报告中的 FULL_PATH_PLANNED 只代表当时较宽松的门槛，不能作为当前安全执行许可。

对 raw rank 32 做两组端点筛查：

- 9 个桌边位姿：x=0.50、y=-0.55，z=-0.15/-0.10/-0.05，yaw=140/150/160°，每个 4 个 seed。
- 6 个横向位姿：x=0.60/0.70、y=-0.55，z=-0.05/-0.10/-0.15，yaw=150°，每个 12 个 seed。

在全关节 0.055 rad 约束下，两组均没有找到该 raw 姿态的 exact 22° 终点解。这是有限搜索的结果，不证明桌边安装不可达，也不代表所有把手抓取姿态不可行。

随后实际重新启动 Isaac/MoveIt，以更新后的全关节门槛复查 raw rank 32：可规划前 21 个 1° 步长，22° 处 `NO_ARC_KINEMATIC_IK`。因此完整路径门禁仍未通过，未开始物理执行。

## 入口与下一步

服务器环境中运行：

```bash
source /data1/home/rangeryx/fr3_moveit_grasp/ros_env/setup.bash
python scripts/run_articulated_microwave.py --stage motion-smoke
python scripts/run_articulated_microwave.py --stage preflight
```

默认预设明确使用桌边低安装。需要先针对局部把手位置/绕轴姿态与不同 IK 分支，找到全关节余量合格、真实碰撞通过的完整 0→22° 路径，再测试双指接触与物理开门。当前没有成功操作视频；整体布局渲染仅为静态展示。

精简实际运行记录见 `microwave_table_edge_diagnostics.json`；服务器完整日志位于 `results/microwave_*`。所有模型原始输出保留，未修改 AnyGrasp 或 FR3Backend。
