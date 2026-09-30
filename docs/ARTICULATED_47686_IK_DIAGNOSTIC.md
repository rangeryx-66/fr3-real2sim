# 47686 柜门 grasp / IK / MoveIt preflight

本报告只统计预检。没有完成物理开门，也没有把规划成功当作抓取成功。资产为服务器上准备好的 PhysX-Mobility 47686 URDF/USD；R1-7a + Dex1-1 与 GraspGen-X 原始输出保持原样。仿真真值实例 mask 只用于计算 SAM3 IoU，从不输入 GraspGen-X。

## 本轮修正

- 通过 ModelScope 获取 `facebook/sam3.1` 的 `sam3.1_multiplex.pt`，位于 `/data1/home/rangeryx/sam3_1_weights/`。官方 SAM3 代码在实际 RGB 上推理；固定相机中心 ROI 为 256 px，并排除触及 ROI 边界的截断 mask。ROI 不使用仿真真值。
- 从 Isaac/USD 输出带方向的柜体 OBB。MoveIt 中旋转门板进一步使用 47686 URDF `l_1` 的 7 个 convex collision STL，按 Isaac 实际 moving-link 位姿更新。URDF 推算的门 link 与 Isaac 实测 link 的平移误差约 `1.4e-6 m`。
- 每个 GraspGen-X raw candidate 使用把手目标点云的最近邻局部 PCA 确定锚点和长轴；围绕该 candidate 做把手方向滑动、绕把手轴旋转以及有限 depth/approach 扰动。原始候选及 score 不改。
- 单独记录无碰撞 IK、MoveIt state validity、J5/J6/J7 margin、pregrasp IK、approach、home→pregrasp 规划，以及按 2° 递进的 0→22° 门板碰撞场景规划。只有 `FULL_PATH_PLANNED` 才允许进入物理执行。
- 修正完整执行阶段的候选顺序：先检查 Dex1 局部候选及 raw 无碰撞候选，避免默认 `--max-candidates 80` 全部消耗在 raw top-80 上。安装位姿的 home state validity 也单独记录，碰撞即停止预检。
- 相机捕获前暂停 Isaac 物理，使 RGB-D、门板角度和 MoveIt planning scene 一致。保留开门前恢复物理后的角度漂移保护。门板初始角超过 3° 的规划不标记为完整 0→22° 预检通过。

## 已完成的诊断

| 场景 / 方法 | 原始候选 | Dex1 无碰撞局部候选 | MoveIt 检查 | 纯运动学 IK | 碰撞-free IK | 全关节最小余量 >0.05 | pregrasp IK | approach | home→pregrasp | 完整圆弧 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 旧默认安装 `(0.45, 0.05, -90°)`、AABB、旧局部策略 | 100 | 406 | 406 离线 IK | 0 | — | — | — | — | — | 0 |
| 侧向安装 `(0.275, 0.20, -120°)`、URDF door mesh、同一批保存的 40 个候选、旧 home | 100 | 140 | 40 | 14 | 14 | 14 | 11 | 11 | 0 | 0 |
| 侧向安装、真实 RGB→SAM3→GraspGen-X、URDF door mesh、安全 home、物理未冻结 | 100 | 108 | 50 | 18 | 11 | 11 | 4 | 1 | 1 | 0 |
| 同位姿、下一帧真实 RGB→SAM3→GraspGen-X、物理冻结但门已偏转 6.64° | 100 | 79，另有 raw 8 | 87 | 35 | 0 | 0 | 0 | 0 | 0 | 0 |
| 同位姿、真实 RGB→SAM3→GraspGen-X、闭门后冻结、安全 home | 100 | 60，另有 raw 5 | 50 | 18 | 18 | 17 | 10 | 4 | 4 | 0 |

旧 home 与门板发生 `Link5` 碰撞，使 11 个已能 approach 的候选无法从起点规划。使用附近的安全 home 后，最远候选可走到约 2° 开门段，在约 4° 的 Cartesian 段遇到机器人自碰撞。冻结物理后的 87 个候选中，35 个有纯运动学 IK，全部在 MoveIt 中碰撞（337 次 `CABINET_DOOR_COLLISION`，5 次机器人自碰撞的 seed 级计数）。真实 SAM3 mask 的评估 IoU 为 0.87，但模型候选的姿态分布与上一帧不同，不能把前一帧的 11/50 当稳定比率。该帧在暂停前门已自行偏转 6.64°；重新闭门初始化的预检另行记录。

闭门重置后冻结的关节角为 1.22°，SAM3 mask 评估 IoU 为 0.847。前 50 个（先 5 个 raw 无碰撞候选，再取局部候选）有 18 个 collision-free exact IK，其中 17 个余量 >0.05 rad，15 个 >0.08 rad；10 个通过 pregrasp IK，4 个通过 approach 与 home→pregrasp 规划。最深的 raw rank 82 及其小扰动版本均沿关节圆弧规划到约 15°，下一段 J7 余量只有约 0.03 rad；另一个候选在圆弧约 2° 发生机器人自碰撞。没有完整 0→22° 路径，故没有物理抓取或开门。

同一保存候选的 OBB/mesh 对照：前 40 个中，OBB 有 14 个纯运动学 IK，但仅 6 个通过碰撞及余量检查；URDF 门板 mesh 保留全部 14 个。两者均被旧 home 的门板碰撞挡住。Dex1 clearance 从 0、3、5 mm 变化时，115 个小样本候选分别保留 91、87、86 个，表明 clearance 不是 0 IK 的主因。代表性候选在 0.1/0.25/0.5 s、0/8/16 随机 seed 下可产生不同数量的 IK 解，但仍有本身不可达的姿态；不能把 0 IK 全部归因于 timeout。

## 安装搜索与当前限制

事先限定的局部安装格点先用 URDF 对 exact grasp、pregrasp 和相对采集角度的 22° 运动学圆弧作筛选；只有真实 Isaac/MoveIt 预检才检查闭门约 0→22°。格点筛选不等于碰撞或物理成功。`(0.275, 0.20, -120°)` 在 27 个邻近格点中有 2 个完整**运动学**圆弧候选，随后真实相机确认把手可见，并完成上述 MoveIt preflight，因此是当前最值得继续排查的安装位姿。柜体垫高 0.18 m；R1 底座保持既有 `(0.329, -0.175, 0.237 m; yaw=56.3°)`。对照的扩展格点中，部分位置有安全的运动学圆弧，但真实 Isaac 初始化时门板被机器人推开或 home 碰撞，不能只凭离线格点推荐。

例如离线筛选给 `(0.50, 0.275, -120°)` 一个安全运动学圆弧候选，但独立 Isaac/MoveIt 回放发现柜门启动角约 29.7°，把手在当前相机下的评估可见像素为 0；它不满足闭门任务的安装条件。该回放在所有候选处报告 `INITIAL_DOOR_DRIFT`，因此不能将其视作有效的 IK/规划统计。
另一个离线候选 `(0.35, 0.20, -150°)` 在真实 MoveIt 门板 mesh 下，安全 home 的 `Link4` 与柜门碰撞，预检在抓取前停止。
`(0.20, 0.20, -90°)` 的 home 无碰撞，评估把手可见像素为 2124，但启动时门已偏转约 15.1°，同样不能作为闭门 0→22° 的有效安装方案。
当前位姿附近仅将 x 从 0.275 m 移到 0.300 m 时，home 仍无碰撞、把手评估可见像素 1190，但柜门初始已偏转约 7.9°；这显示该资产当前物理安装并没有验证过宽容的稳定区域。当前最优只能称为**预检诊断位姿**，尚无完整可执行安装方案。

安全起始臂姿态（仅 47686 demo）：`[0.05778, 1.49302, 0.54423, -1.36361, 0.05033, 0.09009, -0.03614] rad`。这不是抓取姿态，也不改变既有刚体 benchmark。

仍需解决：门板在未抓取前会自行偏转，SAM3 在某些帧会把金属机器人部件当成把手，且目前没有任何经过完整碰撞和 MoveIt 规划的 0→22° 开门轨迹。无完整预检即不执行开门。

当前服务器的 SAM3 源码加载 3.1 checkpoint 时报告 4 个 `backbone.vision_backbone.convs.3` 权重缺失。模型仍能完成真实 RGB 推理，但此源码/权重组合需视作感知侧兼容性风险；不能用一帧 IoU 代替稳定性验证。

## 复现

服务器使用 `/data1/home/rangeryx/fr3_real2sim_r1a7`，先 `source /data1/home/rangeryx/fr3_moveit_grasp/ros_env/setup.bash`。预检示例：

```bash
python scripts/run_articulated_47686.py --stage preflight \
  --asset-x 0.275 --asset-y 0.20 --asset-yaw-deg -120 \
  --fixture-height-m 0.18 \
  --home-q 0.0577814919 1.4930180226 0.5442315114 -1.3636135227 0.0503292349 0.090090988 -0.0361356973 \
  --sam3-checkpoint /data1/home/rangeryx/sam3_1_weights/sam3.1_multiplex.pt \
  --max-candidates 100 --output results/articulated_47686_preflight
```

`scripts/preflight_articulated_saved.py` 可将保存的候选随柜体安装位姿以及实测门铰链角度刚体变换，再重新做 MoveIt 检查；其结果明确标注为 **saved-candidate diagnostic**，不代表新场景的 SAM3 或 GraspGen-X 重新推理。`scripts/scan_articulated_placement.py --arc` 是运动学格点预筛，不代表完整 MoveIt/Isaac 可行性。
