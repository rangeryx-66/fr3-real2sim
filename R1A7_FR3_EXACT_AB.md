# FR3 原 benchmark 输入下的 R1-7a + Dex1 严格对照

## 输入与运行条件

本实验只使用 FR3 原固定桌面盒子场景（45 × 45 × 50 mm，质量 60 g，中心 `(0.5, 0, 0.025 m)`）、原相机 `(0.5, 0, 0.8 m)`、原分割 mask、原 AnyGrasp 输入配置与未修改的 top-K 输出。没有新增随机场景，也没有改 AnyGrasp、FR3Backend、扫描或 Real2Sim 模块。FR3 原程序没有场景 seed 参数；其 10 次是相同固定场景的重复执行，并非 10 个独立物体位姿。R1 的 base 仍为 `(0.329, -0.175, 0.237 m; yaw 56.295°)`，其支架属于 R1 安装几何。

原 FR3、当次重新运行的 FR3、R1 Raw、R1 Adapted 的 10 次点云/mask 与 grasp JSON 都通过逐次 SHA-256 校验。点云文件 SHA-256 为 `1bb6b1767254a86c1104c3e01628caa265ea7bf9c3acb00d936a501f4ccdb3de`，AnyGrasp top-K JSON 为 `50e0a2f758a46edb51f21dee75eae8283cc711f793c2fade7f2c2c9402ea86f3`。各组使用同一批 11 个原始候选。校验详情在 `results/r1a7_fr3_exact_ab/input_audit.json`。

## 三组结果

| 指标 | FR3 原 pipeline | R1 Raw AnyGrasp | R1 Dex1 adapted + contact recovery |
|---|---:|---:|---:|
| 完整可执行候选（单次唯一输入） | 10/11 | 0/11 | 2/11 |
| 完整可执行候选（10 次重复累计） | 100/110 | 0/110 | 20/110 |
| 至少一条候选规划成功的试验 | 10/10 | 0/10 | 10/10 |
| 接触丢失事件 | 0 | 未到接触阶段 | 0 |
| 最终抬升并保持成功 | 10/10 | 0/10 | 10/10 |

“完整可执行”由三个独立 MoveIt 候选审计统计：准确 IK、joint margin、Dex1/Franka 抓取几何、self/table/scene collision、完整 approach 和 lift 路径及全局 pregrasp 规划。FR3 11 个中 1 个 approach 失败；R1 Raw 的 11 个中 7 个未通过 Dex1 接触几何、4 个碰撞；R1 Adapted 的 11 个中 2 个完整可执行，另 7 个几何失败、2 个碰撞。这里的 100/110、20/110 是同一 11 候选集重复 10 次，不能当作 110 个独立场景。

R1 Adapted 的 10 次均选择原始 rank 9 的局部 variant，平移幅度 24.90 mm、旋转 0°，J5/J6/J7 最小 joint-limit margin 0.348 rad。实际抬升 9.77–10.03 cm，保持采样跨度 2.067 s。R1 Raw 10 次均以 `NO_EXECUTABLE_CANDIDATE` 结束，没有进入规划与接触；并非这批目标的 IK 或夹爪驱动失败。R1 Adapted 没有最终 IK、碰撞、规划、坏接触、接触丢失或 drop 失败。

| R1 失败阶段（最终试验数） | Raw | Adapted |
|---|---:|---:|
| 无完整候选：Dex1 几何/碰撞筛选 | 10 | 0 |
| IK | 0 | 0 |
| 规划 | 0（未进入） | 0 |
| 坏接触 | 0（未进入） | 0 |
| 接触丢失 | 0（未进入） | 0 |
| Drop | 0（未进入） | 0 |

## R1 代码改动与验证边界

- 保留 AnyGrasp 原始 pose、score、rank；Dex1 适配继续围绕原候选搜索 position、orientation、grasp depth，按官方 Dex1 指端 mesh、接触几何、R1 IK/joint margin、碰撞与完整路径筛选排序。没有添加单物体规则。
- R1 执行在 close 后检查双指接触力或驱动停滞，再以带载 2 cm micro-lift 验证物体是否跟随。若坏接触或 micro-lift 接触丢失，停止后续 lift；Adapted 模式允许一次重新闭合、一次通用 3 mm depth correction，并在需要时重置场景尝试下一个已适配候选。Raw 模式没有恢复。
- 本轮 10 次没有接触丢失，因此恢复分支**未获得 Isaac 物理失败场景的验证**。它不应被记作此次 10/10 成功的原因；之前的单盒子 Adapted 版本已经达到 10/10。
- 原 FR3 的闭合证据包含双指接触力；当前 Dex1 导入的指端 contact sensor 有时报告零，R1 使用双指驱动停滞作为初筛，micro-lift 物体随动才是物理夹持的决定性检查。这一传感差异仍需单独验证。

**结论：**在 FR3 完全相同的感知输入和固定盒子场景下，R1/Dex1 经过 robot-specific adaptation 后，最终成功率达到 FR3 的 10/10；Raw 仍为 0/10。R1 只有 2/11 可执行候选，低于 FR3 的 10/11，且 10 次都依赖同一个约 25 mm 偏移的候选。这个结果确认了同场景执行能力，不能覆盖此前多物体实验中低盒子 0/5、高盒子接触丢失及视野问题，因此暂不进入铰链阶段。
