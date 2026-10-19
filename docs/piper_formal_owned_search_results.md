# PiPER 正式微波炉把手：统一 ownership 后的固定底座搜索

日期：2026-10-02。结果：**本轮限定搜索未找到合法真实 grasp**。
全部实验已停止；未启用 mobile base，未执行拉动或开门。

## 搜索与执行 funnel

| 阶段 | 数量 |
|---|---:|
| 五个把手局部区域、每区域 69 个结构化位置/深度/RPY 姿态 | 345 |
| exact IK 可解 | 327 |
| 开口状态完整 approach 几何预检通过 | 155 |
| home → pregrasp 路径规划通过 | 155 |
| 实际执行的独立 Isaac episode | 155 |
| 实际进入 closure | 154 |
| 物理闭合完成 | 1 |
| 满足 bilateral pad + zero metal + cooked/raw audit 的合法闭合 | **0** |
| 合法 1–2 mm tangential pull | **0，未尝试** |
| 1° / 5° / 10° / 22° 开门 | **全部未尝试** |

按首次拒绝原因：154 次 `NATIVE_FORBIDDEN_CONTACT`，1 次
`OWNED_TRAJECTORY_REJECTED`。前者中 1 次发生在 PREGRASP，153 次发生在
CLOSE。因此不能将全部 155 次执行说成完成了 closure。

| 区域 | exact IK / 69 | approach 预检 | 实际执行 | 合法 grasp |
|---|---:|---:|---:|---:|
| 0，上端 | 57 | 0 | 0 | 0 |
| 1 | 69 | 29 | 29 | 0 |
| 2 | 69 | 61 | 61 | 0 |
| 3 | 69 | 57 | 57 | 0 |
| 4，下端 | 63 | 8 | 8 | 0 |

## 证据与判定

- 原生拒绝来自可识别的 **metal collider → 正式 handle collider**，没有
  用接触点投影或 ±1 mm 规则猜测 ownership。接触判定逐物理子步保留。
- 某些金属力出现在表面仍有正间隙时，来自原有 PhysX contact offset。
  网格未穿透不能覆盖原生金属承力的否决；offset、摩擦和夹紧力没有调整。
- `region_0101` 完成了闭合，真实 aperture 为 **19.8877 mm**，末态 pad
  force 为 **0 / 20.0085 N**，没有双侧抓住。closure 轨迹还在第 713 个
  审计样本被 official raw `gripper_link2 → handle` 非指垫碰撞否决。
  对应关节余量约 **0.6244 rad**，失败与该样本的 IK/限位无关。
  这里 20 N 是测得的单侧接触反力；两个 finger drive 的 10 N effort
  上限保持原值，没有增加夹紧力。
- 全部区域 episode 的最小实际 joint margin 为 **0.0504164 rad**，没有
  降低 0.05 rad 门槛。这个最小值接近门槛，不能称所有路径余量充足。
- 合法 grasp 未建立，滑移、稳定切向载荷及后段开门性能尚未测得。
  早期记录里的 `slip=0` 是 reference 尚未建立的默认值，不能作为稳定
  抓取证据。

## 范围、限制和保留参数

使用原执行入口 `raw_rank=9` 的姿态作为参考，沿同一个分割 handle 点云
的 5th–95th percentile 选择五个 anchor，各自计算局部 PCA frame。
结构化扰动最大 6 mm，局部 RPY 分量最大 12°；配置的平移外包络为 8 mm。
沿把手换 anchor 的总位移可以超过 8 mm，相对 raw 的实际位移/旋转逐项
记录。没有 object-id 条件或微波炉专用 offset。

这是一个参考姿态族内的有限搜索，**不能证明所有抓取姿态都无解**。
当前直接 blocker 是闭合接触合法性；exact IK 高通过率不能替代真实 grasp。
后段 reachability/joint limit 尚未在合法 grasp 下测试，暂不具备进入
mobile-base recovery 的条件。

固定 base 为 `(0.50, -0.55, -0.10 m; yaw=150°)`。
IK、官方 robot/gripper/object 尺寸、摩擦、finger effort、物体 articulation
和 joint-margin 门槛均未调整。R1/Dex1 baseline 和 AnyGrasp 未修改。

## 代码与验证

入口和复现命令见 `piper_owned_grasp_search.md`。SDK 几何导出时暂停物理
时间线；使用实际两指状态和 moving-link tensor pose，避免导出期间出现
未记录运动。每个子步检查非法接触，approach 提前 pad 接触也会拒绝。

接触回调只缓存路径和 finger body 前缀，真实接触记录回放验证 ownership/
force 输出不变，unknown/wrong-target 拒绝不变。URDF/SRDF 原子发布的并发
测试完成 20 次写入、2,000 对 XML 读取，零解析错误，URDF 内容未改变。

另保留早期两组诊断：旧 PCA frame 15 次、修正后的中心 frame 33 次。
包括本轮区域搜索，共 203 次实际 episode、201 次进入 closure、0 合法
grasp。旧 frame 的结果没有混入本轮 345→155 的主要 funnel。

## 交付记录

- `piper_formal_owned_search_results.json`：本轮逐候选结果、参数 hash、验证证据。
- `piper_formal_owned_search_candidates.json`：345 个 SE(3)、IK 解和局部扰动。
- 本地 `results/piper_formal_owned_search_delivery/`：完整汇总、代表性真实
  closure 视频、全部该 episode 的物理轨迹和实际 cooked exports。
- 服务器 `/data1/home/rangeryx/fr3_real2sim_piper_mobile/results/` 下保留每个
  episode 的视频、原生 contact、真实 q/aperture 和 cooked 几何。
