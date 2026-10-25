# P0：同一 World 恢复与完整任务实测

本轮已完成。服务器冻结运行源码为 `2be107024a521539678ab952a0d27aa0edc633f9`，保留分支 `codex/v86-p0-verified-90`；对应本地实现为 `a99203c`。服务器最终针对性回归251项全部通过，无跳过，日志私有保存为 `P0_RECOVERY_REGRESSION_06.log`。之前失败日志和全部历史 NOT PASS 均保留。

## 原因与最小共享修复

原V81 session把白名单之外的局部异常统一升级为CONTROL_STATE_UNTRUSTED，task_recovery据此设置terminal_fault，session返回后finally关闭工作器，runner再关闭Isaac。初始化观察位于命令循环之前，路径异常也可直接进入finally；未达到UNLOADED_OBSERVATION被视为无法恢复。finish还曾按操作名称结束循环，未充分检查结果。ABC loaded-C包装对部分状态的覆盖同样需要约束。

新增共享recovery_supervisor明确区分ACTION_REJECTED、RECOVERABLE_FAILURE、SAFETY_STOP、EPISODE_COMPLETE和UNRECOVERABLE。普通拒绝或可信状态下的可恢复故障，结束的是当前动作；调用原受保护停止、卸载与停车owner，失效旧票据，保留原目标、物体实测状态和历史，然后从实际腕部重新观察。无停车路线但原监督器新鲜证实释放、卸载、静止时保留World为RECOVERY_EXIT_PENDING，只允许观察和经过检查的退离；不能借此抓取或移动底座。

未知执行器故障、无效F/T、非有限状态、原硬保护锁存仍禁止续动，未新增自动清锁。数值模型延迟保持待办，不阻塞后续安全操作。普通连续B的advisory不触发重新决策或卸载。finish只有实际完成目标、目标后反向、独立验证及确认停车，或真实授权终止条件成立，才结束session。

原碰撞owner确实查询当前moving()位姿，并非一律使用关闭状态几何。新增物体/底座/附着签名拒绝过期计划，完整路线检查所有原生时序样本。另修复float64规划与float32原生命令在原0.05rad余量边界不一致：反例分别为0.05000000000000071与0.04999993705749528rad。现在接近和停车检查实际命令dtype，原阈值未放宽。

## 同一 World 的实际证据

唯一计入完整成功的实例为 `v86_p0_45936_20261025_03`，UUID `676d0e1e-c36d-48c4-8c9f-5650e55f5868`，Actor PID3791961，World reset_count=0：

| 事件 | 实测结果与后续行为 |
|---|---|
| 迁移后旧观察路线关节余量不足 | 命令4执行前ACTION_REJECTED；原停车确认、双指0N；命令5改用更高的经过检查的视角并成功观察。 |
| 抓取后预注册碰撞候选拒绝注入 | 已通过真实抓取及1秒保持，随后明确标记engineering_injected；不是实际危险碰撞。卸载停车确认，非零物体状态1.2604364201251883e-5rad保留；同Actor继续新观察和自动抓取。 |
| 自然发生的持续传力退化 | 首段20.020°、反向12.077°之后，长程在54.634°因CONTACT_TRANSMISSION_STALL中断。原卸载停车成功；同World重新观察、更换抓姿，合法请求预载1.0→1.5N，继续剩余目标。 |
| 恢复后的大范围运动 | 达到90.0316°，卸载后最大实测90.0376027°；未重置物体。 |
| 辨识工作器延迟 | 冻结模型尚在计算，VALIDATION_DEFERRED；保留待办及90°当前状态。 |
| 高角重新观察NO_IK | 单个正面视角拒绝，World保留；改用斜视角，原碰撞检查通过并取得新鲜把手图。 |
| 独立反向验证 | 模型冻结后、未参与FIT的独立动作实际反向24.0593892°，TARGET_REACHED、physical_transmission_valid=true。 |
| 最终关闭 | 原卸载/停车/碰撞确认通过，EPISODE_COMPLETE / SAFE_PARK_VERIFIED；Actor和launcher均退出。 |

实际总耗时6728.566s，即112.143分钟，包含故障注入和恢复验证。本轮证明持续交互机制，不能声称比85.2分钟基线更快，也不能用单次结果证明总体成功率提高。

证据：[最终生命周期](FINAL_LIFECYCLE.json)、[余量恢复](LIVE_MARGIN_RECOVERY.json)、[抓取后注入恢复](LIVE_POST_GRASP_RECOVERY.json)、[真实传力退化恢复](LIVE_TRANSMISSION_RECOVERY.json)、[历史失败重放](FAILURE_REPLAY.json)。完整原始240Hz记录、图像、计划和哈希私有保存。

## 独立F/T Physics-ID评估

只在Actor退出并冻结模型哈希后，独立评估器访问隐藏参考；在线Agent未访问隐藏动力学。独立反向动作物理时长18.5s，原导数/边界支持下自由滚动预测17.4875s，仅使用一个真实初始q/qd及未见过的补偿F/T输入，不用后续真状态纠正。

冻结模型 `52d6be6e1e3cd85cc850e912d72bee3f7c146c9ed84de68ddc3fcfbde325f8ff`：反向预测RMSE **0.725764°**，终点误差 **0.803851°**，广义力矩残差RMSE **0.00091946Nm**。同模型相对独立authored参考的惯量误差 **−5.5726%**、阻尼误差 **+11.1143%**、FIT支持区间阻力曲线RMSE **0.00091103Nm**，scaled condition29.905、rank7。这些参考是所选刚体组合和被动模型，并非独立实测的有效PhysX动力学；条件标准差不含未知系统误差。没有用该验证数据重新拟合、挑选模型或覆盖旧模型。

见[冻结预测](FROZEN_PREDICTION.json)、[参数误差与隔离声明](PARAMETER_ERRORS.json)、[实测汇总](FINAL_AUDIT.json)。传力退化的不合格后缀未进入FIT。控制成功与参数精度仍分别评价。

## 安全、失败记录与回归边界

历史06在0°退出且停车未确认，仍EXIT_UNVERIFIED / NOT PASS。P0新01暴露审计事件参数phase重复引起TypeError，仍EXIT_UNVERIFIED / NOT PASS；已修正日志字段并增加相同Runtime.event调用契约回归。P0新02遇共享GPU1 SAM显存不足，保留World；同像素在GPU0原工作器14.938s成功。经用户明确批准一次仅换感知GPU重启，02先原生停车确认、优雅关闭，记录任务未完成；03为新的完整成功实例，绝不拼接不同episode。

原全局历史safety_hold原路径与原哈希保持不变，旧抽屉SAFE_PARK_UNCONFIRMED仍NOT PASS。新实例独立UUID/状态目录、全局资源互斥和原156步无负载检查保持有效；未手动移动或删除锁。251项回归覆盖真实共享dispatcher的初始化失败、IK、碰撞、余量、接触退化、无路径保持、硬事件不续动、过期场景、提前finish拒绝、数值延迟及实例隔离。

保留792fa3b和V8.4成功基线，未修改冻结物理owner、硬保护阈值或F/T估计数学。没有重跑180mm抽屉就不宣称它已在新源码重新验证。完成P0后，同一冻结代码已立即启动46427迁移实验；45746随后串行执行并监测另一被动滑轨，跨物体结果单独报告。
