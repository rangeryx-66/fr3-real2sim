# P0：同一 World 恢复监督器（验证进行中）

本地实现提交 a99203c；服务器冻结运行提交 2be107024a521539678ab952a0d27aa0edc633f9。服务器针对性回归实际运行251项，全通过、无跳过。测试日志私有保存为 P0_RECOVERY_REGRESSION_06.log；此前失败日志保留，没有覆盖。

## 已确认的终止链

1. V81 session 对有限白名单之外的异常返回 CONTROL_STATE_UNTRUSTED；task_recovery.after 将其记录为 terminal_fault；session 返回，finally 关闭工作器；外层 runner 导出并关闭 Isaac。这使部分纯观察/数值发布异常与控制故障共用任务退出路径。
2. 初始化观察发生在命令循环之前，路径异常能够直接到达 finally；反馈包构建的 RuntimeError 同样未被原读错误分支捕获。
3. 没有统一的失败后停止、卸载、确认保持/停车、失效旧计划、新鲜观察接口。未达到 UNLOADED_OBSERVATION 即归类 SAFE_BOUNDARY_UNAVAILABLE 并退出。
4. ABC 的 loaded C 包装曾能覆盖未锁存的 CONTROL_STATE_UNTRUSTED 结果；已要求真实控制故障优先。
5. 最后一次旧版本验证06仍触发 COMMAND_JOINT_MARGIN：命令最后逼近joint2=-2.009，原下限=-2.059。float64规划与float32原生命令数组在.05rad边界并不等价。新增原阈值反例证明float64通过而赋值后拒绝；现在接近/停车均按实际命令dtype检查。该修复不放宽硬保护。

## 最小共享改动

- recovery_supervisor 区分 ACTION_REJECTED、RECOVERABLE_FAILURE、SAFETY_STOP、EPISODE_COMPLETE、UNRECOVERABLE。未知执行器故障、状态读数失败、非有限值、无效F/T、真实硬保护绝不转换成继续权限。
- 可恢复动作结束后调用原减速与卸载/停车owner，记录实际结果；确认卸载且静止、但没有停车路径时保留同一World，进入RECOVERY_EXIT_PENDING。此时允许观察与经过检查的退出重规划，禁止新抓取或底座运动；后续实测停车成功才解除这个局部待退出状态。
- 强制取消ABC的contact-retention请求后才进行故障卸载；正常连续B动作/advisory不增加卸载。数值工作器延迟异步保留，不作为操作终止依据。
- 可恢复后失效执行票据/缓存，保留原目标、真实关节状态与记录。新观察来自当前实际腕部，不回放初始观察轨迹。
- 原生碰撞场景确实使用当前moving()位姿；没有证据证明它总在使用关闭几何。新增物体/底座/附着状态签名防止旧计划被静默用于变更后的场景；完整时序检查使用当前活动部件几何与原碰撞语义。
- finish被拒绝或停车未证实时，不能因为operation名称为finish就让session返回；完成必须有目标、到达后反向、独立验证及实测停车证据。

## 故障注入回归

真实共享session dispatcher的确定性fixture验证：初始化预检查失败 → IK拒绝 → 碰撞拒绝 → 提前finish被拒绝 → 更换候选 → 完成正反向/验证记录 → 合法finish。仅一次初始化、一个World对象；工作器仅在最后关闭。该项是软件故障注入，不能冒称实际PhysX接触试验。

另覆盖：完整时序中间点拒绝、实际命令精度反例、接触退化后卸载与当前状态保留、数值工作器延迟、无停车路径但静止保持、未确认保持拒绝续动、硬锁存/无效F/T/未知执行器错误不续动、advisory不恢复、场景改变拒绝旧抓取、退出成功解除待退出局部状态。原历史事故/全局资源冲突/新UUID原无负载检查回归均通过。

## 物理验证与证据边界

旧06实例源f110491，门0°、零抓取；底座迁移完成，随后重观察触发上述硬余量保护，Actor已关闭，终态EXIT_UNVERIFIED / NOT PASS。不得将旧先前的native_rest停车当作事故后停车。其终态、命令、进程和源码hash保存于私有P0_SOURCE_TRANSITION.json；没有复活或清除该实例状态。

按本轮P0要求，使用原launcher在独立实例 v86_p0_45936_20261025_01 验证修复，UUID 5c0ff884-93c5-4945-a8ef-d1a0a3bbc688。原156步无负载检查通过，F/T有效、双指0N、机器人/物体静止、释放与间隙确认、没有加载旧控制状态。随后新增路径审计把phase重复传给原Runtime.event，TypeError同时阻断初始化及退出规划。该集成失败无抓取、仍EXIT_UNVERIFIED / NOT PASS，保存P0_SOURCE_TRANSITION_02.json；不能计入同场景恢复成功。

已修正日志保留字段冲突，并增加与Runtime.event完全相同关键字所有权的回归，补测“恢复过程本身发生软件异常→原监督器新鲜确认静止卸载→保留World待退出重规划”，禁止将未证停车写成成功。最终冻结源在新实例 v86_p0_45936_20261025_02 验证。所有实例使用相同已承诺工程动力学，各有新模型/记录目录；one-shot planning rejection仅作为显式工程故障注入，不伪称真实碰撞。

[历史失败重放](FAILURE_REPLAY.json)包含真实02/04的规划失败和05/06的硬停止原trace哈希；离线fixture中规划失败不终止、硬事件仍终止且不卸载/清锁。这里不是PhysX实测。原精度反例为float64余量0.05000000000000071rad、原生命令余量0.04999993705749528rad；完整历史失败路径未重建，不能将反例冒称逐点重演整条真实轨迹。

完整90°、反向、独立验证与停车尚待实际记录。原792fa3b成功基线、180mm能力及六维F/T数学未修改；251项回归不等于新跨物体物理成功率证明。没有访问新实例的隐藏参数，也没有把不同episode的最大角度拼接。

资源问题与受控版本切换：02实例原配置感知GPU1在共享任务占用后SAM显存不足，World仍保留并继续执行已检查的观察动作。对完全相同新鲜像素，在GPU0原版SAM工作器14.938秒成功，原失败数据未覆盖、结果未注入运行中Agent。用户随后明确批准一次仅换感知GPU的安全停车后新实例。02实测native_rest停车、0N、碰撞复核通过，优雅关闭SAFE_PARK_VERIFIED；这是用户要求的配置切换，不是完整任务成功或同World恢复。环境唯一差异perception_gpu:1→0，资源锁/隔离/安全条件不变。03实例 UUID 676d0e1e-c36d-48c4-8c9f-5650e55f5868 已通过原无负载检查，GPU0共享物理与感知；首次SAM returncode=0，继续基于本实例当前RGB-D选择视角。

## 03实例中的实际同World恢复

- [关节余量拒绝](LIVE_MARGIN_RECOVERY.json)：命令4复现旧05/06的迁移后初始观察路径。新检查在执行前返回MOTION_PATH_UNAVAILABLE:JOINT_MARGIN，结果ACTION_REJECTED；原监督器复核native_rest停车、0N，取得新鲜实际腕部图。Actor3791961、UUID不变，门0rad、目标π/2rad和历史保留。命令5换用更高的检查过的视角，实际执行并取得完整把手几何；没有重启/瞬移/重置。
- [抓取后注入拒绝](LIVE_POST_GRASP_RECOVERY.json)：自动计划完整认证20°/8°/90°/66°；命令7通过原抓取和1秒保持，真实物体为1.259525e-5rad。预注册one-shot候选拒绝触发，明确标记engineering_injected，原生路线本身valid；不是实际危险碰撞。之后原卸载/停车通过，双指0N、实测碰撞valid，返回RECOVERABLE_FAILURE且episode_termination_required=false。恢复前后真实关节均1.2604364201251883e-5rad，未清零。同Actor接受命令8并完成新观察，命令9正在重新自动选择抓姿、认证原90°任务。

以上证明已超出“进程尚活着”：两次失败后都已实际执行不同的后续动作。但尚未证明本次90°完整任务或跨物体成功率提高；实际接触退化的恢复仍以随后的自然事件/完整记录为准，不能将候选拒绝注入冒充真实滑移或危险碰撞。
