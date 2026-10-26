# Agentic Real2Sim V8.7：服务器 Codex 详细交接

交接日期：2026-10-26（Asia/Shanghai）。本文件是项目状态和工作导航，**不隐含允许清除硬锁存、绕过 launcher 或立即重启试验**。本次用户要求先打包 / 上传 / 交接，不运行物理实验。收到后先只读核对真实文件，再依据用户的下一项明确任务继续。

## 1. 接手入口与当前最重要结论

运行与移目录命令在 [README_SERVER_CODEX.md](README_SERVER_CODEX.md)。项目 Skill 已在 `.agents/skills/`，不依赖任何 Mac 个人 Skill。先读根 `AGENTS.md`、原 `agentic-real2sim` Skill / RUNBOOK / API、`abc-continuous-v84` Skill，再读最新 [STAGED_EXIT.md](docs/experiments/v87_continuous_recovery_20261026/STAGED_EXIT.md)。

**当前不是已完成90°验收的版本。** 最近同一World先完成20.222°正向 / 12.255°反向及分阶段退出，后续长段到40.144539°触发原持续腕部力矩锁存，未完成卸载 / 停车。原World曾在有限诊断会话中保留；**交接时Actor和launcher已退出，World已不再存在**。不能把保存的40°传感器记录当成可恢复的活World，不能把rc=0当作安全停车。

用户最后追问的是：为什么保留World后仍没有完成安全恢复、重新观察和抓取。已明确承认：只实现了“保留World及禁止误续动”，没有接通完整的停止确认 → 受监督卸载 → 安全保持 / 退离 → 满足原条件的复位 → 新鲜观察 / 重新抓取 → 继续原目标。**这个共享恢复缺口仍未解决，不能写成完成。**

## 2. 用户长期目标与当前范围

目标是同一个ABC Agent在不同单目标旋转门和抽屉上，自动腕部RGB-D抓取、快速且鲁棒的大范围正反交互、场景内恢复、六维F/T Physics-ID及独立验证。最终主要测试45936 / R02、46427、45746；必要时其他历史旋转物体。不要逐物体硬编码抓姿 / 轨迹 / 动力学模型。

ABC保持：A观察 / 规划 / 接触建立，B连续执行长段与原实时监督，C根据真实物体结果继续或恢复。不要把小动作简单包成大工具，也不要因普通IK、碰撞候选、warning、接触变化而销毁World。当前目标不能被重置成新的初始状态。

最新已实施范围是“当前运动准入”和“预测终点停车”解耦，不重构ABC、不关闭碰撞、不改Physics-ID。90°实际终点退出验收尚未完成；继续处理恢复缺口时不要借机重写系统或放宽硬阈值。PPT补充是较低优先级，实验未完成前不花时间处理。

## 3. 源码与受保护基线

GitHub：`https://github.com/rangeryx-66/fr3-real2sim`，当前分支 `codex/v87-continuous-recovery`。完整历史保留；不用浅克隆。交接提交见 `git log` 和 `docs/server_handoff/PACKAGE_VERIFICATION.md`，不要把文档提交SHA误当成某次物理运行SHA。

| 版本 / 用途 | SHA / 分支 | 必须如何理解 |
|---|---|---|
| 原V8.0冻结 | `2f021639aea65e6ae859bac9a3385d49db10212f` | baseline_audit实际读取；原控制器 / scripts / configs / V8 Skill保留 |
| V8.1主开发 | `81580cf` | 未被R02原版恢复回退 |
| 原版R02恢复 | `codex/r02-revolute-recovery`, `751bfeea73da2ec6ecd4036ee577e668946eddb8` | 保留dd83cb5行为；最新63.053°复现不是独立验证PASS |
| 受保护90°成功 | `codex/r02-active-recovery`, `792fa3b` | 90.011° / 24.137°反向 / 独立验证 / 安全停车；已单独上传分支 |
| V8.4运行 / 报告 | `78b8352` / `90b391d` | ABC R02成功；旧抽屉硬停止仍NOT PASS |
| 最近45134运行（已关闭） | 服务器 `aaf583bea6ea414ff3e4a9e7601152455da44d10` | 48.645°，未完成90°；另有安全停车，不拼成最近R02结果 |
| 本次P0运动 / 退出核心 | 本地`50683b1`，服务器`48fdaac` | 离线native回放与回归通过 |
| 本次R02真实运行 | 服务器`4b94ad7f40eeb03d75d5058be09550c0a381fe0c` | 加strict launcher字段；40.145°硬停止，NOT PASS |
| 额外退出预算边界修复 | 本地`cb5baa4`，服务器`c80a87e` | 只在独立开发树测试，没有热替换进4b94ad7 Actor |

服务器参考源码：

- `/data1/home/rangeryx/agentic_v87_offline_replay_20261026`：4b94ad7，本次实际运行参考；保留，不作为新代码编辑目录。
- `/data1/home/rangeryx/agentic_v87_staged_exit_reserve_20261026`：额外修复和报告的服务器开发树，SHA因报告提交不同于Mac；关键代码字节与已推Git版本核对。
- 新服务器项目目录应从GitHub克隆；不复用Mac `.git` worktree链接。只读迁移验证会创建独立新目录，见打包验证文档。

## 4. 本次事故的最终真实状态

私有 episode：
`/data1/home/rangeryx/agentic_v86_private/v87_continuous_recovery_20261026/episodes/v87_r02_staged_exit_20261026_01`

- 原Actor3138040、launcher3137960，实例UUID `a5c89fe3-51f0-4e46-ab84-f4cc2d7d753b`。
- 原156步无负载准入通过。使用GPU5物理、GPU6感知，原12GB准入；没有改锁 / 阈值 / 仿真plant。
- sequence2：预抓实际路径重检拒绝夹爪 / 柜体碰撞，未抓取。保留World并通过真实释放状态确认、4cm退离、重新观察及原停车。
- sequence5：换IK / 接近路径后完成20.221940°正向、12.254582°反向；实际回到7.967358°，释放后约7.889042°。其释放 / 退离 / 停车均通过。
- 14°独立验证因反向FIT无合格样本而待办，没有执行。反向动作完成不等于参数拟合合格。
- 之后同World多种观察策略受遮挡或NO_IK；一次原生XY底座迁移到约(0.15,−0.70)m，新鲜图像完整。旧观察 / 规划 / 批准失效，物体未重置。
- sequence17：直达90°的当前运动覆盖不足（候选最多约58.34°），拒绝；不是预测停车失败。sequence18复用当前未执行候选池，55°段全检查通过。
- sequence19：真实推进到40.144539°，409.875仿真秒，原 `SUSTAINED_OVERLOAD`。原保护离线重放在同一时刻触发：0.15秒均值pad3.972986N（限4）、腕部力4.062922N（限8）、腕部矩0.600200N·m（限0.6）。TCP13.891mm/s（硬限30），跟踪误差约0.453mm、相对滑移约1.282mm。力矩为什么上升尚无完整根因结论，不能断言40°机械限位。
- 硬锁存后World保留并暂停，sequence20仅只读inspect。没有提交运动、清锁或finish。停止后没有stationary dwell / 控制器确认，不能称为机械臂已安全刹停。
- 之后原有限诊断资源窗口结束。`EPISODE_RESULT.json`为 `LATCHED_DIAGNOSTIC_RESOURCE_OR_OWNER_END`，无 `STOP_AND_EXPORT` marker；`PROCESS_COMPLETION.json`记rc0、Actor退出，**physical_outcome仍HARD_STOP_UNPARKED**。运行wall8711.010461s。最近只读核查原两个PID已不存在。
- 90°、到达90°后反向、独立验证、90°替代停车均未完成。不能用前一段已停车替代本次硬停止后的停车。不能从头重跑后声称同World恢复。

转移时的最终哈希与进程状态保存在私有：
`/data1/home/rangeryx/agentic_v86_private/server_codex_handoff_20261026/CLOSED_R02_AT_TRANSFER.json`。

当前事故锁：
`/data1/home/rangeryx/agentic_v86_private/v87_continuous_recovery_20261026/instances/a5c89fe3-51f0-4e46-ab84-f4cc2d7d753b/instance.safety_hold.json`。

另一个旧V8.4全局历史hold仍原样保留：
`/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/fast_runtime_v1_20261018/owned_actor.lock.safety_hold.json`，SHA256 `f9cec4d6bac8a3cba1681617b8d6ab64a5d7eb2087b05fb7a4c51dd879d58ab0`。新实例隔离准入没有删除 / 移动它，旧实例仍没有恢复资格。

## 5. 已修改的共享机制、未完成的机制

根因与实现细节见 [STAGED_EXIT.md](docs/experiments/v87_continuous_recovery_20261026/STAGED_EXIT.md)，无需重新做一次全面代码审查。

- `agentic_interaction_v81/revolute_certificate.py`、`replanning.py`：旧预测终点的 `plan_exit` / `exit_valid`硬依赖；新opt-in保留完整当前运动 / 刹停 / 精确终点保持证明，未来退出明确UNKNOWN。
- `agentic_interaction_v87/prismatic_motion.py`、`staged_cost.py`：同机制的轨道适配；保留原退出预算，不伪造未来停车零代价。
- `agentic_interaction_v87/staged_exit.py`：实际状态 / 停止保持 → 释放资格 / 确认 → 局部退离 → 新鲜观测 → 原生替代停车 / 测量确认。保留五类退出结果与已完成操作。
- `agentic_interaction_v81/operations.py`、`agentic_interaction_v84/session.py`：将上述结果接回共享恢复，不把停车搜索失败重新定义成操作未完成。
- `agentic_interaction_v81/launch.py`：仅V8.7 opt-in字段白名单，V8兼容spec去掉新字段。第一次漏PYTHONPATH scripts、第二次未注册白名单的启动失败均在创建Actor前，记录保留。
- `agentic_interaction_v87/command_governor.py`、`runtime.py`：已有最终命令FK限速 / 加速度、阶段切换先刹停，再恢复RELEASE限制。原真正硬保护不变。
- `agentic_interaction_v87/lifecycle.py`：硬锁存保留有限只读World，**只有inspect / finish，无复位owner**。这就是安全恢复仍未闭合的位置；保留World不等于获得续动资格。
- `agentic_interaction_v81/recovery_supervisor.py`：已有普通失败停止 / 保持 / 新鲜观测 / 状态与目标保留，不能重新造一整套状态机。
- `agentic_interaction_v81/instance_admission.py`：原全局互斥、历史事故校验、新UUID / 状态目录、原无负载自检；不把纯仿真隔离批准解释成可清任意全局锁。

原V8 Skill写着HARD不可清除，现有硬诊断会话也没有验证过的复位API。后续若用户明确要求补齐恢复，应先定义并验证安全资格与责任归属；不得将这份handoff当作直接清锁授权。可控载荷应尽可能在触发硬阈值前走受控反馈 / 卸载，但不能按40°写特例或提高阈值。不要在没有实际停止确认时直接重抓。

## 6. 已有证据与回归范围

离线native几何回放：R02当前44.899°状态，旧停车拒绝使旧链失败，新运动证明通过，5136原生构型；45746当前34.575mm状态同类回放通过5376构型。它们是明确标注的预测停车拒绝测试，不是实际90° / 新抽屉物理PASS。

P0选定回归133项+26子测试；strict launcher组合44+14；退出预算补测29+14。选择有重叠，不能相加。打包新增迁移工具6项标准库测试；全新目录验证结果在打包验证文档。单测 / 回放不能替代90°、参数准确度或硬停止后真实停车。

私有主证据目录：`/data1/home/rangeryx/agentic_v87_private/continuous_recovery_20261026`：

- `STAGED_R02_INTERIM_PHYSICAL_01.json`：早期完整正反段与实际退出，不替代后续终态；
- `STAGED_R02_HARD_GUARD_REPLAY_01.json`：原力矩guard精确重放；
- `STAGED_R02_HARD_STOP_AUDIT_01.json`：当时保留World的哈希快照；
- `STAGED_R02_RUNTIME_SIDECAR_01.json`：原运行时生成的asset-path文件及哈希，不是热改源码；
- `STAGED_EXIT_RESERVED_PHASE_REGRESSION_01.json`、`regression_13_staged_exit_final.log`、`regression_14_staged_launcher.log`：各来源分开；
- 原生回放的失败帧绑定 / rail-anchor重构失败也保留，不删失败记录。

最终关闭后的终态 / loaded-source / completion哈希看转移目录中的 `CLOSED_R02_AT_TRANSFER.json`，不要拿较早活World审计哈希覆盖最终记录。

## 7. 历史操作与Physics-ID：不能混写

| 场景 / 版本 | 已证结果 | 限制 |
|---|---|---|
| R02 `792fa3b` | 90.011°、反24.137°、独立验证、安全停车；原版复现63.042° / 31.545° | 保留为成功基线，不在其上试验性改代码 |
| V8.4 R02 | 90.035°、反24.197°；总耗时160.5→85.2min；命令37→15 | 不等于跨物体成功率已提高 |
| V8.6 45936 | 90.037603°、反24.059389°、独立验证、安全停车 | 本次4b94ad7的40°故障是另一episode |
| V8.6 45746 | 180.274740mm、反145.534169mm、独立再开145.698894mm、安全停车；另一rail全程0 | 这是单目标关节任务，不是多关节控制；用户接受监测另一关节 |
| 46427 | 历史腕部自动抓取开3.004496°；后续当前规则规划仍有失败 | V8.7新实际抓取未证明，不能称新成功物体 |
| 45134 V8.7 | 同World受控中断后继续，最终48.645°、原停车确认 | 未90°、未完整独立验证，后因用户改优先级安全结束 |

历史F/T评估：V8.6 45936惯量−5.57258%、阻尼+11.11427%、阻力RMSE0.000911N·m，高角反向预测0.725764°，终点+0.803851° / 17.4875s；45746质量+0.227229%、阻尼+1.126239%、阻力9.916e−5N、三段验证合并RMSE0.229050mm。原始入口和独立评估见相应报告，不拿这些误差填充本次未完成验证。

Physics-ID报告：[V8.3](docs/experiments/v83_physics_id_20261024/REPORT.md)、[V8.4](docs/experiments/v84_abc_20261025/REPORT.md)、[V8.5](docs/experiments/v85_sensorless_20261025/REPORT.md)、[V8.6](docs/experiments/v86_generalization_20261025/REPORT.md)。以仓库实际文件为准，个别历史目录名可用 `rg --files docs/experiments`定位。V8.3高角反向候选改善6.654→0.277°，但全独立数据9.483→11.044°，不能统一替换。V8.5 effort-only信号 / 本体补偿有适用域限制；本轮继续以六维F/T为主，不改辨识主体。

FIT / VALIDATION严格隔离，诊断用过的验证转开发集；中断 / 接触不可信记录按原准入排除。参数GT只由独立评估器读取，轨迹RMSE低不等于参数准确。不要让拟合不合格阻塞操作，也不要因此伪称验证通过。

## 8. GraspGenX现状与后续路线

保留规则候选优先，GX仅fallback。当前native-G1标定来自原URDF：GX+Z approach / +X closing到G1 TCP +Z / +Y closing，变换与夹爪开度 / 172mm参考偏置在 `agentic_interaction_v87/graspgenx_geometry.py`，不得用R1/Dex1标定。

当前46427保存帧的正确native重放是：规则0、GX0、MoE0安全可执行候选。早期错误harness用了abstract body并把retained command bias置零，名义通过数无效；负结果与旧文件均保留。不要继续盲目倍增候选，先查当前几何 / 真实夹爪关闭扫掠 / 坐标 / IK与接近路径的共同拒绝原因。实际新46427抓取仍待验证。

接手后建议最短顺序：

1. 只读核对代码、关闭终态、全部锁与资源，完成新目录审计；不因为handoff示例自动启动。
2. 针对用户最后指出的恢复缺口，明确哪一步缺控制器确认 / 合法卸载 / 复位资格，用历史故障与不驱动物理硬件的回归验证最小共享修复。
3. 保持原硬保护，补足同World恢复证据；90°任务不能用重置物体或不同episode拼接。
4. 真正可执行后再完成R02的90° / 反向 / 独立验证 / 分阶段退出；再做45746必要回归和46427迁移。
5. 每轮只验证一个明确假设，已有证据足够就回主线。不做无休止安全锁调查、规划器重构或局部测试。

GPU有剩余容量可共享，切卡不需用户重复授权；同一个机器人 / 同一Isaac控制资源不能有两个owner。不启动并行物理Actor。不要使用原SSH操作方式当运行架构，新的Codex直接在服务器本地操作。

## 9. Suggested skills / 给新Codex的启动提示

必须读取本仓库 `.agents/skills/agentic-real2sim/SKILL.md` 及references、`.agents/skills/abc-continuous-v84/SKILL.md`。它们已包含在Git和离线Git bundle中，不需要安装Mac全局Skill。以后若再次交接，优先更新这份状态导航与运行README，原始证据仍私有。

可粘贴给新Codex：

> 请在当前新checkout只读接手Agentic Real2Sim。先读AGENTS.md、HANDOFF_SERVER_CODEX.md、README_SERVER_CODEX.md及两套项目Skill与RUNBOOK/API，核对私有CLOSED_R02_AT_TRANSFER.json、实际Git SHA和安全记录。旧R02 World已关闭，40.145°硬停止仍NOT PASS /停车未确认，不能清锁或伪装恢复。先报告当前代码、资源、事故状态及尚缺的停止确认—安全卸载—合格复位恢复通道；不要直接启动新实验，等待我给下一步任务。全部操作以后直接在服务器本地执行，不通过SSH编排。
