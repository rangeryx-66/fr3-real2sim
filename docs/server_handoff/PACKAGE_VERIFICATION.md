# 服务器移目录交付核查

范围：完整Git代码 / 项目Skill / 文档迁移到同一Isaac服务器的新目录；不是重新安装Isaac，也不是新物理实验。

当前代码和报告已上传 `codex/v87-continuous-recovery`，`codex/r02-active-recovery` 单独保留792fa3b。最终交付SHA以GitHub分支及离线包manifest为准；此文件不写自引用commit hash。

## 包内与包外

- Git中包含当前V8.7适配器、冻结V8源码快照、完整Git祖先、测试、两套项目Skill及RUNBOOK/API、根handoff和运行README。
- `server_transfer.prepare` 只核对哈希和复制私有元数据，不导入Isaac / 驱动、不启动进程、不改锁、不授予交互准入。
- 服务器私有 `PRIVATE_CATALOG.json` 包含三案例的精确环境 / spec引用；446文件资源bundle、485个SAM源文件、权重及既有运行时保持独立、私有。
- 原始sensor / RGB-D / 视频、隐藏plant、模型、原锁存未发布；源历史新增352个blob的发布前扫描没有发现权重 / 原始telemetry路径、10MB以上blob或实际凭证模式。

## 已完成的本地准备检查

- 迁移标准库测试：6项通过，包括任意新代码目录、只读check不落盘、原输入哈希变化拒绝、不能另设共享互斥锁、拒绝覆盖和私有文件进入代码目录。
- handoff / README中本地Markdown链接检查通过。
- 本次仅修改部署文档和不启动仿真的搬迁工具；原冻结controller / scripts / configs / V8 Skill字节仍由原baseline_audit验证。

## 全新服务器目录验证

已从GitHub克隆到全新独立目录：
`/data1/home/rangeryx/agentic_real2sim_server_codex_20261026`。

测试源码：`047a249d65aa0038221faeaed1b140b56300f575`。随后仅补充本核查结果和文档，运行 / 安全源码没有再修改。

| 核查 | 实际结果 |
|---|---|
| GitHub克隆、完整祖先与对象连接 | 通过；`git fsck --connectivity-only --no-dangling`通过 |
| 新目录对旧Git对象库的依赖 | 无；临时reference加速后已`--dissociate`，没有alternates文件 |
| V8.0 baseline_audit | 606个物理源、71个旋转冻结源、956个发布源检查通过，failures为空 |
| 私有bundle审计与真实URDF重定位 | 通过；两套URDF的非路径XML和网格字节一致 |
| 项目模块导入 | V81、V84、V87、server_transfer均从新checkout加载 |
| 两套Skill及RUNBOOK/API | 四文件哈希核对通过，均位于新checkout |
| 45936、46427、45746私有模板准备 | 三个全新私有目录成功生成，未授予启动准入 |
| 原历史全局hold | 原SHA `f9cec4d6...879d58ab0`不变，未移动 / 清除 |
| 迁移 / staged exit / governor / instance / lifecycle / V8打包回归 | **63项测试、16项子测试通过，63.82秒** |
| 新物理实验 / 新Actor | **未启动**；这些结果不是90°或参数辨识PASS |

最初的普通全量网络克隆太慢，已停止仅属于本次打包的git进程；随后采用已有服务器Git对象作临时缓存，完成后独立化。没有停止任何机器人 / Isaac进程，也没有保留对旧checkout的对象引用。

第一次新目录审计发现新增导航使 `AGENTS.md` 偏离原V8发布哈希，原回归为1失败 / 62通过 / 16子测试。恢复原AGENTS字节后通过上述全部检查；冻结manifest没有改。失败日志和修复记录均保留，不把第一次写成成功。

私有验证证据在 `/data1/home/rangeryx/agentic_v86_private/server_codex_handoff_20261026`：
`MIGRATION_VERIFICATION_02.json`、`MIGRATION_REGRESSION_02.log`、`MIGRATION_AUDIT_FAILURE_01.json`及旧失败日志。离线Git bundle与最终SHA索引也放在该目录，包仅包含Git代码 / Skill / 文档 / 必需历史，外部资源仍由私有catalog明确引用。

整个迁移过程不启动新Actor，不提交旧episode命令，不改变旧事故结论。最新原R02已关闭，仍NOT PASS / HARD_STOP_UNPARKED；转移时最终哈希存于私有 `CLOSED_R02_AT_TRANSFER.json`，早先live-World审计不删除。
