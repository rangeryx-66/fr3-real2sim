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

在最终交付前，从GitHub克隆到独立新目录，运行原源码审计 / bundle审计、上述迁移工具的实际目录准备及必要离线回归。结果与测试源码SHA随后补在本节；在记录出现前不要将“已准备”误读为“已验证”。

整个迁移过程不启动新Actor，不提交旧episode命令，不改变旧事故结论。最新原R02已关闭，仍NOT PASS / HARD_STOP_UNPARKED；转移时最终哈希存于私有 `CLOSED_R02_AT_TRANSFER.json`，早先live-World审计不删除。
