# Agentic Real2Sim：服务器 Codex 接手与运行

本入口对应 `codex/v87-continuous-recovery`，是 xArm7/native-G1、Isaac Sim、ABC 持续交互工程线。根目录原 README 后半部分是更早的 FR3/PiPER 等研究记录，不是本轮启动入口。

**首次接手先只读，不启动实验。** 先读 [HANDOFF_SERVER_CODEX.md](HANDOFF_SERVER_CODEX.md)，核对私有事故终态，再处理尚未完成的恢复闭环。此次打包没有启动新实验，也没有清除任何安全锁。

## 1. 换到任意项目目录

```bash
git clone --branch codex/v87-continuous-recovery \
  https://github.com/rangeryx-66/fr3-real2sim.git agentic-real2sim
cd agentic-real2sim
export REAL2SIM_REPO="$(git rev-parse --show-toplevel)"
export REAL2SIM_PYTHON=/data1/home/rangeryx/isaaclab-arena/.venv/bin/python
export PYTHONPATH="$REAL2SIM_REPO:$REAL2SIM_REPO/scripts"
```

不要使用 `--depth 1`、仅下载 ZIP 或把旧目录的 `.git` 文件复制过来：原审计需要 `2f021639aea65e6ae859bac9a3385d49db10212f` 等完整 Git 对象。源码根目录由各模块的 `__file__` 推导；无需原 Mac 文件夹、旧聊天上下文或全局个人 Skill。

在服务器 Codex 中把工作目录设为这个新 checkout。首先依次读取：

1. `AGENTS.md`、`HANDOFF_SERVER_CODEX.md`；
2. `.agents/skills/agentic-real2sim/SKILL.md`；
3. 该 Skill 的 `references/RUNBOOK.md`、`references/API.md`；
4. `.agents/skills/abc-continuous-v84/SKILL.md`、`agentic_interaction_v84/README.md`；
5. `docs/experiments/v87_continuous_recovery_20261026/STAGED_EXIT.md`。

两套项目 Skill 都已提交 Git；保持原版 V8 Skill 字节不变。当前 V8.7 的增量和限制由 handoff、ABC 文档及实际源码说明，不能把 V8.0 的旧 CLI/旧 profile 当成最新 ABC 入口。

## 2. 代码包与服务器私有资源

| 内容 | 位置 / 处理 |
|---|---|
| 当前代码、测试、报告、项目 Skill、历史冻结源快照 | 本仓库，随 Git 完整克隆 |
| V8.0 冻结物理源 / 外部源映射 | `agentic_interaction_v8/BEHAVIOR.json`、`RELEASE_SOURCES.json`、`historical_sources/`；原 71+22 依赖保留 |
| Isaac Python、SAM3 源码 / overlay / 权重、ffmpeg、GraspGenX 环境 | 同一服务器的既有安装；不重新 pip 升级 |
| 官方机器人网格、校准、三物体输入 bundle | `/data1/home/rangeryx/agentic_v86_private/generalization_20261025/resources`；446 文件，有完整哈希 |
| 三案例的私有环境 / spec 索引 | `/data1/home/rangeryx/agentic_v86_private/server_codex_handoff_20261026/PRIVATE_CATALOG.json` |
| 原始 RGB-D / 数值 / 连续视频 / 隐藏 plant / 模型与事故锁 | 原服务器私有目录，**不上传 GitHub** |

这里验证的是“同服务器、全新代码目录”的迁移。跨机器还需按原许可证复制 / 安装外部运行时与资源，并重新确认安全资源域；不能宣称代码 Git 包包含数 GB 权重或 Isaac 安装。包内没有密码、API key、个人 Mac Skill 路径或已成功抓姿。

私有目录权限为700，环境和 spec 文件为600。`PRIVATE_CATALOG.json` 固定每个环境 / spec 的 SHA256。`TEMPLATE_PREPARATION.json` 记录来源与改动：45936 复用此次 P0 的预声明；46427 加 `staged_exit:true`；45746 从原 V8.6 spec 改为 V8.7 profile 并加 staged exit。**后两项只是准备好的待验证模板，不是物理通过证据。**

原 canonical Python、共享 `actor_lock`、全局保护和已审核的历史事故绑定必须保持。源码可以换目录；不能因为换项目目录而改成一个新的互斥锁、隐藏旧事故或绕过 launcher。原 `ISOLATION_DOMAIN.json` 限制私有输出在 `/data1/home/rangeryx/agentic_v86_private` 内；准备工具会检查，拒绝把私有输出写进 Git checkout。

## 3. 只读核查：不会创建 World

```bash
cd "$REAL2SIM_REPO"
"$REAL2SIM_PYTHON" -m agentic_interaction_v81.preflight
"$REAL2SIM_PYTHON" -m agentic_interaction_v8.cli audit \
  --bundle /data1/home/rangeryx/agentic_v86_private/generalization_20261025/resources
"$REAL2SIM_PYTHON" -m server_transfer.prepare check \
  --catalog /data1/home/rangeryx/agentic_v86_private/server_codex_handoff_20261026/PRIVATE_CATALOG.json
```

这只检查代码 / 资源来源，不授予交互准入。SAM 文件 / 权重、可用 GPU 容量、互斥锁、实例安全准入与无负载自检仍由原 launcher 检查。测试环境完整版本见 `docs/experiments/skill_v80_20261023/ENVIRONMENT_LOCK.json`；原测试栈是 Isaac6.0.1 / Python3.12.14。不要把本 Mac 的 Python 当作服务器仿真 Python。

离线回归（不运行 Isaac）：

```bash
PYTHONPATH="$REAL2SIM_REPO:$REAL2SIM_REPO/scripts:$REAL2SIM_REPO/tests" \
  "$REAL2SIM_PYTHON" -m pytest -q \
  tests/test_server_transfer_package.py \
  tests/test_staged_exit_v87.py \
  tests/test_command_governor_v87.py \
  tests/test_instance_admission_v86.py \
  tests/test_lifecycle_recovery_v87.py
```

迁移工具的独立标准库测试也可运行：

```bash
python3 -m unittest discover -s tests -p test_server_transfer_package.py -v
```

## 4. 准备一次新任务：只复制元数据，不启动仿真

先读取 handoff 中的事故状态。下面命令只生成新的私有环境 / spec / 回执；它不能恢复已关闭 World，也不会改任何锁：

```bash
export REAL2SIM_RUN=/data1/home/rangeryx/agentic_v86_private/server_codex_runs/first_case
"$REAL2SIM_PYTHON" -m server_transfer.prepare prepare \
  --catalog /data1/home/rangeryx/agentic_v86_private/server_codex_handoff_20261026/PRIVATE_CATALOG.json \
  --case 45936 \
  --destination "$REAL2SIM_RUN" \
  --episode-id server_codex_45936_NEW_UNIQUE_ID \
  --physics-gpu 5 --perception-gpu 6
```

案例可选45936、46427、45746。每次更换唯一 `--destination` 和 `--episode-id`；工具拒绝覆盖和原 episode ID。46427 的 GX 配置可用 `--grasp-gpu N` 明确切卡；未注册 GX 的案例不能凭该参数启用 GX。其余行为字段、plant commitment、校准和安全绑定原样保留。

生成 `environment.json`、`spec.json`、`PREPARED_TRANSFER.json`。核查回执中 `launch_admission_granted:false`、`no_physics_started:true`。它仅解决路径 / 配置搬迁，**不是替代启动入口**。原始模板的历史授权文字是来源记录，不能代替当前用户任务。

GPU不必空闲，剩余显存满足原守卫即可。默认 physics 入口要求至少12000MiB剩余；现有 measured-capacity 工程方案需合法、匹配的实测证据。不要靠修改阈值、伪造峰值或任意增加超时准入。旧的 GPU5/6 只是此次案例配置，接手后重新读取 `nvidia-smi` 选择，不依赖历史空闲状态。

## 5. 原 launcher：仅在用户要求启动且原安全条件通过后使用

**不要在首次接手时直接执行这一节。当前优先缺口是硬停止后的合法恢复链，见 handoff。** 不要用未修复代码再从0°重跑来替代原40°失败。

已获当前任务授权、原资源 / 安全检查均成立后，启动命令是：

```bash
cd "$REAL2SIM_REPO"
nohup "$REAL2SIM_PYTHON" -u -m agentic_interaction_v81.launch \
  --spec "$REAL2SIM_RUN/spec.json" \
  --environment "$REAL2SIM_RUN/environment.json" \
  > "$REAL2SIM_RUN/launcher.log" 2>&1 < /dev/null &
printf '%s\n' "$!" > "$REAL2SIM_RUN/launcher.pid"
```

这仍经过原全局互斥、旧事故完整性、新实例 UUID / 独立状态、Actor 父进程归属、无负载自检和全部物理保护。禁止直接调用 `agentic_interaction_v81.entry`、自建替代 launcher、清锁或移动 `safety_hold`。

若 GitHub 在服务器需要现有代理，可在服务器 home 中显式 `source ./proxyon.sh` 再克隆；不把代理凭证或环境值写入日志 / Git。实验入口本身不依赖 SSH，Codex可直接在服务器本地调用工具。

## 6. ABC 操作与结果读取

```bash
export REAL2SIM_EPISODE="$REAL2SIM_RUN/episodes/server_codex_45936_NEW_UNIQUE_ID"
"$REAL2SIM_PYTHON" -m agentic_interaction_v8.cli status --episode "$REAL2SIM_EPISODE"
```

先检查 `PROCESS_COMPLETION.json`；已结束时，旧 WAITING / DECISION_PACKET 不是活会话。未结束时以 `AGENT_SESSION_STATUS.json` 的 sequence / state / deadline 为准，再读 `DECISION_PACKET.json`、本次响应和当前图像。

命令写入一个完整 JSON 后，统一用 ABC 原子 publisher：

```bash
"$REAL2SIM_PYTHON" -m agentic_interaction_v84.transport \
  --episode "$REAL2SIM_EPISODE" --sequence 0 --json /private/command_000.json
```

不要用旧 v8 generic publisher 发送 `acquire_execute`、`continue_motion` 或硬锁存诊断命令；也不要直接写半截 `agent_commands/NNN.json`。发布成功不等于执行成功；读取 `.accepted.json` 和最终 `NNN.json`，再发下一个 sequence。

实际动作顺序：

1. `observe` / 已检查的 `observe_view`；真实查看 RGB、mask、depth、点云板。
2. `prepare_grasp`：预声明 FIT 与至少一条独立完整 VALIDATION，即使打算先执行正向子集。当前协议会拒绝仅 FIT 的声明。
3. 实际查看新 `AGENT_GRASP_PLAN_INSPECTION.png`，绑定其 SHA。几何覆盖 / 规划成功不是物体操作成功。
4. `acquire_execute` 只执行预声明且完整检查过的有限段；B底层连续执行。
5. C只在可信接触和原有限期限内用 `continue_motion`。不稳定时安全卸载，在原World实际角度重新观察 / 抓取 / 迁移。
6. 退离后的图像早于最终停车结束时，不能用于下一次移动批准。应取得**停车后**新图像。换底座后需显式 `observe`，旧计划失效。
7. 已用于拟合或模型选择的记录不能充当最终独立 VALIDATION；没有合格模型就保留待办，操作仍可推进。

普通拒绝 / 可恢复失败保持World；未知危险和硬锁存按原安全协议处理。当前硬锁存诊断只接受 `inspect` / `finish`，没有合法自动清锁入口。不要把保持World等同于机械臂已刹停、可松手或可续动。

## 7. 结束与验收

分别记录 operation、release、local retreat、parking；前面通过不能代替后面。完成 / 放弃试验后走原收尾并确认自己拥有的 Actor、launcher、GPU资源释放。进程rc=0不等于物理PASS。保留事故锁、失败动作、未完成停车和全部原始证据，不拼接不同 episode。

本次打包的移目录核查结果与源码 / 包索引见 `docs/server_handoff/PACKAGE_VERIFICATION.md`；私有证据索引留在服务器，不在 GitHub 放 RGB-D、NPZ、视频、权重或隐藏物理文件。
