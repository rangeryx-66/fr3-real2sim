import json
import subprocess
from pathlib import Path
from collections import Counter
ROOT=Path(__file__).resolve().parents[1]
runs=[]
for d in sorted((ROOT/'results').glob('run_*')):
    f=d/'summary.json'
    if f.exists():runs.append((d,json.loads(f.read_text())))
formal=[(d,s) for d,s in runs if s['trials']==10]
assert formal,'No completed 10-trial batch'
run,summary=sorted(formal)[-1]
audit=json.loads(subprocess.check_output(['/opt/anaconda3/bin/python',str(ROOT/'src/audit_results.py'),str(run)]))
(run/'audit.json').write_text(json.dumps(audit,indent=2))
trials=[json.loads(p.read_text()) for p in sorted(run.glob('trial_[0-9][0-9].json'))]
pilots=[]
if (ROOT/'results/trial_01.json').exists():pilots.append(json.loads((ROOT/'results/trial_01.json').read_text()))
for d,s in runs:
    if d!=run:
        pilots.extend(json.loads(p.read_text()) for p in sorted(d.glob('trial_[0-9][0-9].json')))
rows=['| 次数 | 结果 | 抬升 cm | 保持 s | 保持期最小单指接触力 N | 高度漂移 mm | 候选排名（从 0 开始） |','|---|---|---:|---:|---:|---:|---:|']
for a in audit:
    rows.append('| '+ ' | '.join(str(a.get(k,'—')) for k in ['trial','category','lift_cm','hold_seconds','min_finger_force_N','drift_mm','selected_rank'])+' |')
pilotrows=['| 联调实验 | 结果 | 说明 |','|---|---|---|']
for i,r in enumerate(pilots,1):pilotrows.append(f"| {i} | {r['category']} | {r.get('detail','完整物理抓取成功')} |")
text=f'''# FR3 独立 MoveIt 抓取验证报告

正式连续批次：**{summary['successes']}/10 成功**。验收是否通过：**{summary['passed']}**。场景未扩展。

服务器目录：`/data1/home/rangeryx/fr3_moveit_grasp`。

正式证据目录：`{run.relative_to(ROOT)}`。每次都重置同一 box、重新采集仿真深度、重新运行 AnyGrasp、重新调用 MoveIt 规划和执行。成功由自由刚体的物理接触和高度时间序列确认，未将 box 绑定到仿真手部。

## 每次正式实验

{chr(10).join(rows)}

该表由 `audit_results.py` 独立复核原始样本生成；保持窗口要求至少 2 秒，双指每个采样点 >0.1 N，物体保持高度至少高于初始 7 cm，峰值抬升至少 8 cm，窗口内高度漂移不超过 1 cm。完整原始数据见各 `trial_XX.json` 中的 `close_samples`、`hold_samples` 和 `executions`。

## 失败统计

正式连续批次：`{json.dumps(summary['categories'])}`。

此前保留的联调实验：`{json.dumps(dict(Counter(r['category'] for r in pilots)))}`。联调失败不从历史中删除，也不混入最终冻结配置的 10 次批次。

{chr(10).join(pilotrows)}

联调故障已修复：ROS 消息要求显式 float；AnyGrasp 子进程须使用自身 PATH 才能完成 MAC/许可证检查；MoveIt 附着目标时已自动从 world 移除，不应重复 REMOVE。

## 系统和坐标核验

- MoveIt 2 Humble：KDL IK、OMPL RRTConnect、PlanningScene 碰撞检查、ExecuteTrajectory。
- Isaac Sim 6.0.1：官方 FR3 + Franka Hand，共用官方 URDF，导入物理 USD；机械臂只接受关节轨迹目标。
- AnyGrasp：原环境与模型不变，输出按评分排序，经标准 NMS 后最多 20 个候选。
- 本批次 MoveIt/Isaac TCP 位置最大差：{max(r['tf_check']['position_error_m'] for r in trials):.3g} m；角度最大差：{max(r['tf_check']['rotation_error_rad'] for r in trials):.3g} rad。
- 坐标单元测试：3 项通过（正交性/反射、无效数值、顶视与指尖对齐）。
- 只读负例：不可达位姿返回 -31；桌面穿透位姿返回左右手指/table 碰撞。证据见 `results/negative_checks.json`。
- 模型哈希：`model_checksums.txt`；环境固定版本：`ros_environment_explicit.txt`；源文件清单：`source_manifest.sha256`。

详细转换公式、系统架构、文件职责、启动命令见 `README.md`。

## 当前主要瓶颈与边界

1. 当前是固定单 box 的验证，不代表复杂场景或真机成功率。相机标定为已知仿真外参，目标 mask 由真实渲染深度点与仿真 box 位置确定；真机迁移需要实际标定和目标分割。
2. 仿真与 ROS 通过本机 HTTP 连接以隔离 Python/CUDA 环境，适合单机械臂串行实验；相机渲染、逐点物理执行和进程启动影响吞吐。
3. 夹爪通过位置驱动闭合，物理力上限来自导入 URDF/drive。首版桥尚未把 GripperCommand 的 max_effort 动态映射为仿真力上限；真机使用官方 Franka gripper action 时需核对力参数语义。
4. 检查/规划均通过 MoveIt，但失败后的 recovery、动态障碍、复杂 bbox 未实现，符合本次范围。

## 启动

在服务器上述目录中分开运行：

```bash
bash run.sh sim
bash run.sh bridge
bash run.sh moveit
bash run.sh trial 10
```

下载前执行 `source /data1/home/rangeryx/proxyon.sh`。运行过程不改动原 Arena、AnyGrasp 模型、权重或旧 executor。
'''
(ROOT/'REPORT.md').write_text(text)
print(run)
print(summary)
