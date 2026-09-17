import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
diagnostic=json.loads((ROOT/'results/settling_diagnostic/summary/summary.json').read_text())
formal=json.loads((ROOT/'results/settling_regrasp_summary/summary.json').read_text())

def table(headers,rows):
    return '\n'.join(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|']+['|'+'|'.join(map(str,row))+'|' for row in rows])

lines=['# Settling-aware refinement 与有限 regrasp 实验结果','',
'冻结条件：总夹持力 30 N、pad-target μ=0.7；AnyGrasp 模型、推理、raw score/rank/pose 未改；MoveIt IK/PlanningScene/Cartesian collision checks、mesh hand gate 和 ≥8 cm/≥2 s 物理成功标准保留。GT benchmark 无 clutter，不使用 weld。','',
'## 1. Settling 与真实不稳定','',
f"30 次连续诊断：`{diagnostic['outcomes']}`。旧 gate confusion：`{diagnostic['confusion']['old']}`；新 gate 不带/带 transport probe：`{diagnostic['confusion']['new_without_probe']}` / `{diagnostic['confusion']['new_with_probe']}`。",'',
'bowl 在极慢继续 lift 中保持双指接触并被抬至 7.310–7.340 cm，最后 500 ms 已收敛且最终完全悬空；其累计约 36 mm / 28.1° 变化是从单侧边沿夹持姿态绕接触线转到重力平衡姿态的 settling。失败只来自没有达到严格的 8 cm 成功线，应记为 SETTLING_THEN_STABLE + INSUFFICIENT_LIFT，不是 DROP 或持续 slip。正式 5 mm micro-lift + 5 mm probe 尚未让 bowl 离开桌面支撑，因此 vertical-follow gate 在该阶段不适用于 bowl。mug 10/10 在继续慢速 transport 后完成 ≥8 cm 抬升与 ≥2 s 保持。','',
'冻结的新 gate 阈值：`'+json.dumps(diagnostic['thresholds'],ensure_ascii=False)+'`。','',
'## 2. 120 次严格配对 A/B','',
table(['物体','A 成功','B 成功','B 首次通过','regrasp 转化','B 失败分类'],[[name,f"{formal['stats'][name]['A']['success']}/{formal['stats'][name]['A']['n']}",f"{formal['stats'][name]['B']['success']}/{formal['stats'][name]['B']['n']}",formal['stats'][name]['B']['first_attempt_success'],formal['stats'][name]['B']['regrasp_conversions'],formal['stats'][name]['B']['categories']] for name in ['soup','banana','bowl','mug']]),'',
'配对变化：`'+json.dumps(formal['paired'],ensure_ascii=False)+'`。总体失败：`'+json.dumps(formal['failures'],ensure_ascii=False)+'`。','',
'## 3. Refinement 几何与力矩','',
table(['物体','B torque cost','B COM lever mm','B gravity roll lever mm','transport probes'],[[name,formal['stats'][name]['B']['torque_cost'],formal['stats'][name]['B']['COM_lever_mm'],formal['stats'][name]['B']['gravity_roll_lever_mm'],formal['stats'][name]['B']['transport_probes']] for name in ['soup','banana','bowl','mug']]),'',
'所有 refinement 都在原 pose 的冻结局部范围内生成，先过原 mesh gate，再按 contact patch、opposing normals、COM lever、gravity roll torque 和 contact asymmetry 排序；每个 parent 最多 12 个进入完整 MoveIt 检查。成功 regrasp 的逐次 offset 和前后指标保存在 summary.json 的 `successful_refinements`。','',
'## 4. 验收与下一步','',
'验收结果：`'+json.dumps(formal['acceptance'],ensure_ascii=False)+'`。',
'只有 `controls`、`complex_combined` 和 `advance` 同时通过，才建议重新跑完整冻结 AnyGrasp complex benchmark；在该 benchmark 继续保持原 non-target safety 后，才建议进入 support graph。若 bowl 仍失败且 refinement 已降低局部 torque，说明 ±10 mm/±15° 的执行层局部搜索无法跨越其大尺度 COM lever，剩余限制属于 Franka Hand 两指几何与 grasp family，而不是继续放宽 stability gate 的理由。','',
'## 5. 产物','',
'- `calibration/settling_gate.py`：rolling velocity、contact continuity、target/TCP synchrony 与分类。',
'- `src/grasp_refinement.py`：256 个确定性局部样本及 torque-aware 排序。',
'- `calibration/stability_backend.py`：MoveIt 检查、慢速 transport probe、下降/release/replan 和最多三次 regrasp。',
'- `results/settling_diagnostic/`：30 次诊断、trace、threshold 和 confusion matrix。',
'- `results/settling_regrasp_formal/`：120 次逐次 JSON、240 Hz trace、MoveIt checks 和 refinement metrics。',
'- `videos/settling_regrasp/`：代表性 measured-state replay。']
(ROOT/'SETTLING_REGRASP_RESULTS.md').write_text('\n'.join(lines)+'\n')
print('WROTE',ROOT/'SETTLING_REGRASP_RESULTS.md')
