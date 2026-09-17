"""Build complete tables from audited saved results; no experiment changes."""
import csv,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/arena_complex_summary'
s=json.loads((OUT/'summary.json').read_text());rows=list(csv.DictReader((OUT/'episodes.csv').open()))
names={'mustard':'芥末瓶','raisin':'葡萄干盒','hidden_tuna':'扁金枪鱼罐','bowl':'碗','banana':'香蕉','sugar':'糖盒','soup':'汤罐','mug':'带柄杯','ALL':'总体'}
lines=['# Arena 冻结执行层泛化：数据表','',f'已审计 {len(rows)} / 40 个正式 episode。rank 为 AnyGrasp 原始零基 rank。','', '| 物体 | 成功 | 成功率 | 非目标接触 | 扰动 | 无候选 | scene 拒绝数 | pad 拒绝数 |','|---|---:|---:|---:|---:|---:|---:|---:|']
for name,x in s['stats'].items():
    lines.append(f'| {names[name]} | {x["success"]}/{x["n"]} | {x["rate"]:.1%} | {x["non_target_contact"]} | {x["disturbance"]} | {x["no_candidate"]} | {x["scene_filtered"]} | {x["pad_filtered"]} |')
lines+=['','## Episode 终止分类','','| 类别 | 数量 |','|---|---:|']
for cat,n in s['stats']['ALL']['categories'].items():lines.append(f'| {cat} | {n} |')
lines+=['','## 候选过滤分类','','每个候选按首先失败的检查归类；不能把这些候选数当成 episode 数。','', '| 类别 | 数量 |','|---|---:|']
for cat,n in s['candidate_failures'].items():lines.append(f'| {cat} | {n} |')
lines+=['','## 逐次结果','','| seed | 目标 | 结果 | rank | coverage | 接触 | 扰动 | 候选 | scene拒绝 | pad拒绝 | 规划秒 |','|---:|---|---|---:|---:|---|---|---:|---:|---:|---:|']
for r in rows:
    cv=f'{float(r["selected_coverage"]):.1%}' if r['selected_coverage'] else '—'
    lines.append(f'| {r["seed"]} | {names[r["target"]]} | {r["category"]} | {r["selected_rank"] or "—"} | {cv} | {r["non_target_contact"]} | {r["disturbance"]} | {r["candidate_count"]} | {r["scene_filtered"]} | {r["pad_filtered"]} | {float(r["planning_seconds"]):.2f} |')
lines+=['','完整原始结果、240 Hz 接触及所有物体位姿：`results/arena_complex40`。','离线接触诊断：`diagnostics.json`、`actual_contact_surfaces.json`。','源资产、质量和实际材质：`asset_physics.json`。','所有正式成功均重新核验原抬升、双指接触和保持判据，没有放宽。']
(ROOT/'ARENA_COMPLEX_TABLES.md').write_text('\n'.join(lines)+'\n')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
classes=[k for k in s['stats'] if k!='ALL'];fig,axes=plt.subplots(1,2,figsize=(13,5))
x=np.arange(len(classes));vals=[s['stats'][k]['rate'] for k in classes];cis=np.array([s['stats'][k]['ci95'] for k in classes]);axes[0].bar(x,vals,color='#428caa');axes[0].errorbar(x,vals,yerr=np.maximum(0,np.array([np.array(vals)-cis[:,0],cis[:,1]-vals])),fmt='none',color='black',capsize=3)
axes[0].set_ylim(0,1.1);axes[0].set_xticks(x,classes,rotation=40,ha='right');axes[0].set_ylabel('Pick success rate');axes[0].set_title('Five trials/class; Wilson 95% intervals')
for i,k in enumerate(classes):axes[0].text(i,vals[i]+.02,f'{s["stats"][k]["success"]}/5',ha='center')
cats=list(s['stats']['ALL']['categories']);axes[1].barh(cats,[s['stats']['ALL']['categories'][k] for k in cats],color=['#3f9a70' if k=='SUCCESS' else '#b76b58' for k in cats]);axes[1].set_xlabel('Episodes');axes[1].set_title('Original terminal categories')
fig.tight_layout();fig.savefig(OUT/'overview.png',dpi=170);plt.close(fig)
