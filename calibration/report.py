"""Generate tables and heatmaps from completed immutable calibration episodes."""
import json,csv,collections
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];base=ROOT/'results/hand_calibration/formal'
rows=list(csv.DictReader((base/'episodes.csv').open()));names=['soup','banana','bowl','mug'];forces=[10,20,30,40,60];mus=[.3,.5,.7,1.]
for r in rows:r['success']=r['success']=='True';r['force_total_N']=int(r['force_total_N']);r['mu']=float(r['mu'])
def subset(name=None,force=None,mu=None,control=None):return [r for r in rows if (name is None or r['target']==name) and (force is None or r['force_total_N']==force) and (mu is None or r['mu']==mu) and (control is None or r['controller']==control)]
def rate(rr):return f"{sum(r['success'] for r in rr)}/{len(rr)}"
lines=['# Franka Hand calibration — 完整统计','',f'有效正式试次：{len(rows)}/252。每个 force × friction × object 单元预定 3 次。','', '同一物体重复固定 GT 抓姿；这不是新场景泛化成功率。','', '|物体|力反馈（全部摩擦）|原位置控制 μ=0.7|力反馈 30 N / μ=0.7|','|---|---:|---:|---:|']
for n in names:lines.append(f"|{n}|{rate(subset(n,control='force'))}|{rate(subset(n,control='position'))}|{rate(subset(n,30,.7,'force'))}|")
lines+=['','## Force × friction 完整矩阵','','N 为双指法向力之和；每格为成功次数/重复数。','','|总力 N|μ|soup|banana|bowl|mug|','|---:|---:|---:|---:|---:|---:|']
for f in forces:
 for mu in mus:lines.append('|'+ '|'.join([str(f),str(mu)]+[rate(subset(n,f,mu,'force')) for n in names])+'|')
lines+=['','## 失败分类','','|物体|分类与次数|','|---|---|']
for n in names:lines.append(f"|{n}|{dict(collections.Counter(r['category'] for r in subset(n)))}|")
lines+=['','UNSTABLE_GRASP 表示 micro gate 已测到漂移/转动或接触中断，停止后未执行正式 lift；不能把它等同于已观测 DROP。未绕过 gate，因此不能估计误拦截率。','', '## 控制力对照','','以下为 close 后保持窗口的实测双指总法向力均值，不是命令值。','','|物体|位置控制 N|目标 10 N|20 N|30 N|40 N|60 N|','|---|---:|---:|---:|---:|---:|---:|']
def measured(rr):
 vals=[float(r['close_hold_force_1_N'])+float(r['close_hold_force_2_N']) for r in rr if r.get('close_hold_force_1_N') and r.get('close_hold_force_2_N')];return f'{np.mean(vals):.2f}' if vals else '—'
for n in names:lines.append('|'+ '|'.join([n,measured(subset(n,mu=.7,control='position'))]+[measured(subset(n,f,.7,'force')) for f in forces])+'|')
lines+=['','## Micro gate','','|物体|放行|拦截|已放行后失败|','|---|---:|---:|---:|']
for n in names:
 rr=subset(n);passed=[r for r in rr if r.get('gate_pass')=='True'];lines.append(f"|{n}|{len(passed)}|{sum(r['category']=='UNSTABLE_GRASP' for r in rr)}|{sum(not r['success'] for r in passed)}|")
lines+=['','## 文件','','- [逐次完整指标](results/hand_calibration/formal/episodes.csv)','- [分组统计 JSON](results/hand_calibration/formal/summary.json)','- [接触与重心力矩](results/hand_calibration/formal/contact_diagnostics.json)','- 每次完整状态与 240 Hz 物理接触日志保存在对应物体目录。','- [冻结协议](calibration/PROTOCOL.md)','- [冻结文件哈希](results/hand_calibration/controller_frozen.json)','']
(ROOT/'HAND_CALIBRATION_TABLES.md').write_text('\n'.join(lines))
fig,axs=plt.subplots(1,4,figsize=(15,4.5),constrained_layout=True)
for ax,n in zip(axs,names):
 data=np.full((5,4),np.nan)
 for i,f in enumerate(forces):
  for j,mu in enumerate(mus):
   rr=subset(n,f,mu,'force');data[i,j]=sum(r['success'] for r in rr)/len(rr) if rr else np.nan;ax.text(j,i,rate(rr),ha='center',va='center',color='white' if data[i,j]<.2 or data[i,j]>.8 else 'black')
 im=ax.imshow(data,cmap='RdYlGn',vmin=0,vmax=1);ax.set_xticks(range(4),mus);ax.set_yticks(range(5),forces);ax.set(title=n,xlabel='Pad-target friction',ylabel='Total normal-force target (N)')
fig.colorbar(im,ax=axs,shrink=.7,label='Pick success fraction');fig.suptitle('Fixed GT grasps, unchanged micro stability gate');fig.savefig(base/'force_friction_matrix.png',dpi=180);plt.close(fig)
print('REPORT',len(rows))
