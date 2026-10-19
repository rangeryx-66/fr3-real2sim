"""Post-run passive comparison; never loaded in the physical actor."""
from pathlib import Path
import json,gzip,pickle,numpy as np,collections
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');out=ROOT/'results/45415_precontact_continuity_20261011'
def summarize(folder):
 phases=collections.defaultdict(lambda:{'samples':0,'duration_s':0.,'max_TCP_m_s':0.,'max_command_delta_rad':0.,'peak_pad_n':[0.,0.]});curves=[];prev=None;entry=None;near_entry=[];grasp=None
 events=json.loads((folder/'events.json').read_text());grasp=next((e for e in events if e.get('event')=='GRASP_VERIFIED'),None)
 for f in sorted((folder/'observations_chunks').glob('*.gz')):
  for r in pickle.load(gzip.open(f,'rb')):
   p=phases[r['phase']];p['samples']+=1;p['duration_s']+=1/240;p['peak_pad_n']=np.maximum(p['peak_pad_n'],list(r['forces_n'].values())).tolist()
   v=0. if prev is None else float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(prev['T_tcp'])[:3,3])*240)
   p['max_TCP_m_s']=max(p['max_TCP_m_s'],v)
   if prev is not None:p['max_command_delta_rad']=max(p['max_command_delta_rad'],float(np.max(abs(np.array(r['command_q_arm'])-prev['command_q_arm']))))
   if entry is None and r['phase']=='SLOW_CLOSE':entry={'t':r['t'],'actual_TCP_m_s':v,'command_delta_rad':p['max_command_delta_rad'],'loads_n':r['forces_n']}
   if entry is not None and r['t']<=entry['t']+.1:near_entry.append({'t_from_entry':r['t']-entry['t'],'actual_TCP_mm_s':v*1000})
   if r['t']<65 or p['samples']%24==0:curves.append([r['t'],v*1000,r['door_angle_deg'],r['relative_translation_slip_m']*1000,r['margin_rad'],*r['forces_n'].values()])
   prev=r
 return {'phases':dict(phases),'closure_entry':entry,'closure_entry_window':near_entry,'grasp':grasp},np.array(curves)
old,oc=summarize(ROOT/'results/45415_closure_verification_20261011/physical_01');new,nc=summarize(out/'physical_01');m=json.loads((out/'physical_01/PHYSICAL_METRICS.json').read_text());baseline=json.loads((ROOT/'results/45415_closure_verification_20261011/physical_01/PHYSICAL_METRICS.json').read_text())
result={'classification':'POSTRUN_EVIDENCE_ONLY','before':old,'after':new,'PREGRASP_peak_reduction_percent':100*(1-new['phases']['PREGRASP']['max_TCP_m_s']/old['phases']['PREGRASP']['max_TCP_m_s']),'closure_entry_peak_reduction_percent':100*(1-new['closure_entry']['actual_TCP_m_s']/old['closure_entry']['actual_TCP_m_s']),'current_physical_metrics':m,'baseline_metrics':{k:baseline[k] for k in ['physical_PASS_30deg','max_actual_angle_after_valid_grasp_deg','max_TCP_velocity_m_s_by_phase','max_handle_relative_translation_m_after_grasp','minimum_joint_margin_rad_after_grasp','peak_pad_loads_n_all','simulation_s','wall_s']}}
(out/'COMPARISON.json').write_text(json.dumps(result,indent=2))
fig,axes=plt.subplots(2,3,figsize=(14,8))
for curve,label in [(oc,'e1e534c frozen'),(nc,'single continuity trial')]:
 axes[0,0].plot(curve[:,0],curve[:,1],label=label,linewidth=.7);axes[0,1].plot(curve[:,0],curve[:,2],label=label);axes[1,0].plot(curve[:,0],curve[:,3],label=label);axes[1,1].plot(curve[:,0],curve[:,4],label=label)
for summary,label in [(old,'frozen'),(new,'continuity')]:
 c=summary['closure_entry_window'];axes[0,2].plot([r['t_from_entry'] for r in c],[r['actual_TCP_mm_s'] for r in c],label=label)
axes[1,2].plot(nc[:,0],nc[:,5],label='left');axes[1,2].plot(nc[:,0],nc[:,6],label='right')
for ax,title,y in zip(axes.flat,['All-phase measured TCP speed','Actual opening','Closure entry (raw 240 Hz)','Handle-relative position drift','Measured joint margin','New-trial raw pad loads'],['mm/s','deg','mm/s','mm','rad','N']):
 ax.set_title(title);ax.set_xlabel('Simulation time (s)' if ax is not axes[0,2] else 'Time since closure entry (s)');ax.set_ylabel(y);ax.grid(alpha=.25);ax.legend(fontsize=8)
axes[1,1].axhline(.05,color='r',linestyle='--',linewidth=.8)
fig.suptitle('45415 / ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT — measured physics, one trial');fig.tight_layout();fig.savefig(out/'COMPARISON.png',dpi=150)
print(json.dumps({'before_PREGRASP_mm_s':old['phases']['PREGRASP']['max_TCP_m_s']*1000,'after_PREGRASP_mm_s':new['phases']['PREGRASP']['max_TCP_m_s']*1000,'before_closure_entry_mm_s':old['closure_entry']['actual_TCP_m_s']*1000,'after_closure_entry_mm_s':new['closure_entry']['actual_TCP_m_s']*1000,'PASS':m['physical_PASS_30deg']},indent=2))
