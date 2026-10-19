"""Run on server after experiments stop; summarize sensor traces and saved evaluation."""
import json,sys,shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
reports=[]
for folder in sorted(root.glob('**/report.json')):
 p=folder.parent
 if not (p/'observations.json').exists():continue
 r=json.loads(folder.read_text());rows=json.loads((p/'observations.json').read_text())
 name=p.name;dest=out/name;dest.mkdir(exist_ok=True)
 for filename in ['report.json','evaluation_only.json','estimated_articulation.json','structured_memory.json','estimated_articulation.urdf','robot_control_check_initial.json','robot_control_check_grasp.json','frozen_baseline.json','frozen_proxy.json','posthoc_contact_normal_evaluation.json','posthoc_contact_normal_trajectory.json','compliance_transition.json']:
  if (p/filename).exists():shutil.copy2(p/filename,dest/filename)
 fields=['t','phase','q','T_tcp','forces_n','aperture_m','margin_rad','tactile_patch_drift_m','relative_translation_slip_m','estimated_angle_deg','articulation_consistency_error_m','constrained_drive']
 trace=[{k:s.get(k) for k in fields} for i,s in enumerate(rows) if i%8==0 or i==len(rows)-1]
 (dest/'sensor_trace_30hz.json').write_text(json.dumps(trace))
 active=[s for s in rows if s['phase'] in ['COMPLIANT_SETTLE','EXPLORATORY','PROBE_HOLD','ESTIMATED_FOLLOW','FINAL_HOLD','ZERO_PROBE']]
 if active:
  E0=np.array(active[0]['T_tcp']);points=np.array([s['T_tcp'] for s in active]);loads=np.array([list(s['forces_n'].values()) for s in active]);t=np.array([s['t']-active[0]['t'] for s in active])
  metrics={'observed_motion_max_m':float(np.linalg.norm(points[:,:3,3]-E0[:3,3],axis=1).max()),'minimum_interaction_margin_rad':min(s['margin_rad'] for s in active),'max_tactile_patch_drift_m':max(s.get('tactile_patch_drift_m',0) for s in active),'peak_total_grasp_load_n':float(loads.sum(axis=1).max()),'peak_per_finger_load_n':float(loads.max()),'interaction_duration_s':float(t[-1])}
  fig,axes=plt.subplots(2,2,figsize=(12,8));d=(points[:,:3,3]-E0[:3,3])*1000
  for k,n in enumerate('xyz'):axes[0,0].plot(t,d[:,k],label=n)
  axes[0,0].set(ylabel='Measured EE displacement (mm)',xlabel='Interaction time (s)');axes[0,0].legend()
  axes[0,1].plot(t,loads[:,0],label='finger 1');axes[0,1].plot(t,loads[:,1],label='finger 2');axes[0,1].axhline(2,color='r',ls='--',label='existing load limit');axes[0,1].set(ylabel='Contact load (N)',xlabel='Interaction time (s)');axes[0,1].legend()
  axes[1,0].plot(t,[s.get('tactile_patch_drift_m',0)*1000 for s in active],label='Observed contact-plane drift');axes[1,0].axhline(1,color='r',ls='--');axes[1,0].set(ylabel='Observed surface drift (mm)',xlabel='Interaction time (s)');axes[1,0].legend()
  axes[1,1].plot(t,[s['margin_rad'] for s in active]);axes[1,1].axhline(.05,color='r',ls='--');axes[1,1].set(ylabel='Minimum joint margin (rad)',xlabel='Interaction time (s)')
  fig.suptitle(name+' | '+r['status']);fig.tight_layout();fig.savefig(dest/'measured_probe.png',dpi=150);plt.close(fig)
 else:metrics={}
 (dest/'measured_metrics.json').write_text(json.dumps(metrics,indent=2))
 reports.append({'name':name,'source':str(p),'status':r['status'],'success':r['success'],'report':r,'metrics':metrics})
 del rows
(out/'verified_summary.json').write_text(json.dumps({'runs':reports,'success_count':sum(r['success'] for r in reports),'active_trial_count':sum(not r['report']['no_operation'] for r in reports),'control_count':sum(r['report']['no_operation'] for r in reports),'same_initial_state_repetitions_not_generalization':True,'no_GT_online':True},indent=2))
print(json.dumps([{'name':r['name'],'status':r['status'],**r['metrics']} for r in reports],indent=2))
