"""Offline continuity diagnostics. Never imported by the physical actor."""
from pathlib import Path
import json,gzip,pickle,numpy as np,sys
root=Path(sys.argv[1])
for p in root.glob('*/development_*'):
 if not (p/'report.json').exists():continue
 previous=None;max_cmd=0.;max_ref=0.;max_lag=0.;first=None;loaded=0.;angles=[];grasp=json.loads((p/'acquired_grasp_reference.json').read_text()) if (p/'acquired_grasp_reference.json').exists() else None
 for f in sorted((p/'observations_chunks').glob('*.gz')):
  for r in pickle.load(gzip.open(f,'rb')):
   if previous is not None and r['phase']=='FIXED_BASE_PINK_OPEN':
    dt=r['t']-previous['t'];dq=float(np.max(abs(np.array(r['command_q_arm'])-previous['command_q_arm'])));max_cmd=max(max_cmd,dq)
    if r['planned_T_tcp'] is not None and previous['planned_T_tcp'] is not None:
     refv=float(np.linalg.norm(np.array(r['planned_T_tcp'])[:3,3]-np.array(previous['planned_T_tcp'])[:3,3])/dt);max_ref=max(max_ref,refv)
    else:refv=None
    v=float(np.linalg.norm(np.array(r['T_tcp'])[:3,3]-np.array(previous['T_tcp'])[:3,3])/dt)
    max_lag=max(max_lag,float(abs(r['door_angle_deg']-np.rad2deg(r['planned_articulation_state']))))
    if first is None and v>.005:first={'t':r['t'],'actual_TCP_speed_m_s':v,'reference_speed_m_s':refv,'maximum_joint_command_increment_rad':dq,'pad_loads_n':r['forces_n']}
    angles.append(r['door_angle_deg'])
   previous=r
 data={'classification':'DIAGNOSTIC_ONLY_OFFLINE','acquired_T_body_tcp':grasp,'maximum_opening_joint_command_increment_rad':max_cmd,'maximum_reference_translation_speed_m_s':max_ref,'maximum_reference_object_lag_deg':max_lag,'first_actual_TCP_speed_spike':first,'does_not_govern_execution':True}
 (p/'MOTION_CONTINUITY.json').write_text(json.dumps(data,indent=2))
