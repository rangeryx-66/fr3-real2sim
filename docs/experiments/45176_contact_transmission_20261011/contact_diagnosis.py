"""Post-run pose/contact geometry diagnostics; no controller imports or gates."""
from pathlib import Path
import sys,json,gzip,pickle,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from piper_mobile_demo.model import Model
from articulated_demo.kinematics import transform
out=ROOT/'results/45176_contact_transmission_20261011';trial=out/'physical_01';job=json.loads((out/'job.json').read_text());model=Model(ROOT/'config/piper.urdf',job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()));rows=[]
for f in sorted((trial/'observations_chunks').glob('*.gz')):rows.extend(pickle.load(gzip.open(f,'rb')))
grasp=json.loads((trial/'PHYSICAL_METRICS.json').read_text())['grasp_event'];loaded=[r for r in rows if grasp and r['t']>=grasp['t']];first_open=next(i for i,r in enumerate(rows) if r['phase']=='FIXED_BASE_PINK_OPEN');handover=rows[first_open-1];bias=np.array(handover['command_q_arm'])-np.array(handover['q'][:6]);base=rows[0]['base'];B=transform(base[:3],[0,0,np.deg2rad(base[3])]);errors=[];rotation_errors=[];intervals=[[],[]];start=[None,None];examples={}
for r in loaded:
 if r['phase']=='FIXED_BASE_PINK_OPEN':
  predicted=B@model.fk(np.array(r['command_q_arm'])-bias);actual=np.array(r['T_tcp']);errors.append(float(np.linalg.norm(predicted[:3,3]-actual[:3,3])));rotation_errors.append(float(np.rad2deg(Rotation.from_matrix(predicted[:3,:3]@actual[:3,:3].T).magnitude())))
 for i,finger in enumerate(['gripper_link1','gripper_link2']):
  absent=r['forces_n'][finger]==0
  if absent and start[i] is None:start[i]=r
  if not absent and start[i] is not None:
   a=start[i];intervals[i].append({'start_s':a['t'],'end_s':r['t'],'duration_s':r['t']-a['t'],'start_angle_deg':a['door_angle_deg'],'start_slip_m':a['relative_translation_slip_m']});start[i]=None
for i,s in enumerate(start):
 if s:intervals[i].append({'start_s':s['t'],'end_s':loaded[-1]['t']+1/240,'duration_s':loaded[-1]['t']+1/240-s['t'],'start_angle_deg':s['door_angle_deg'],'start_slip_m':s['relative_translation_slip_m']})
first_zero=next(r for r in loaded if r['forces_n']['gripper_link1']==0);maximum=max(loaded,key=lambda r:r['door_angle_deg'])
for label,r in [('valid_grasp',loaded[0]),('first_left_zero',first_zero),('maximum_angle',maximum),('terminal',loaded[-1])]:
 points=[]
 for c in r['contacts']:
  if not c.get('allowed_pad_target') or c['force_n']<=0:continue
  P=np.array(r['finger_world_poses'][c['finger']]);point=(np.linalg.inv(P)@np.r_[c['contact_point_world_m'],1])[:3];n=np.array(c['native_normal_world']);angle=float(np.rad2deg(np.arccos(np.clip(abs(n@P[:3,2]),0,1))))
  points.append({'finger':c['finger'],'load_n':c['force_n'],'pad_local_point_m':point.tolist(),'pad_bounds_m':model.pads[c['finger']].tolist(),'absolute_normal_vs_pad_plane_angle_deg':angle,'native_normal_world':c['native_normal_world'],'collider0':c['collider0'],'collider1':c['collider1'],'point_world_m':c['contact_point_world_m'],'normal_polarity_not_reinterpreted':True})
 examples[label]={'t':r['t'],'angle_deg':r['door_angle_deg'],'position_slip_m':r['relative_translation_slip_m'],'rotation_slip_deg':r['relative_rotation_slip_deg'],'contacts':points}
result={'classification':'POSTRUN_DIAGNOSTIC_ONLY','command_tracking':{'method':'FK of commanded joints minus retained opening-handover equilibrium offset, compared with actual TCP; not a reconstructed object wrench','offset_rad':bias.tolist(),'max_position_error_m':max(errors),'p95_position_error_m':float(np.percentile(errors,95)),'max_orientation_error_deg':max(rotation_errors)},'pad_zero_report_intervals':intervals,'longest_left_zero_interval':max(intervals[0],key=lambda x:x['duration_s']),'first_left_zero_interval_over_one_second_for_description_only':next((x for x in intervals[0] if x['duration_s']>=1),None),'examples':examples,'maximum_postgrasp_gain_deg':maximum['door_angle_deg']-grasp['angle_deg'],'genuine_mechanical_limit_not_established':True}
(trial/'CONTACT_GEOMETRY_DIAGNOSIS.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
