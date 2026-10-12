import json,pickle,gzip,collections,hashlib,subprocess
from pathlib import Path
import numpy as np
from xarm7_native_gripper import ROOT,OUT,configuration

def read_chunks(folder):
 for f in sorted(folder.glob('*.gz')):
  for row in pickle.load(gzip.open(f,'rb')):yield row

def analyze(asset):
 d=OUT/asset/'physical_01';r=json.loads((d/'report.json').read_text());rows=list(read_chunks(d/'observations_chunks'));dt=1/240;c=configuration();names=json.loads((d/'joint_sensor_index_metadata.json').read_text())['dof_names'];arm=[names.index(n) for n in c['arm_joint_names']];acq=r.get('grasp_acquisition');start=acq['t'] if acq else float('inf');active=[x for x in rows if x['t']>=start];speeds=[np.linalg.norm(np.array(b['T_tcp'])[:3,3]-np.array(a['T_tcp'])[:3,3])/dt for a,b in zip(rows,rows[1:])];loaded=[v for v,row in zip(speeds,rows[1:]) if row['t']>=start]
 phase=collections.Counter(x['phase'] for x in rows);counts=collections.Counter();contacts=[];pairs=collections.Counter();gap_events=[];last=None;loss_start=None;native_feedback=[];joint_tracking=[];native_vel=[];fd_vel=[];qprev=None
 for x in rows:
  t=x.get('telemetry',{});counts.update(k for k,v in t.items() if v is not None)
  if t.get('native_gripper_feedback'):native_feedback.append(t['native_gripper_feedback'])
  if 'xarm_actuator' in x:joint_tracking.append(np.max(abs(np.asarray(x['q'])[arm]-np.asarray(x['xarm_actuator']['reference']))))
  if qprev is not None:fd_vel.append(((np.array(x['q'])[arm]-qprev)/dt).tolist())
  qprev=np.asarray(x['q'])[arm]
  if t.get('joint_velocities') is not None:native_vel.append(np.asarray(t['joint_velocities'])[arm].tolist())
  if x['t']>=start:
   present=max(t.get('pad_load_filtered_n',[1,1]))>=.05
   if not present and loss_start is None:loss_start=x['t']
   if present and loss_start is not None:gap_events.append({'start_s':loss_start,'end_s':x['t'],'duration_s':x['t']-loss_start});loss_start=None
  for k in x['contacts']:
   pairs[(k['collider'],k['target'])]+=1
   if k['allowed_pad_target'] and k.get('contact_point_world_m'):
    P=np.asarray(x['finger_world_poses'].get(k['finger'],np.eye(4)));p=(np.linalg.inv(P)@np.r_[k['contact_point_world_m'],1])[:3];contacts.append({'finger':k['finger'],'local_point_m':p.tolist(),'point_world_m':k['contact_point_world_m'],'normal_world':k.get('contact_normal_world'),'impulse_world_ns':k['impulse_world_ns'],'force_n':k['force_n'],'t':x['t'],'collider':k['collider'],'target':k['target']})
 if loss_start is not None:gap_events.append({'start_s':loss_start,'end_s':rows[-1]['t'],'duration_s':rows[-1]['t']-loss_start+dt})
 ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffprobe';video=json.loads(subprocess.check_output([ff,'-v','error','-select_streams','v:0','-show_entries','stream=nb_frames,width,height,duration','-of','json',str(d/'contact_baseline.mp4')]))
 analysis={'asset_id':asset,'actual_physical_grasp':bool(acq),'max_angle_after_valid_grasp_deg':r.get('max_actual_angle_after_valid_grasp_deg'),'max_relative_slip_mm':max((x['relative_translation_slip_m']*1000 for x in active),default=None),'max_relative_rotation_deg':max((x['relative_rotation_slip_deg'] for x in active),default=None),'max_all_phase_TCP_mm_s':max(speeds,default=0)*1000,'max_loaded_TCP_mm_s':max(loaded,default=None)*1000 if loaded else None,'minimum_margin_rad':r['minimum_joint_margin_rad'],'max_arm_reference_tracking_error_rad':max(joint_tracking,default=None),'max_native_velocity_rad_s':np.abs(native_vel).max(0).tolist() if native_vel else None,'max_fullstep_velocity_rad_s':np.abs(fd_vel).max(0).tolist() if fd_vel else None,'peak_raw_pad_load_n':r['peak_pad_load_n'],'sim_s':r['simulation_s'],'wall_s':r['wall_s'],'phase_sim_s':{k:v*dt for k,v in phase.items()},'both_pad_filtered_contact_loss_intervals':gap_events,'native_collider_pairs':[{'collider':a,'target':b,'samples':v} for (a,b),v in pairs.items()],'native_gripper_mimic_error_max_rad':max((max(abs(v) for v in x['mimic_error_rad'].values()) for x in native_feedback),default=None),'native_position_hold':True,'hardware_pad_feedback':False,'telemetry_counts':dict(counts),'observation_records':len(rows),'raw_physics_records':json.loads((d/'physics_steps_chunks/index.json').read_text())['records'],'video':video,'native_force_note':'Actual raw PhysX contact, not native hardware tactile feedback','position_preload_note':'Legacy clamp request converted to a latched finite implicit position offset using approximate gripper stiffness and aperture Jacobian; no external jaw torque applied','status':r['status'],'success':r['success']}
 (d/'NATIVE_ANALYSIS.json').write_text(json.dumps(analysis,indent=2));folder=d/'contact_points_chunks';folder.mkdir(exist_ok=True)
 for i in range(0,len(contacts),1024):
  with gzip.open(folder/('%08d.pkl.gz'%i),'wb',compresslevel=1) as f:pickle.dump(contacts[i:i+1024],f,protocol=5)
 print(asset,json.dumps({k:analysis[k] for k in ['success','status','max_angle_after_valid_grasp_deg','max_relative_slip_mm','max_loaded_TCP_mm_s','observation_records']}))
if __name__=='__main__':
 import sys
 analyze(sys.argv[1])
