"""Saved-evidence paired outcome/telemetry summary. No execution imports."""
from pathlib import Path
import collections,gzip,hashlib,json,pickle,subprocess
import numpy as np
ROOT=Path(__file__).resolve().parents[1];B=ROOT/'results/native_g1_visual_input_20261014';O=ROOT/'results/native_g1_robust_rank_20261013';cases=[]

def stream(folder,name):
 index=folder/(name+'_chunks/index.json');rows=[];errors=[]
 if not index.exists():return rows,dict(records=0,indexed=None,error='index unavailable')
 meta=json.loads(index.read_text())
 for chunk in meta['chunks']:
  try:
   data=pickle.load(gzip.open(folder/(name+'_chunks')/chunk['file'],'rb'));rows.extend(data)
   if len(data)!=chunk['records']:errors.append('chunk count mismatch: '+chunk['file'])
  except Exception as e:errors.append(repr(e))
 return rows,dict(records=len(rows),indexed=meta['records'],errors=errors)

for asset in ['45671','45638','45600','45606']:
 p=B/asset/'physical_01';r=json.loads((p/'report.json').read_text()) if (p/'report.json').exists() else {};oracle=json.loads((O/asset/'CASE_SUMMARY.json').read_text())
 row=dict(asset_id=asset,paired_development_case=True,status=r.get('status','NOT_EXECUTED'),visual_grasp=r.get('initial_grasp'),actual_gain_after_acquisition_deg=None,short_transmission_ge2deg=False,oracle_gain_deg=oracle.get('actual_gain_after_acquisition_deg'),oracle_grasp=oracle.get('initial_grasp'),simulation_s=r.get('simulation_s'),wall_s=r.get('wall_s'),minimum_joint_margin_rad=r.get('minimum_joint_margin_rad'),peak_raw_pad_load_n=r.get('peak_pad_load_n'),first_divergence=r.get('first_divergence'),physical_scene_root=str(p),GT_grasp_fallback=r.get('GT_grasp_fallback'),remaining_known_dependencies=r.get('GT_dependencies'))
 if not r:cases.append(row);continue
 obs,oi=stream(p,'observations');phy,pi=stream(p,'physics_steps');close,ci=stream(p,'closure_commands');row['streams']=dict(observations=oi,physics_steps=pi,closure_commands=ci)
 event=r.get('grasp_acquisition');valid=[v for v in obs if event and v['t']>=event['t']]
 if valid:
  row['actual_gain_after_acquisition_deg']=max(v['door_angle_deg'] for v in valid)-event['angle_deg'];row['short_transmission_ge2deg']=row['actual_gain_after_acquisition_deg']>=2.;row['max_relative_shift_mm']=max(v['relative_translation_slip_m'] for v in valid)*1000;row['max_relative_rotation_deg']=max(v['relative_rotation_slip_deg'] for v in valid)
 if len(obs)>1:
  velocity=np.linalg.norm(np.diff(np.asarray([v['T_tcp'] for v in obs])[:,:3,3],axis=0),axis=1)/np.diff([v['t'] for v in obs])*1000;row['max_all_phase_TCP_mm_s']=float(velocity.max());row['max_postgrasp_TCP_mm_s']=float(velocity[np.asarray([v['t'] for v in obs[1:]])>=event['t']].max()) if event else None;row['TCP_speed_definition']='unfiltered measured post-step TCP position differences / recorded dt'
  peak=int(np.argmax(velocity))+1;row['all_phase_TCP_peak_context']=dict(t=obs[peak]['t'],phase=obs[peak]['phase'],raw_loads_n=obs[peak]['forces_n'],after_grasp=bool(event and obs[peak]['t']>=event['t']))
 phase=collections.Counter(v['phase'] for v in obs);row['phase_simulation_s']={k:v/240 for k,v in phase.items()};row['physical_motion_started']=bool(obs);row['closure_attempt_started']=bool(close);row['hold_s']=sum(v for k,v in row['phase_simulation_s'].items() if k in ['FORCE_HOLD','VALIDATED_PREFIX_EXTRA_HOLD']) if event else None
 force_events=[]
 for v in obs:
  f=v.get('force_event',{});level=f.get('status') if isinstance(f,dict) else None
  if level and level not in ['NORMAL','OK']:force_events.append(dict(t=v['t'],phase=v['phase'],event=f,raw_loads=v['forces_n']))
 row['force_events']=force_events;row['force_event_levels']=dict(collections.Counter(x['event'].get('status') for x in force_events));row['raw_peak_record']=max((dict(t=v['t'],phase=v['phase'],loads_n=v['forces_n'],force_event=v.get('force_event')) for v in obs),key=lambda v:max(v['loads_n'].values()),default=None)
 contacts=collections.defaultdict(list)
 for v in phy:
  labels={c['finger'] for c in v.get('contacts',[]) if c.get('allowed_pad_target') and c.get('force_n',0)>0};contacts[v.get('phase','UNKNOWN')].append(['gripper_link1' in labels,'gripper_link2' in labels])
 row['phase_native_positive_contact_fraction']={k:dict(left=float(np.asarray(v)[:,0].mean()),right=float(np.asarray(v)[:,1].mean()),bilateral=float(np.asarray(v).all(1).mean())) for k,v in contacts.items() if v}
 row['postgrasp_native_zero_load_intervals']={}
 if event:
  post=[v for v in phy if v['t']>=event['t']]
  for finger in ['gripper_link1','gripper_link2']:
   intervals=[];start=None;end=None
   for v in post:
    positive=any(c['finger']==finger and c.get('allowed_pad_target') and c.get('force_n',0)>0 for c in v.get('contacts',[]))
    if not positive and start is None:start=v['t']
    if positive and start is not None:intervals.append([start,v['t']]);start=None
    end=v['t']
   if start is not None and end is not None:intervals.append([start,end+1/240])
   row['postgrasp_native_zero_load_intervals'][finger]=dict(intervals_s=intervals,total_s=sum(b-a for a,b in intervals),longest_s=max((b-a for a,b in intervals),default=0),definition='no positive allowed pad normal contact; not an added grasp-loss gate')
 for name in ['visual_plan.json','visual_correction.json','visual_adapter_provenance.json','VISUAL_EXECUTABLE_HASH.json']:
  if (p/name).exists():
   data=json.loads((p/name).read_text());row[name]=data if name!='visual_plan.json' else dict(planned_range_deg=float(np.rad2deg(data['planned_reached_state'])),base=data['base'],T=data['T'],selected_parameters={k:data[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']},scores={k:data['contact_geometry_audit']['robust_contact'][k] for k in ['acquisition_score','retention_score','task_score','robust_combined_score','chosen_expected_aperture_m']})
 row['raw_telemetry_is_normal_only_not_full_wrench']=True;row['telemetry_issues']=r.get('telemetry_issues')
 if obs:
  ffmpeg='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/ffmpeg';log=(p/'VIDEO_DECODE.log').open('w');check=subprocess.run([ffmpeg,'-v','error','-i',str(p/'contact_baseline.mp4'),'-f','null','-'],stdout=subprocess.DEVNULL,stderr=log);log.close();row['video_decode_rc']=check.returncode;row['video_decode_error_bytes']=(p/'VIDEO_DECODE.log').stat().st_size
 hashes={str(f.relative_to(p)):hashlib.sha256(f.read_bytes()).hexdigest() for f in p.rglob('*') if f.is_file() and f.name!='PRIVATE_EVIDENCE_HASHES.json'};(p/'PRIVATE_EVIDENCE_HASHES.json').write_text(json.dumps(hashes,indent=2));row['evidence_hash_index_sha256']=hashlib.sha256((p/'PRIVATE_EVIDENCE_HASHES.json').read_bytes()).hexdigest();cases.append(row)
 print(asset,row['visual_grasp'],row['actual_gain_after_acquisition_deg'],row['status'],flush=True)
summary=dict(reference='30f5b30',classification='RGBD_SAM3_HANDLE / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION',cases=cases,physical_episode_entries=len([r for r in cases if r['status']!='NOT_EXECUTED']),motion_episodes=sum(r.get('physical_motion_started',False) for r in cases),grasp_successes=sum(r['visual_grasp'] is True for r in cases),short_transmission_successes=sum(r['short_transmission_ge2deg'] for r in cases),paired_development_not_held_out=True,original_30deg_benchmark_unchanged='4/10',between_case_tuning=False)
(B/'PAIRED_RESULTS.json').write_text(json.dumps(summary,indent=2))
