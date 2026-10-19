"""Post-exit metrics only. No output is fed back to the experiment."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from summarize_native_g1_multiview_skill import analyze,intervals
from analyze_native_g1_visual_contact_mismatch import records

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root.resolve();run=root/'episode';out=root/'analysis';out.mkdir(exist_ok=True)
read=lambda p:json.loads(p.read_text())
assert (root/'PROCESS_EXIT.json').exists(),'Post-exit analysis only'
analyze(run,out/'telemetry');m=read(out/'telemetry/METRICS.json');r=m['report'];trace=read(run/'AGENT_ACTION_TRACE.json');obs=read(run/'AGENT_OBSERVATIONS.json');g=read(run/'AGENT_GEOMETRY_PLAN_RESULT.json') if (run/'AGENT_GEOMETRY_PLAN_RESULT.json').exists() else {}
events=read(run/'events.json');events=events if isinstance(events,list) else events.get('events',[])
acq=r.get('grasp_acquisition');start=next((e for e in events if e.get('event')=='SHORT_OPENING_START'),None)
physics=records(run,'physics_steps');O=records(run,'observations');t=np.asarray([x['t'] for x in physics]);P=np.asarray([x['T_tcp'] for x in physics]);speed=np.r_[0,np.linalg.norm(np.diff(P[:,:3,3],axis=0),axis=1)/np.diff(t)]*1000
loads=np.asarray([[sum(c.get('force_n',0) for c in x.get('contacts',[]) if c.get('finger')==f and c.get('allowed_pad_target')) for f in ['gripper_link1','gripper_link2']] for x in physics]);loaded=(loads>0).any(1)
filtered=np.asarray([list(x.get('forces_n',{}).values()) for x in O]);dt=float(read(run/'sensor_metadata.json')['physics_dt_s'])
net=float(r['final_actual_angle_deg']-acq['angle_deg']) if acq else None
hold=(start['t']-acq['t']) if acq and start else None
holdmask=(t>=acq['t'])&(t<start['t']) if hold is not None else np.zeros(len(t),bool)
result=dict(reference='3da07ab',frozen_physical_baseline='a47fe09',classification='INCREMENTAL_CODEX_WRIST / RGBD_SAM3 / KNOWN_HINGE_STATE_COLLISIONS / APPROXIMATE_ACTUATOR_SIMULATION',valid_physical_attempts=1,physical_retries=0,stages=dict(active_perception=g.get('geometry',False),automatic_plan=g.get('plan',False),execution_approved=(run/'AGENT_EXECUTION_APPROVED.json').exists(),bilateral_acquisition=acq is not None,hold_duration_s=hold,post_acquisition_max_gain_deg=m['actual_gain_after_acquisition_deg'],post_acquisition_net_gain_deg=net,short_outcome_at_least2deg=bool(acq and net is not None and net>=2)),report=r,metrics={k:v for k,v in m.items() if k!='report'},peak_allowed_normal_load_per_pad_n=loads.max(0).tolist(),peak_filtered_load_per_pad_n=filtered.max(0).tolist() if filtered.size else None,max_TCP_with_positive_allowed_normal_contact_mm_s=float(speed[loaded].max()) if loaded.any() else None,hold_normal_report_absence={f:intervals(t[holdmask],loads[holdmask,i]<=0,dt) for i,f in enumerate(['left','right'])},warning_or_stop_events=[e for e in events if any(s in e.get('event','') for s in ['WARNING','STOP','OVERLOAD','COLLISION','SPEED'])],first_divergence=r.get('first_divergence'),observations=obs,plan_geometry_wall_s=g.get('geometry_wall_s'),planning_wall_s=g.get('planning_wall_s'),actor_wall_s=read(root/'PROCESS_EXIT.json')['end_unix_s']-read(root/'PROCESS_EXIT.json')['start_unix_s'],agent_wait_wall_s=sum(x['agent_wait_wall_s'] for x in trace),sensing_wall_s=sum(x['sensing_and_segmentation_wall_s'] for x in obs),execution_handoff_wall_s=next((x['operation_wall_s'] for x in trace if x['command']['operation']=='execute'),None),raw_evidence_private=True,normal_forces_are_partial_simulator_measurements=True)
if (run/'INCREMENTAL_EXECUTION_INPUT_after.json').exists():
 before=read(run/'INCREMENTAL_EXECUTION_INPUT_before.json');after=read(run/'INCREMENTAL_EXECUTION_INPUT_after.json');result['execution_input_consistency']=dict(plan_hash_unchanged=before['selected_plan_sha256']==after['selected_plan_sha256'],geometry_hashes_unchanged=before['geometry_sha256']==after['geometry_sha256'],assigned_T_matches_selected=after['actual_T']==after['selected_T'],assigned_whole_T_matches_selected=after['whole_T']==after['selected_T'],contact_supported_mesh=after['contact_supported_mesh'],geometry_sha256=after['geometry_sha256'])
post=t>=acq['t'] if acq else np.zeros(len(t),bool)
result['normal_load_asymmetry']=dict(post_acquisition_mean_left_right_n=loads[post].mean(0).tolist() if post.any() else None,post_acquisition_max_abs_left_minus_right_n=float(abs(loads[post,0]-loads[post,1]).max()) if post.any() else None,hold_mean_left_right_n=loads[holdmask].mean(0).tolist() if holdmask.any() else None,diagnostic_only=True,no_symmetry_requirement=True)
seen_force_status=set()
for row in O:
 event=row.get('force_event') or {};status=event.get('status')
 if status and status!='NORMAL' and status not in seen_force_status:
  seen_force_status.add(status);result['warning_or_stop_events'].append(dict(event=status,t=row['t'],source='original observation.force_event',force_policy=event))
compact=[]
for x in trace:
 v=x['result'];brief={k:v[k] for k in ['feasible','observation','geometry','plan','valid','approved','reason','error','t_sim','returned_with_actuators','measured_approach_revalidated','geometry_wall_s','planning_wall_s'] if k in v}
 if v.get('route'):brief['minimum_planned_margin_rad']=v['route']['minimum_joint_margin_rad']
 compact.append(dict(sequence=x['sequence'],command=x['command'],actual_result=brief,agent_wait_wall_s=x['agent_wait_wall_s'],operation_wall_s=x['operation_wall_s']))
(root/'COMPACT_RESULT.json').write_text(json.dumps(result,indent=2));(root/'COMPACT_AGENT_TRACE.json').write_text(json.dumps(compact,indent=2));print(json.dumps(dict(stages=result['stages'],status=r['status'],max_loads=result['peak_allowed_normal_load_per_pad_n'],loaded_TCP=result['max_TCP_with_positive_allowed_normal_contact_mm_s']),indent=2))
