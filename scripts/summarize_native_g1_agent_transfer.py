"""Failure-stage-aware passive evaluation; never imported by physical actors."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from analyze_native_g1_visual_contact_mismatch import records,summarize
from summarize_native_g1_multiview_skill import analyze
ROOT=Path(__file__).resolve().parents[1]
def load(p,default=None):return json.loads(Path(p).read_text()) if Path(p).exists() else default
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):Path(p).write_text(json.dumps(v,indent=2))
def evaluate(case):
    process=load(case/'PROCESS_EXIT.json')
    if process is None:raise SystemExit('Passive evaluator requires actor exit')
    run=case/'episode';out=case/'analysis';out.mkdir(exist_ok=True)
    report=load(run/'report.json',{});trace=load(run/'AGENT_ACTION_TRACE.json',[]);observations=load(run/'AGENT_OBSERVATIONS.json',[]);build=load(run/'AGENT_GEOMETRY_PLAN_RESULT.json',{})
    active=load(run/'AGENT_EXECUTABLE_HASH.json',{});checks={p:sha(p)==v for p,v in active.get('sources',{}).items()}
    inspected={i['path']:sha(run/i['path'])==i['sha256'] for t in trace for i in t['command'].get('inspected_files',[])}
    frozen=load('/tmp/wrist_baseline_frozen.json',{}).get('sources',{});frozen_checks={p:sha(p)==v for p,v in frozen.items()}
    physics=records(run,'physics_steps');obs=records(run,'observations');events=load(run/'events.json',[]);events=events if isinstance(events,list) else events.get('events',[])
    acquisition=report.get('grasp_acquisition');opening=next((e for e in events if e.get('event')=='SHORT_OPENING_START'),None)
    gain=report.get('max_actual_angle_after_valid_grasp_deg');gain=gain-acquisition['angle_deg'] if acquisition and gain is not None else None
    metrics={}
    if physics:
        analyze(run,out/'telemetry');metrics=load(out/'telemetry/METRICS.json',{});metrics.pop('report',None)
        metrics['peak_recorded_raw_pad_n']={n:max((float(row.get('telemetry',{}).get('pad_load_raw_n',{}).get(n,0)) for row in obs),default=None) for n in ['gripper_link1','gripper_link2']}
        metrics['peak_recorded_filtered_pad_n']={n:max((float(row['telemetry']['pad_load_filtered_n'][i]) for row in obs if row.get('telemetry',{}).get('pad_load_filtered_n') is not None),default=None) for i,n in enumerate(['left','right'])}
        tt=np.asarray([row['t'] for row in physics]);pp=np.asarray([row['T_tcp'] for row in physics]);speed=np.r_[0,np.linalg.norm(np.diff(pp[:,:3,3],axis=0),axis=1)/np.diff(tt)]
        positive=np.asarray([any(c.get('allowed_pad_target') and c.get('force_n',0)>0 for c in row.get('contacts',[])) for row in physics])
        metrics['max_TCP_with_positive_allowed_normal_contact_mm_s']=float(speed[positive].max()*1000) if positive.any() else None
        metrics['positive_contact_speed_is_passive_diagnostic_not_new_threshold']=True
    stages=dict(initial_view_route=bool(load(run/'INITIAL_VIEW_PLAN.json',{}).get('route')),fresh_wrist_observations=len(observations),accepted_wrist_views=sum(r['accepted_for_geometry'] for r in observations),fused_measured_geometry=bool(build.get('geometry')),automatic_grasp_plan=bool(build.get('plan')),execution_approved=(run/'AGENT_EXECUTION_APPROVED.json').exists(),physical_grasp_attempted=bool(records(run,'closure_commands')),bilateral_acquisition=acquisition is not None,verified_hold_s=opening['t']-acquisition['t'] if opening and acquisition else None,post_acquisition_opening_gain_deg=gain,short_transmission_at_least_2deg=gain is not None and gain>=2)
    stage='SHORT_TRANSMISSION_SUCCESS' if stages['short_transmission_at_least_2deg'] else ('OPENING' if opening else 'GRASP' if stages['physical_grasp_attempted'] else 'EXECUTION_RETURN_OR_APPROACH' if stages['execution_approved'] else 'GRASP_PLANNING' if stages['fused_measured_geometry'] else 'VISUAL_OBSERVATION' if observations else 'INITIAL_VIEW_REACHABILITY')
    compact=[]
    for t in trace:
        result=t['result'];brief={k:result[k] for k in ['feasible','observation','geometry','plan','planning_wall_s','geometry_wall_s','reason','completed','error','valid','approved','t_sim'] if k in result};compact.append(dict(sequence=t['sequence'],command=t['command'],actual_result=brief,agent_wait_wall_s=t['agent_wait_wall_s'],operation_wall_s=t['operation_wall_s']))
    packets=[load(Path(o['folder'])/'INSPECTION.json',{}) for o in observations]
    result=dict(asset=case.name,reference='34607b4',classification='CODEX_AGENT_WRIST / RGBD_SAM3 / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION',stages=stages,result_stage=stage,status=report.get('status'),geometry_plan_reason=build.get('reason'),report=report,metrics=metrics,
        observations=[dict(index=o['index'],accepted=o['accepted_for_geometry'],t_sim=o['t_sim'],cue=p.get('highest_cue'),camera_pose=p.get('camera',{}).get('T_world_camera_optical'),robot_q=p.get('camera',{}).get('robot_q'),FK_camera_translation_error_m=p.get('camera',{}).get('FK_camera_translation_error_m'),geometry_issue=o.get('geometry_issue')) for o,p in zip(observations,packets)],
        agent_selected_adjustments=load(run/'AGENT_EPISODE_RESULT.json',{}).get('agent_adjustments'),
        time=dict(simulation_s=report.get('simulation_s'),actor_wall_s=process['end_unix_s']-process['start_unix_s'],Agent_wait_wall_s=sum(t['agent_wait_wall_s'] for t in trace),sensing_wall_s=sum(o['sensing_and_segmentation_wall_s'] for o in observations),geometry_wall_s=build.get('geometry_wall_s'),grasp_planning_wall_s=build.get('planning_wall_s'),motion_wall_s=sum(r['motion_wall_s'] for r in load(run/'WRIST_ROBOT_TRAJECTORIES.json',[])),Agent_wait_includes_transfer_tools_and_scheduling=True),
        warning_or_stop_events=[e for e in events if any(s in e.get('event','') for s in ['WARNING','STOP','OVERLOAD','COLLISION','SPEED'])],telemetry_counts=dict(physics_steps=len(physics),observations=len(obs),closure_commands=len(records(run,'closure_commands'))),
        evidence_integrity=dict(active_sources=checks,frozen_baseline=frozen_checks,inspected_images=inspected,all_match=all(checks.values()) and all(frozen_checks.values()) and all(inspected.values())),
        known_dependencies=['common registered station','coarse cabinet envelope for initial survey','hinge and dynamic state','scene collision safety models','ideal simulated camera/robot calibration','approximate actuators'],GT_handle_grasp_fallback=False,physical_retries=0,raw_evidence_private=True)
    save(case/'COMPACT_RESULT.json',result);save(case/'AGENT_TRACE.json',compact)
    job=load(case/'job.json');config_paths=[ROOT/'configs/xarm7_wrist_rgbd_sim.json',Path(job['robust_uncertainty_config']),ROOT/'results/xarm7_native_gripper_20261012/NATIVE_CONFIG.json']
    save(case/'PROVENANCE.json',dict(actor_sources=active,config_sha256={str(p):sha(p) for p in config_paths},job_sha256=sha(case/'job.json'),evidence_integrity=result['evidence_integrity']))
    save(case/'OWNED_PROCESS_CLEANUP.json',dict(pid=process['pid'],process_exists=Path('/proc/'+str(process['pid'])).exists(),returncode=process['returncode'],export_complete=(run/'EXPORT_COMPLETE.json').exists(),other_processes_touched=False))
    print(json.dumps(dict(asset=case.name,stages=stages,result_stage=stage,status=result['status'],integrity=result['evidence_integrity']['all_match']),indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);evaluate(p.parse_args().case.resolve())
