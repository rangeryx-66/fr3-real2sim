"""Passive completed-episode evidence summary; never imported by the actor."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from summarize_native_g1_multiview_skill import analyze
from analyze_native_g1_visual_contact_mismatch import records
ROOT=Path(__file__).resolve().parents[1]

def load(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,d):Path(p).write_text(json.dumps(d,indent=2))

def supplement(result,run,trace,events):
    from scipy.spatial.transform import Rotation
    result['time']['actuator_return_and_measured_handoff_wall_s']=next(t['operation_wall_s'] for t in trace if t['command']['operation']=='execute')
    result['contact_continuity_summary']={k:dict(absence_intervals=len(v),maximum_normal_report_absence_s=max((i['duration_s'] for i in v),default=0),absence_is_not_proof_of_grasp_loss=True) for k,v in result['metrics']['positive_normal_absence_after_acquisition'].items()}
    result['safety_warning_or_stop_events']=[e for e in events if any(s in e.get('event','') for s in ['WARNING','STOP','OVERLOAD','COLLISION','SPEED'])]
    correction=load(run/'visual_correction.json');D=np.asarray(correction['D_visual'])
    result['pregrasp_refinement']=dict(source=correction['source'],common_points=correction['common_points'],translation_mm=float(np.linalg.norm(D[:3,3])*1000),rotation_deg=float(Rotation.from_matrix(D[:3,:3]).magnitude()*180/np.pi),GT_pose_correction=False)
    phase=lambda name:next(e['t'] for e in events if e.get('event')=='PHASE_CHANGE' and e.get('to')==name)
    observation_end=result['observations'][-1]['t_sim'];returned=next(t['result']['t_sim'] for t in trace if t['command']['operation']=='execute')
    acq=result['report']['grasp_acquisition'];opening=next(e['t'] for e in events if e.get('event')=='SHORT_OPENING_START')
    pre=next(e['t'] for e in events if e.get('event')=='PHASE_CHANGE' and e.get('from')=='SETTLE' and e.get('to')=='PREGRASP')
    result['time']['phases_sim_s']=dict(wrist_observation_motion_and_settle=observation_end,actuator_return_and_settle=returned-observation_end,original_preapproach_settle=pre-returned,home_to_pregrasp=phase('APPROACH')-pre,approach=phase('SLOW_CLOSE')-phase('APPROACH'),closure_to_verified=acq['t']-phase('SLOW_CLOSE'),verified_hold=opening-acq['t'],short_opening=result['report']['simulation_s']-opening)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root.resolve();run=root/'episode';out=root/'analysis';out.mkdir(exist_ok=True)
    process=load(root/'PROCESS_EXIT.json');analyze(run,out/'telemetry');metrics=load(out/'telemetry/METRICS.json');report=metrics['report'];trace=load(run/'AGENT_ACTION_TRACE.json');observations=load(run/'AGENT_OBSERVATIONS.json');inspections={};compact=[]
    for entry in trace:
        command=entry['command'];result=entry['result'];brief={k:result[k] for k in ['feasible','observation','geometry','plan','planning_wall_s','geometry_wall_s','reason','completed','error','valid','approved','returned_with_actuators','measured_approach_revalidated','t_sim','next'] if k in result}
        if result.get('route'):brief['minimum_planned_margin_rad']=result['route']['minimum_joint_margin_rad']
        compact.append(dict(sequence=entry['sequence'],command=command,actual_result=brief,agent_wait_wall_s=entry['agent_wait_wall_s'],operation_wall_s=entry['operation_wall_s']))
        for image in command.get('inspected_files',[]):inspections[image['path']]=sha(run/image['path'])==image['sha256']
    frozen=load('/tmp/wrist_baseline_frozen.json')['sources'];baseline={p:sha(p)==s for p,s in frozen.items()};active=load(run/'AGENT_EXECUTABLE_HASH.json');sources={p:sha(p)==s for p,s in active['sources'].items()}
    events=load(run/'events.json');events=events if isinstance(events,list) else events.get('events',[])
    start=next((e for e in events if e.get('event')=='SHORT_OPENING_START'),None);acquisition=report.get('grasp_acquisition')
    # Timestamp-native hold duration, not a contact-symmetry criterion.
    hold_s=start['t']-acquisition['t'] if start and acquisition else None
    build=load(run/'AGENT_GEOMETRY_PLAN_RESULT.json') if (run/'AGENT_GEOMETRY_PLAN_RESULT.json').exists() else {}
    obs=[]
    for row in observations:
        packet=load(Path(row['folder'])/'INSPECTION.json');obs.append(dict(index=row['index'],accepted=row['accepted_for_geometry'],t_sim=row['t_sim'],cue=packet['highest_cue'],camera_pose=packet['camera']['T_world_camera_optical'],robot_q=packet['camera']['robot_q'],FK_camera_translation_error_mm=packet['camera']['FK_camera_translation_error_m']*1000))
    result=dict(reference='2c126cd',physical_baseline='dba947d',execution_source_commit='edd5bd0',classification='CODEX_AGENT_WRIST / RGBD_SAM3 / KNOWN_HINGE_STATE_COLLISION / APPROXIMATE_ACTUATOR_SIMULATION',one_development_episode_not_robustness=True,
        stages=dict(active_perception=bool(build.get('geometry')),automatic_plan=bool(build.get('plan')),explicit_execution_approved=(run/'AGENT_EXECUTION_APPROVED.json').exists(),bilateral_grasp_acquired=acquisition is not None,post_acquisition_hold_s=hold_s,post_acquisition_gain_deg=metrics['actual_gain_after_acquisition_deg'],short_transmission_at_least_2deg=metrics['actual_gain_after_acquisition_deg'] is not None and metrics['actual_gain_after_acquisition_deg']>=2),
        report=report,observations=obs,Agent_selected_adjustments=sum(t['command']['operation']=='move_and_observe' for t in trace),first_contacts=metrics['first_contacts'],metrics={k:v for k,v in metrics.items() if k!='report'},
        time=dict(simulation_s=report['simulation_s'],actor_wall_s=process['end_unix_s']-process['start_unix_s'],Agent_wait_wall_s=sum(t['agent_wait_wall_s'] for t in trace),sensing_wall_s=sum(o['sensing_and_segmentation_wall_s'] for o in observations),grasp_planning_wall_s=build.get('planning_wall_s'),motion_routes_wall_s=sum(r['motion_wall_s'] for r in load(run/'WRIST_ROBOT_TRAJECTORIES.json')),Agent_wait_includes_transfer_tools_and_scheduling=True),
        evidence_integrity=dict(active_source_checks=sources,frozen_baseline_checks=baseline,inspected_image_checks=inspections,all_match=all(sources.values()) and all(baseline.values()) and all(inspections.values())),
        remaining_dependencies=['registered station','coarse cabinet survey envelope','known hinge and dynamic state','ideal simulated wrist calibration','scene collision safety models','approximate actuators'],GT_handle_grasp_fallback=False,physical_retries=0,scene_stitching=False,raw_evidence_private=True)
    result=supplement(result,run,trace,events)
    save(root/'COMPACT_RESULT.json',result);save(root/'AGENT_TRACE.json',compact)
    configs=[ROOT/'configs/xarm7_wrist_rgbd_sim.json',Path(load(root/'job.json')['robust_uncertainty_config']),ROOT/'results/xarm7_native_gripper_20261012/NATIVE_CONFIG.json',ROOT/'results/xarm7_native_gripper_20261012/xarm7_native_gripper.urdf']
    save(root/'PROVENANCE.json',dict(reference='2c126cd',execution_source_commit='edd5bd0',actor_sources=active,config_hashes={str(p):sha(p) for p in configs},evidence_integrity=result['evidence_integrity']))
    save(root/'OWNED_PROCESS_CLEANUP.json',dict(pid=process['pid'],process_exists=Path('/proc/'+str(process['pid'])).exists(),returncode=process['returncode'],export_complete=(run/'EXPORT_COMPLETE.json').exists(),other_processes_touched=False))
    print(json.dumps(dict(stages=result['stages'],status=report['status'],integrity=result['evidence_integrity']['all_match']),indent=2))
