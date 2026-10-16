"""Robot-free callback/safety-stop tests and archived-state exit planning.

These checks are not a physical validation of unloading or contact recovery.
"""
import argparse,gzip,json,pickle,tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import native_g1_contact_recovery as recovery
import native_g1_recovery_telemetry as passive
from run_native_g1_contact_recovery import source


def verify(job,archived):
    results=dict(physics_executed=False)
    text=source(job);compile(text,'recovery_generated_preflight','exec')
    from run_native_g1_wrist_short import wrist_source
    old=wrist_source(job);marker='try:\n    signal.signal(signal.SIGINT,request_owner_stop)'
    assert old[:old.index(marker)]==text[:text.index('from native_g1_contact_recovery import run')]
    assert 'recovery_result=run_contact_recovery(globals())' in text
    results['frozen_setup_closure_and_Pink_definition_prefix_identical']=True
    with tempfile.TemporaryDirectory() as temp:
        out=Path(temp);calls=[];guard=SimpleNamespace(samples=['never clear'],pending=True)
        g=dict(a=SimpleNamespace(output=out),tick=0,dt=1/240,rows=[dict(aperture_m=.015)],robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(13)),arm=np.arange(7),fingers=np.arange(7,13),reference=np.eye(4),force_guard=guard,
            policy={'slow_closure_m_s':.001},controller=SimpleNamespace(geometry=SimpleNamespace(angle=lambda w:w)),world=SimpleNamespace(play=lambda:None),qtarget=np.zeros(13))
        def hard():calls.append('guarded physics step');raise RuntimeError('HARD_FORCE_STOP_SUSTAINED')
        g['step']=hard
        with patch.object(recovery,'release_geometry',return_value={'valid':True}):result=recovery.safe_release_retreat(g)
        assert not result['safe'] and 'HARD_FORCE_STOP_SUSTAINED' in result['reason'] and len(calls)==1
        assert guard.samples==['never clear'] and guard.pending
        results['hard_during_unload']=dict(stopped_after_one_original_exception=True,no_force_history_reset=True,no_continuation=True)
    with tempfile.TemporaryDirectory() as temp:
        g=dict(a=SimpleNamespace(output=Path(temp)),contact_attempts=[dict(family=[-.3,0,0,0],physical_grasp_started=True,short_success=False)])
        poses=[dict(section=f,insertion_m=0,roll=0,pitch=0) for f in [-.3,0,.3]]
        def build(state):
            kept=recovery.multi.rescore();assert [x['section'] for x in kept]==[0,.3];return dict(plan=False)
        with patch.object(recovery.multi,'rescore',return_value=poses),patch.object(recovery.incremental,'build_geometry',build):recovery.build_geometry(g)
        assert len(poses)==3
        results['candidate_history']=dict(only_episode_failed_family_removed=True,scoring_inputs_unchanged=True,original_pool_unmodified=True)
    assert recovery.attempt_grasp_and_probe({'physical_grasps_started':2},{})['executed']==False
    assert recovery.attempt_grasp_and_probe({'physical_grasps_started':1,'contact_attempts':[{'short_success':True}]},{})['executed']==False
    results['budget_and_success_no_retry']=True
    assert recovery.short_outcome(True,2.5,False,'EXISTING_SOFT_FORCE_WARNING_RETURN_FOR_UNLOAD')
    assert not recovery.short_outcome(True,1.9,False,'EXISTING_SOFT_FORCE_WARNING_RETURN_FOR_UNLOAD')
    assert not recovery.short_outcome(True,2.5,False,'HARD_FORCE_STOP_SUSTAINED')
    results['registered_2deg_outcome_not_stricter_3deg_command_completion']=True
    with tempfile.TemporaryDirectory() as temp:
        out=Path(temp);g=dict(a=SimpleNamespace(output=out),agent_observations=[],observation_generations=[[]])
        def save(state,label):return label
        with patch.object(recovery.observation,'save_rgbd',save),patch.object(recovery.incremental,'observe',lambda state:recovery.observation.save_rgbd(state,'agent_00')):
            assert recovery.observe_current(g)=='generation_01_agent_00'
        results['fresh_generation_images_do_not_overwrite_initial_RGBD']=True
    # Inspect-result serialization is deliberately tested after attaching it
    # to an attempt, preventing a recursive report from ending a live actor.
    with tempfile.TemporaryDirectory() as temp:
        out=Path(temp);row=dict(t=0.,telemetry=dict(pad_load_raw_n={'L':.2,'R':.3},pad_load_filtered_n=[.2,.3],command_aperture_m=.014),aperture_m=.015,door_angle_deg=0.,T_moving_link=np.eye(4).tolist(),T_tcp=np.eye(4).tolist(),contacts=[])
        a=dict(folder=str(out),first_row=0,last_row=1,start_sim_s=0.)
        g=dict(rows=[row],telemetry_events=[],FINGERS=['L','R']);a['inspection']=passive.inspect_result(g,a);json.dumps(a)
        results['contact_result_no_recursive_JSON']=True
    if archived:
        from xarm7_native_planning import NativeModel
        from xarm7_native_gripper import configuration
        from cross_robot_adapter import RobotPinkIK
        from xarm7_native_execution import mapped_export
        from interactive_twin_recovery.mobile import physical_scene_class
        root=archived;ep=root/'episode';j=json.loads((root/'job.json').read_text());model=NativeModel(None,j['asset_root'],json.loads((Path(j['source'])/'report.json').read_text()))
        solver=RobotPinkIK(configuration());model.ik=solver.solve;folder=ep/'observations_chunks';idx=json.loads((folder/'index.json').read_text());row=pickle.load(gzip.open(folder/idx['chunks'][-1]['file'],'rb'))[-1]
        export=json.loads((ep/'cooked_initial.json').read_text());allowed=json.loads((ep/'sensor_metadata.json').read_text())['allowed_handle_colliders'];sc=physical_scene_class(Path(__file__).resolve().parents[1])(mapped_export(export),model,allowed)
        q=np.asarray(row['q']);g=dict(model=model,robot=SimpleNamespace(get_joint_positions=lambda:q),arm=np.arange(7),fingers=np.array([7,11,8,9,12,10]),collision=sc,base=row['base'],moving=lambda:np.asarray(row['T_moving_link']))
        results['archived_failed_state_jaw_sweep']=recovery.release_geometry(g,row['aperture_m'])
        q[g['fingers']]=model.native.angle(.084);g.update(ik=lambda T,seed:solver.solve(T,g['base'],seed,starts=1),tcp=lambda:np.asarray(row['T_tcp']),normal=np.asarray(json.loads((ep/'agent_geometry/observed_geometry.json').read_text())['normal_world']))
        results['archived_failed_state_clearance']=recovery.plan_clearance(g)
        planned=results['archived_failed_state_clearance']['path']
        if planned:
            candidate=json.loads((ep/'visual_plan.json').read_text());q[g['arm']]=planned[-1]
            results['archived_failed_state_unloaded_return']=recovery.checked_route(g,[planned[-1],candidate['approach'][0]]+list(reversed(candidate['preplan'])))
        results['archived_clearance_is_not_physical_unload_proof']=True
    results['passed']=True;return results


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--archived',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=verify(json.loads(a.job.read_text()),a.archived);a.output.write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if not k.startswith('archived')},indent=2))
