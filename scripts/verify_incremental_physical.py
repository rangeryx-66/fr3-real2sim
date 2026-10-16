"""Robot-free integration checks. Synthetic tests do not validate physical safety."""
import argparse,hashlib,json,tempfile,types,faulthandler
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import native_g1_agent_incremental_physical as adapter
import native_g1_agent_incremental_activation as incremental
import native_g1_agent_observation as original
import native_g1_agent_execution as physical
import native_g1_multiview_input as multi
from run_native_g1_incremental_physical import source
from verify_incremental_callbacks import verify as verify_motion


def verify(job):
    faulthandler.dump_traceback_later(15)
    results=dict(motion_callback=verify_motion(),physics_executed=False)
    proxy=adapter.prepare.__globals__['observation']
    assert proxy.observe is incremental.observe
    assert proxy.move_and_observe is incremental.move_and_observe
    assert proxy.build_geometry is incremental.build_geometry
    assert proxy.move_and_observe.__globals__['actuate'] is original.actuate
    assert adapter.prepare.__globals__['execute'] is adapter.execute
    assert adapter.validate_execution is physical.validate_execution
    with tempfile.TemporaryDirectory() as temp:
        out=Path(temp);seed=out/'fresh_observation';seed.mkdir();events=[]
        g=dict(a=SimpleNamespace(output=out),agent_observations=[dict(folder=str(seed),accepted_for_geometry=True)])
        def geometry(first,others,folder):
            events.append('semantic_incremental_geometry');folder.mkdir();(folder/'observed_patches.ply').write_bytes(b'contact-supported incremental test surface')
            return dict(spec={},geometry_folder=str(folder),marker='new_incremental_geometry')
        chosen=dict(T=np.eye(4).tolist(),preplan=[[0.]*7],contact_geometry_audit={'test':'current candidate'})
        def plan(state,folder):
            assert (folder/'observed_patches.ply').read_bytes()==b'contact-supported incremental test surface'
            events.append('planner_reads_incremental_contact_mesh');(out/'visual_plan.json').write_text(json.dumps(chosen));return chosen,{}
        semantic=incremental.build_geometry.__globals__['incremental']
        with patch.object(semantic,'build_geometry',geometry),patch.object(multi,'build_geometry',side_effect=AssertionError('OLD_FIRST_SEED_FUSION_CALLED')),patch.object(multi,'plan_observed',plan):
            result=incremental.build_geometry(g)
        assert result['geometry'] and result['plan'];assert events==['semantic_incremental_geometry','planner_reads_incremental_contact_mesh']
        results['geometry_binding']=events

    with tempfile.TemporaryDirectory() as temp:
        out=Path(temp);folder=out/'agent_geometry';folder.mkdir();seed=out/'fresh_observation';seed.mkdir()
        T=np.eye(4);T[:3,3]=[.123,.234,.456]
        candidate=dict(T=T.tolist(),preplan=[[0.]*7,[.1]*7],contact_geometry_audit={})
        data=dict(normal_world=[0.,-1.,0.],axis_world=[0.,0.,1.],legacy_estimate=dict(anchor_world_m=[.1,.2,.4]))
        (out/'visual_plan.json').write_text(json.dumps(candidate));(folder/'observed_geometry.json').write_text(json.dumps(data));(folder/'observed_patches.ply').write_bytes(b'current incremental contact-supported mesh')
        before=(out/'visual_plan.json').read_bytes();events=[]
        class World:
            def play(self):events.append('play')
        class Robot:
            def get_joint_positions(self):return np.full(7,.125)
        class Near:
            def set_world_pose(self,**kw):events.append('diagnostic_video_camera_only')
        g=dict(ROOT=out,a=SimpleNamespace(output=out),world=World(),robot=Robot(),arm=np.arange(7),qtarget=np.full(7,.125),tick=0,dt=1/240,
               wrist_config={'settle_s':.4},agent_observations=[dict(folder=str(seed),accepted_for_geometry=True)],
               job=dict(visual_reobservations=0,capture_shadow_only=False),near=Near(),moving=lambda:np.eye(4),
               profile_mark=lambda label:events.append(label),move=lambda q,duration:events.append('actuator_return'),hold=lambda t:events.append('settle'),
               diagnostic_issue=lambda name,error:events.append(['diagnostic',name,str(error)]))
        valid=lambda state:dict(valid=True,return_path=[[.125]*7,[.125]*7,[0.]*7])
        approach=lambda state,start,plan:dict(valid=True,preplan=[np.asarray(start).tolist(),[.1]*7],approach=[[.2]*7])
        with patch.dict(physical.execute.__globals__,validate_execution=valid,approach_from=approach,check_path=lambda *args:dict(valid=True)):
            adapter.execute(g,{'operation':'execute','synthetic_only':True})
        assert np.array_equal(g['T'],T) and g['whole']['T']==candidate['T']
        assert (out/'visual_plan.json').read_bytes()==before
        assert not any(isinstance(e,list) and e[0]=='diagnostic' for e in events),events
        results['physical_handoff']=dict(selected_T_preserved=True,plan_file_unchanged=True,incremental_geometry_loaded=True,events=events,
            return_and_approach_validation_stubbed=True,real_actuators_executed=False)

    with tempfile.TemporaryDirectory() as temp:
        out=Path(temp);commands=out/'agent_commands';commands.mkdir();events=[]
        for i,cmd in enumerate([dict(operation='evaluate_view',target={}),dict(operation='move_and_observe',evaluated_command=0),dict(operation='build_geometry'),dict(operation='validate_execution'),dict(operation='execute')]):(commands/('%03d.json'%i)).write_text(json.dumps(cmd))
        class Robot:
            def get_joint_positions(self):return np.zeros(7)
        def observe(state):state['agent_observations'].append({'folder':str(out),'accepted_for_geometry':True});events.append('observe');return {}
        def move(state,route):state['agent_moves']+=1;return observe(state)
        fake=SimpleNamespace(**proxy.__dict__)
        fake.install_camera=lambda state:None;fake.survey_candidates=lambda *args:iter([{}]);fake.select_route=lambda *args:({},[])
        fake.actuate=lambda *args:events.append('actuate');fake.observe=observe;fake.move_and_observe=move;fake.build_geometry=lambda state:events.append('incremental_build') or {'plan':True};fake.evaluate_view=lambda *args:dict(feasible=True,route={})
        # Original prepare source is reused; only its test dependencies are stubbed.
        fake.prepare=original.prepare
        fn=types.FunctionType(adapter.prepare.__code__,dict(adapter.prepare.__globals__,observation=fake,validate_execution=lambda state:dict(valid=True),execute=lambda *args:events.append('approved_handoff') or dict(approved=True)))
        g=dict(a=SimpleNamespace(output=out),robot=Robot(),arm=np.arange(7),world=SimpleNamespace(pause=lambda:None),job={'agent_wait_timeout_s':1},model=None,wrist_config={},solver=None,collision=None,base=[],wrist_X=None,wrist_routes=[])
        fn(g)
        trace=json.loads((out/'AGENT_ACTION_TRACE.json').read_text());assert len(trace)==5 and trace[-1]['command']['operation']=='execute'
        assert events==['actuate','observe','observe','incremental_build','approved_handoff'],events
        results['complete_Agent_interface']=dict(operations=[x['command']['operation'] for x in trace],events=events,returned_to_frozen_physical_runner=True)
    text=source(job);compile(text,'incremental_generated_preflight','exec')
    assert 'from native_g1_agent_incremental_physical import prepare' in text
    assert 'from native_g1_wrist_input import refine' in text
    results.update(passed=True,generated_sha256=hashlib.sha256(text.encode()).hexdigest(),measured_state_refine='unchanged native_g1_wrist_input.refine',controller_changes=False)
    faulthandler.cancel_dump_traceback_later()
    return results


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=verify(json.loads(a.job.read_text()));a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
