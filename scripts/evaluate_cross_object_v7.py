"""Post-subset-exit benchmark evaluation. No fitting, truth-driven choices or raw export."""
import argparse,datetime,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def authored_inertial_reference(truth,asset_root,joint_type):
    """Registered selected joint, not the first joint in a multi-door URDF.

    Same authored-body convention as the frozen evaluator; unrandomized moving
    bodies are explicitly reported, rather than silently calling this effective
    PhysX inertia. Native tensor readbacks remain a separate fidelity diagnostic.
    """
    import xml.etree.ElementTree as ET
    from articulated_demo.kinematics import URDFChain
    root=Path(asset_root);manifest=json.loads((root/'manifest.json').read_text());path=next((root/'urdf').glob('*.urdf'))
    tree=ET.parse(path).getroot();joint=tree.find("joint[@name='%s']"%manifest['joint_name']);chain=URDFChain(path)
    moving={joint.find('child').get('link')}
    for _ in range(len(tree.findall('link'))):
        moving|={j.find('child').get('link') for j in tree.findall('joint') if j.find('parent').get('link') in moving}
    bodies={b['body']:b for b in truth['bodies']};unrandomized=[]
    for link in tree.findall('link'):
        if link.get('name') not in moving or link.get('name') in bodies:continue
        inertia=link.find('inertial')
        if inertia is not None and float(inertia.find('mass').get('value'))>0:unrandomized.append(link.get('name'))
    if joint_type=='prismatic':return sum(b['mass_kg'] for b in bodies.values()),unrandomized
    X=chain.root_to_link(joint.find('child').get('link'),{});axis=X[:3,:3]@np.fromstring(joint.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis);origin=X[:3,3];value=0.
    for b in bodies.values():
        T=chain.root_to_link(b['body'],{});r=T[:3,:3]@b['com_local_m']+T[:3,3]-origin
        value+=float(axis@T[:3,:3]@np.asarray(b['inertia_local_kg_m2'])@T[:3,:3].T@axis+b['mass_kg']*(r@r-(axis@r)**2))
    return value,unrandomized


def evaluate(root):
    root=Path(root);registration=json.loads((root/'registration/BENCHMARK.json').read_text())
    active=[e for e in registration['episodes'] if e.get('execution_authorized',True)]
    # This check precedes ALL hidden-parameter reads, including failed episodes.
    for e in active:
        p=root/'episodes'/e['episode_id'];done=p/'PROCESS_COMPLETION.json'
        if not done.exists() or not json.loads(done.read_text()).get('actor_exited'):
            raise RuntimeError('Subset still online/unexecuted: '+e['episode_id'])
        pid=json.loads((p/'PROCESS.json').read_text())['pid']
        if Path('/proc/%d'%pid).exists():raise RuntimeError('Owned actor still alive')
    secret=root/'scene_private/CONFIGURATIONS.json'
    if hashlib.sha256(secret.read_bytes()).hexdigest()!=registration['hidden_configuration_sha256']:
        raise ValueError('Configuration commitment changed')
    from diagnose_probing_v3 import columns
    from update_probing_v7_model import load_segments
    from xarm_ft_v7_estimator import basis
    from interaction_runtime.compact_v2 import storage
    truth=json.loads(secret.read_text());rows=[];revealed=[]
    for e in registration['episodes']:
        row=dict(episode_id=e['episode_id'],asset_id=e['asset_id'],joint_type=e['joint_type'])
        if not e.get('execution_authorized',True):
            rows.append(dict(row,status='NOT_EXECUTED',reason=e.get('deferral_reason'),physical_attempts=0));continue
        p=root/'episodes'/e['episode_id'];case=truth[e['configuration_index']];revealed.append(case)
        ep=json.loads((p/'EPISODE_RESULT.json').read_text());completion=json.loads((p/'PROCESS_COMPLETION.json').read_text())
        runtime=json.loads((p/'RUNTIME_RESULT.json').read_text()) if (p/'RUNTIME_RESULT.json').exists() else {}
        trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text()) if (p/'AGENT_ACTION_TRACE.json').exists() else []
        row.update(status=ep.get('status'),wall_s=completion['wall_s'],runtime_profile=runtime.get('profile'),runtime_issues=runtime.get('issues'),storage=storage(p),segments=[],online_model=None,authored_reference=None)
        attempts=[t['result'] for t in trace if t.get('command',{}).get('operation')=='attempt_grasp_and_probe']
        row.update(physical_attempts=sum(bool(v.get('physical_grasp_started')) for v in attempts),grasp_acquisitions=sum(bool(v.get('acquisition',{}).get('acquired')) for v in attempts),recovery_attempts=max(sum(bool(v.get('physical_grasp_started')) for v in attempts)-1,0),
            observation_checkpoints=len(list((p/'rgbd').glob('*/camera.json'))),Agent_selected_view_moves=sum(t.get('command',{}).get('operation')=='move_and_observe' and 'observation' in t.get('result',{}) for t in trace),
            final_safe_park=bool(trace and trace[-1].get('result',{}).get('safe_exit',{}).get('safe')),initial_grasp_success=bool(attempts and attempts[0].get('acquisition',{}).get('acquired')))
        model_path=p/'model/estimate.json';model_hash=hashlib.sha256(model_path.read_bytes()).hexdigest() if model_path.exists() else None
        model=json.loads(model_path.read_text()) if model_path.exists() else {}
        if (p/'telemetry/index.json').exists() and json.loads((p/'telemetry/index.json').read_text()).get('chunks'):
            a,index=columns(p);q=a['joint_q'];row.update(maximum_coordinate=float(q.max()),minimum_coordinate=float(q.min()),final_coordinate=float(q[-1]),stored_records=len(q),physics_steps=index['physics_steps_seen'],simulation_s=index['physics_steps_seen']/240,
                raw_pad_peak_N=a['pad_load'].max(0).tolist(),raw_sensor_peak_abs=np.max(abs(a['raw_ft']),axis=0).tolist(),warning_samples=int(a['load_warning'].sum()),hard_stop_samples=int(a['safety_stop'].sum()),maximum_relative_translation_m=float(np.nanmax(a['relative_slip_m'])),
                numeric_chunks_intact=all(hashlib.sha256((p/'telemetry'/c['file']).read_bytes()).hexdigest()==c['sha256'] for c in index['chunks']),recording_issues=index.get('issues',[]))
            for s in index['segments']:
                ix=np.flatnonzero(a['segment']==s['id']);r=s.get('result',{})
                row['segments'].append(dict(id=s['id'],role=s['role'],requested_q=s['target_q'],actual_start_q=float(q[ix[0]]) if len(ix) else None,actual_end_q=float(q[ix[-1]]) if len(ix) else None,gain_native=float(q[ix[-1]]-q[ix[0]]) if len(ix) else None,status=r.get('status'),reason=r.get('reason'),sim_duration_s=r.get('elapsed_sim_s'),target_reached=bool(r.get('ok')),physical_transmission_valid=r.get('physical_transmission_valid')))
            row['opening_segment_travel_native']=sum(max(s['gain_native'] or 0,0) for s in row['segments'])
            row['closing_segment_travel_native']=-sum(min(s['gain_native'] or 0,0) for s in row['segments'])
        row['complete_validation']=any(s['role'].startswith('VALIDATION') and s['target_reached'] for s in row['segments'])
        first_val=min((s['id'] for s in row['segments'] if s['role'].startswith('VALIDATION') and s['target_reached']),default=10**9)
        row['post_complete_validation_FIT']=any(s['id']>first_val and s['role'].startswith('FIT') for s in row['segments'])
        if (p/'identification_input.json').exists():
            pieces,settings,quality=load_segments(p,True);prior=settings['prior'];authored=json.loads((p/'HIDDEN_AUTHORED_PHYSICS.json').read_text())
            job=json.loads((p/'job.json').read_text())
            inertial,extra=authored_inertial_reference(authored,job['asset_root'],e['joint_type'])
            row['authored_reference']=dict(inertial=inertial,damping=case['b'],resistance_law=case['law'],additional_unrandomized_moving_bodies=extra,inertial_reference='authored randomized rigid-body combination, not independently verified PhysX effective inertia/mass; additional moving masses require separate audit')
            row['excitation']=[dict(block=d['block_id'],role=str(d['segments'][0]),coordinate_range=[float(d['q'].min()),float(d['q'].max())],velocity_RMS=float(np.sqrt(np.mean(d['native_qd'][d['valid']]**2))),acceleration_RMS=float(np.sqrt(np.mean(d['qdd'][d['valid']]**2))),valid_samples=int(d['valid'].sum())) for d in pieces]
            row['FIT_quality']=quality
            if model.get('selected'):
                row['online_model']=dict(version=model['estimator_version'],selected=model['selected'],constant_model=model.get('constant_model'),extended_model=model.get('extended_model'),worker_wall_s=model.get('worker_wall_s'),source_sha256=model.get('source_sha256'),validation_used_for_fit_or_selection=model.get('validation_used_for_fit_or_selection'))
                comparisons={}
                for name in ['constant_model','extended_model']:
                    m=model.get(name)
                    if not m:continue
                    support=m['support_intervals_native'];grid=np.concatenate([np.linspace(lo,hi,101) for lo,hi in support])
                    pred=basis(grid,m['domain_native'],m['intervals'])[:,m['active_controls']]@m['parameters'][2:] if m['intervals'] else np.full(len(grid),m['parameters'][2])
                    law=case['law'];actual=law['base']+law['amplitude']*np.exp(-.5*((grid-law['center'])/law['width'])**2)
                    comparisons[name]=dict(inertial_absolute_error=m['parameters'][0]-inertial if not extra else None,inertial_relative_error_pct=100*(m['parameters'][0]/inertial-1) if not extra else None,damping_absolute_error=m['parameters'][1]-case['b'],damping_relative_error_pct=100*(m['parameters'][1]/case['b']-1),resistance_RMSE_observed_FIT_support=float(np.sqrt(np.mean((pred-actual)**2))),heldout_motion=m.get('heldout_motion',{}))
                row['posthoc_errors']=comparisons
        row['online_model_sha256']=model_hash;row['online_model_unchanged']=model_hash==hashlib.sha256(model_path.read_bytes()).hexdigest() if model_hash else None
        rows.append(row)
    out=root/'public_derived';out.mkdir(exist_ok=True)
    atomic_json(out/'REVEALED_COMPLETED_CONFIGURATIONS.json',revealed)
    metrics=dict(reference='25e2189',implementation_commit=registration['implementation_commit'],execution_scope=registration.get('execution_scope','all'),registered_episodes=16,executed_episodes=len(active),deferred_episodes=16-len(active),configuration_commitment=registration['hidden_configuration_sha256'],truth_reveal_UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,raw_published=False,authored_truth_used_for_online_fitting_or_decisions=False)
    atomic_json(out/'METRICS.json',result_packet(metrics))
    atomic_json(root/'TRUTH_ACCESS.json',dict(accessed_UTC=metrics['truth_reveal_UTC'],all_authorized_actors_exited=True,revealed_episode_ids=[e['episode_id'] for e in active],deferred_truth_not_exported=True))
    print(json.dumps(dict(executed=len(active),deferred=16-len(active),output=str(out/'METRICS.json'))));return metrics


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);evaluate(p.parse_args().root)
