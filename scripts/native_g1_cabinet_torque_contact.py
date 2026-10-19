"""V1.1 cabinet experiment adapter; frozen perception/control/safety are reused."""
import inspect
from pathlib import Path
from unittest.mock import patch
import native_g1_hybrid_contact as hybrid
import native_g1_large_range_contact as continuous
import native_g1_contact_recovery as recovery


def pull(g):
    source=inspect.getsource(continuous.pull).replace('run_torque_components_v1.py','run_cabinet_torque_v11.py')
    ns=dict(continuous.__dict__)
    exec(compile(source,__file__+'::same_temporal_block','exec'),ns)
    return ns['pull'](g)


def handle_attempt(g,command):
    if g['physical_grasps_started']>=2:
        return dict(executed=False,reason='one initial acquisition plus one recovery already used')
    if g['contact_attempts'] and g['contact_attempts'][-1]['controller_target_reached']:
        return dict(executed=False,reason='completed interaction block; no recovery needed')
    actions=command.get('segments',[])
    if not actions or len(actions)>4:
        return dict(executed=False,reason='explicit finite interaction block required')
    if any(not (0<float(a['target_deg'])<=30 and a['role'].startswith(('FIT','VALIDATION'))) for a in actions):
        return dict(executed=False,reason='outside registered finite action menu')
    g['torque_preapproved_actions']=actions
    g['interaction_mode']='HANDLE_PULL' if not g['contact_attempts'] else 'HANDLE_RECOVERY_REGRASP'
    source=inspect.getsource(recovery.attempt_grasp_and_probe)
    source=source.replace("    if any(r['short_success'] for r in g['contact_attempts']):return dict(executed=False,reason='first success: recovery prohibited by registration')\n",'')
    anchor="ok,status,_=namespace['open_with_contact_feedback'](g)"
    if source.count(anchor)!=1:raise ValueError('Frozen operation binding changed before execution')
    source=source.replace(anchor,'ok,status,_=pull(g)')
    ns=dict(recovery.__dict__,continuous=continuous,pull=pull,safe_release_retreat=hybrid.safe_exit)
    exec(compile(source,__file__+'::frozen_acquisition_and_exit','exec'),ns)
    result=ns['attempt_grasp_and_probe'](g,command)
    result['acquisition_kind']='INITIAL' if result['index']==0 else 'RECOVERY'
    result['full_block_complete']=result['controller_target_reached']
    recovery.write(g,'CONTACT_ATTEMPTS.json',g['contact_attempts'])
    return result


def run(g):
    policy=g['a'].output/'CONTINUOUS_CONTACT_POLICY.py'
    policy.write_text(continuous.contact_policy_source())
    g['spec']=dict(g['spec'],shared_policy=str(policy));g['torque_workers']=[]
    # Reuse the existing bounded panel alternatives, current-state observation,
    # sensor, passive plant, release and joint retreat/parking operations.
    # Only registered recovery permission and segment worker are adapted.
    original_save=hybrid.save_segment
    def save(state,first,label,result):
        original_save(state,first,label,result)
        folder=Path(state['hybrid_segments'][-1]['folder'])
        import json
        p=folder/'INPUT.json';d=json.loads(p.read_text());d['model_variant']='vertical_cabinet_constant';p.write_text(json.dumps(d,indent=2))
    with patch.object(hybrid,'handle_attempt',handle_attempt),patch.object(hybrid,'pull',pull),patch.object(hybrid,'push_step',continuous.push_step),patch.object(hybrid,'save_segment',save):
        return hybrid.run(g)
