"""Bounded contact-mode owner with continuous virtual-reference handovers.

Frozen closure, Pink solver, actuator, force and collision checks are reused.
Segment workers are passive; there is no waiting for estimates under load.
"""
import json,pickle,subprocess,sys,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
import native_g1_hybrid_contact as hybrid
import native_g1_ft_multistate as multistate
from interaction_reference_handover import handover,contact_policy_source


def segment(g,action,index):
    source=Path(multistate.__file__).read_text()
    # Reuse the SAME finite temporal segment; only repair its entry reference.
    import inspect
    source=inspect.getsource(multistate.segment)
    anchor="    g['step']=profiled_step"
    if source.count(anchor)!=1:raise ValueError('Pinned segment entry boundary changed')
    source=source.replace(anchor,anchor+"\n    handover(g,'SEGMENT_START_'+name)")
    namespace=dict(multistate.__dict__,handover=handover)
    exec(compile(source,__file__+'::continuous_segment_entry','exec'),namespace)
    return namespace['segment'](g,action,index)


def pull(g):
    records=[]
    actions=g['torque_preapproved_actions']
    for action in actions:
        first=len(g['rows']);index=len(g['hybrid_segments']);result=dict(ok=False,status='interrupted before segment completion',action=action)
        try:
            result=segment(g,action,index);records.append(result)
        finally:
            # Preserve even an interrupted segment independently.
            hybrid.save_segment(g,first,action['role'],result)
            folder=Path(g['hybrid_segments'][-1]['folder'])
            log=(folder/'torque_worker.log').open('w')
            worker=subprocess.Popen([sys.executable,str(g['ROOT']/'scripts/run_torque_components_v1.py'),str(folder)],stdout=log,stderr=subprocess.STDOUT)
            log.close();g['torque_workers'].append(worker)
        if not result['ok']:return False,result['status'],records
    return True,'PREAPPROVED_BIDIRECTIONAL_BLOCK_COMPLETE',records


def run(g):
    policy=g['a'].output/'CONTINUOUS_CONTACT_POLICY.py';policy.write_text(contact_policy_source())
    g['spec']=dict(g['spec'],shared_policy=str(policy));g['torque_workers']=[]
    # Reuse explicit Agent mode interface. Only pull primitive and its entry
    # operation differ. No new asynchronous dynamic callback dependency chain.
    with patch.object(hybrid,'pull',pull),patch.object(hybrid,'handle_attempt',handle_attempt_original),patch.object(hybrid,'push_step',push_step):
        return hybrid.run(g)

# The original operation table resolves handle_attempt as a module global.
# Keep that call target explicit to avoid self-recursion under the patch.
_original_handle_attempt=hybrid.handle_attempt

def handle_attempt_original(g,command):
    actions=command.get('segments')
    if not actions or len(actions)>4:return dict(executed=False,reason='explicit bounded segment list required')
    if any(not(0<float(a['target_deg'])<=30 and a['role'].startswith(('FIT','VALIDATION'))) for a in actions):
        return dict(executed=False,reason='outside registered finite action envelope')
    g['torque_preapproved_actions']=actions
    return _original_handle_attempt(g,command)


def push_step(g,goal):
    """Same Pink solver; smooth compact-hand entry, same pair/limit checks."""
    from scipy.spatial.transform import Rotation
    if not g.get('_panel_reference_started'):
        handover(g,'UNLOADED_TO_PANEL_PUSH');g['_panel_reference_started']=True
    radius=max(.001,float(np.linalg.norm(g['panel_contact_tcp'])))
    ref=g['ref'];dp=goal[:3,3]-ref[:3,3];rv=Rotation.from_matrix(goal[:3,:3]@ref[:3,:3].T).as_rotvec()
    error=np.r_[dp,rv*radius];distance=np.linalg.norm(error);dt=g['dt'];acc=.002
    wanted=error/max(distance,1e-12)*min(.0015,np.sqrt(2*acc*distance));change=wanted-g['twist']
    g['twist']+=change*min(1.,acc*dt/max(np.linalg.norm(change),1e-12));inc=g['twist']*dt
    if np.dot(inc,error)>0 and np.linalg.norm(inc)>distance:inc=error.copy();g['twist']=inc/dt
    target=ref.copy();target[:3,3]+=inc[:3];target[:3,:3]=Rotation.from_rotvec(inc[3:]/radius).as_matrix()@ref[:3,:3]
    q=g['solver'].velocity_step(target,g['base'],g['seed'],dt)
    if q is None:raise RuntimeError('PANEL_PUSH_NO_IK')
    command=q+g['bias']
    if g['model'].margin(command)<=.05:raise RuntimeError('EXISTING_COMMAND_JOINT_MARGIN')
    delta=command-g['qtarget'][g['arm']];fraction=min(1.,float(np.min((g['vel']/3)/np.maximum(abs(delta)/dt,1e-12))))
    command=g['qtarget'][g['arm']]+fraction*delta
    ok,why=g['collision'].check(g['model'].poses(command,g['base'],finger_q=np.asarray(g['robot'].get_joint_positions())[g['fingers']]),g['moving'](),False)
    if not ok:raise RuntimeError(why)
    g['qtarget'][g['arm']]=command;g['qvelocity']=fraction*delta/dt;g['seed']=command-g['bias'];g['ref']=target;g['step']()
