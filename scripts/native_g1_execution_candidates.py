"""Current-state execution feasibility after frozen measured-contact ranking.

Native geometry is used only for safety; scores continue to use observed surfaces.
No collision exception, changed margin, grasp offset or asset-ID branch.
"""
import copy,json,time
from contextlib import contextmanager
from pathlib import Path
import numpy as np
from agent_command_protocol_v12 import atomic_json
from native_g1_agent_execution import validate_execution
from interaction_runtime.motion import timed_path


def current_export(export,moving_link,moving_pose):
    result=copy.deepcopy(export);M=np.asarray(moving_pose)
    for entry in result['shapes']:
        if entry.get('rigid_body_path','').split('/')[-1]!=moving_link:continue
        old=np.asarray(entry['rigid_body_world_transform']);D=M@np.linalg.inv(old)
        entry['world_transform']=(D@np.asarray(entry['world_transform'])).tolist()
        entry['rigid_body_world_transform']=M.tolist()
    return result


class CandidateCheck:
    def __init__(self,g,policy,config,solver):
        self.g=g;self.policy=policy;self.config=config;self.solver=solver;self.audit=[];self.return_cache={}
        self.moving=np.asarray(g['moving']()).copy();self.fingers=np.asarray(g['robot'].get_joint_positions())[g['fingers']].copy()
    def __call__(self,candidate):
        began=time.perf_counter();g=self.g
        check=validate_execution(g,candidate=candidate,write=False)
        result=dict(valid=check['valid'],return_valid=check['return_audit']['valid'],approach_valid=check['approach_audit']['valid'],
                    actual_door_state_rad=float(g['actual']()),measured_fingers_rad=self.fingers.tolist(),
                    reason=next((check[k].get('reason') for k in ['return_audit','approach_audit'] if not check[k]['valid']),None))
        if g['model'].manifest.get('joint_type','revolute')=='prismatic':
            result['actual_joint_coordinate_m']=result.pop('actual_door_state_rad')
        if result['valid']:
            def checked(q):
                if g['model'].margin(q)<=self.policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
                return g['collision'].check(g['model'].poses(q,g['base'],finger_q=self.fingers),self.moving,False)
            def timed(path,phase):
                return sum(1 for _ in timed_path(self.solver,g['base'],path,1/240,self.policy.motion(phase),np.asarray(self.config['arm_speed_limits'])/3,self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)],checked))
            try:
                key=np.asarray(check['return_path']).tobytes()
                if key not in self.return_cache:self.return_cache[key]=timed(check['return_path'],'PREGRASP')
                result['return_timed_samples']=self.return_cache[key]
                result['preplan_timed_samples']=timed(check['approach_audit']['preplan'],'PREGRASP')
                result['approach_timed_samples']=timed(check['approach_audit']['approach'],'APPROACH')
            except RuntimeError as error:result.update(valid=False,reason='EXECUTION_TIMED_ROUTE:'+str(error))
        result['wall_s']=time.perf_counter()-began
        self.audit.append(dict(family=[candidate[k] for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']],**result))
        atomic_json(g['a'].output/'EXECUTION_CANDIDATE_AUDIT.json',dict(candidates=self.audit,weights_changed=False,scene='measured current moving body and released fingers',native_use='collision safety only'))
        return result


@contextmanager
def context(g,policy,config,solver):
    keys=['planning_collision_export','planning_start_state_rad','execution_candidate_check'];old={k:g[k] for k in keys if k in g}
    M=np.asarray(g['moving']()).copy();export=json.loads((g['a'].output/'cooked_initial.json').read_text());p=g['a'].output/'planning_current_scene.json'
    # Preserve the native export convention (unused authored drive limits may
    # contain infinities); strict Agent-command JSON is a different interface.
    temporary=p.with_suffix('.pending');temporary.write_text(json.dumps(current_export(export,g['collision'].moving_link,M)));temporary.replace(p)
    g.update(planning_collision_export=p,planning_start_state_rad=float(g['actual']()),execution_candidate_check=CandidateCheck(g,policy,config,solver))
    try:yield g['execution_candidate_check']
    finally:
        for k in keys:
            if k in old:g[k]=old[k]
            else:g.pop(k,None)
