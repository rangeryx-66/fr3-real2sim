"""ENGINEERING ONLY: independent known-force breakaway audit, no robot contact.

An explicitly applied calibrated test force is allowed only in this separate
fixture. Never imported by a manipulation runner, estimator or benchmark.
Native mass, resistance, constraints, contacts and robot protections are kept.
No object joint position/velocity/effort target is submitted after normal init.
"""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def run(job):
    from interaction_runtime.probing_v7 import RuntimeV7
    from xarm_ft_known_joint_prior import native_joint_prior
    from agent_command_protocol_v12 import atomic_json
    r=None;plant=None;rows=[];began=time.perf_counter();result=dict(engineering_only=True,robot_contact=False,object_joint_commands=False,independent_external_test_load=True,
        source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['scripts/check_prismatic_breakaway_v7.py','scripts/xarm_ft_v7_scene.py','interaction_runtime/native_scene.py']})
    try:
        r=RuntimeV7(job);r.g['phase']='PREFLIGHT'
        prior=native_joint_prior(job['asset_root'],r.moving())
        if prior['joint_type']!='prismatic':raise ValueError('Rail-only independent fixture')
        axis=np.asarray(prior['axis_world']);axis/=np.linalg.norm(axis)
        plant=r.engineering_before_physics;load=np.zeros(3)
        body=r.scene['door_contacts'];local_com,_=body.get_coms();local_com=np.asarray(local_com).reshape(-1,3)
        if len(local_com)!=1:raise ValueError('Independent force fixture requires exactly one moving body COM')
        local_com=local_com[0]
        def apply():
            plant()
            X=r.moving();point=X[:3,:3]@local_com+X[:3,3]
            body.apply_forces_and_torques_at_pos(forces=load[None,:],positions=point[None,:],is_global=True)
        r.engineering_before_physics=apply
        def step(force,phase):
            load[:]=axis*force;row=r.step()
            rows.append(dict(t=row['t'],q=row['joint_coordinate'],dq=row['joint_coordinate_velocity'],force=force,phase=phase,raw_ft=row['wrist_ft']['raw_wrench_sensor'].copy()))
            if any(c['force_n']>0 for c in row['contacts']):raise RuntimeError('Independent fixture contaminated by robot contact')
        def settle(label):
            for _ in range(240):step(0.,label)
        settle('initial_unloaded')
        for force in np.arange(0,.501,.025):
            for _ in range(48):
                if r.actual()>=.015:break
                step(float(force),'opening_force_ramp')
            if r.actual()>=.015:break
        settle('opening_coast')
        open_q=r.actual();target=max(.002,open_q-.010)
        if open_q>.003:
            for force in np.arange(0,.501,.025):
                for _ in range(48):
                    if r.actual()<=target:break
                    step(float(-force),'closing_force_ramp')
                if r.actual()<=target:break
        settle('final_coast')
        x={k:np.asarray([row[k] for row in rows]) for k in rows[0]}
        moving=np.abs(x['dq'])>1e-4
        onset={}
        for phase in ['opening_force_ramp','closing_force_ramp']:
            ix=np.flatnonzero((x['phase']==phase)&moving)
            onset[phase]=dict(force_N=float(x['force'][ix[0]]),q_m=float(x['q'][ix[0]]),dq_m_s=float(x['dq'][ix[0]])) if len(ix) else None
        np.savez_compressed(r.output/'independent_force_samples.npz',**x)
        result.update(status='COMPLETED',breakaway=onset,maximum_q_m=float(x['q'].max()),closing_travel_m=float(open_q-r.actual()),final_q_m=r.actual(),samples=len(rows),physics_hz=240,
            input_limit_N=.5,force_applied_at='measured body COM; world rail axis',native_resistance_readback=json.loads((r.output/'PRIVATE_NATIVE_FRICTION_PREFLIGHT.json').read_text()),
            limitations=['Separate external-force engineering fixture, not robot manipulation evidence','Velocity threshold is an onset diagnostic, not a force/safety limit','Authored parameters are not independently calibrated effective dynamics'])
    except BaseException as error:
        result.update(status='FAILED',error=repr(error));raise
    finally:
        if r is not None:
            if plant is not None:r.engineering_before_physics=plant
            result['wall_s']=time.perf_counter()-began
            atomic_json(r.output/'INDEPENDENT_BREAKAWAY_RESULT.json',result)
            r.close()
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path);a=p.parse_args()
    print(json.dumps(run(json.loads(a.job.read_text())),indent=2))
