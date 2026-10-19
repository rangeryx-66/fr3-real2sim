"""Synthetic + archived-data + actual async subprocess validation. No Isaac."""
import argparse,concurrent.futures,hashlib,json,sys,threading,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from xarm_ft_v31_estimator import process,regression_samples,VERSION
from update_probing_v31_model import update,conditional_model
import update_probing_v31_model as production
from interaction_runtime.estimator_v31 import OperationsV31,WORKER_TIMEOUT_S
from interaction_runtime.probing_v3 import OperationsV3


def synthetic_rows(role='FIT_SYNTHETIC',mass=.75):
    dt=1/240;t=np.arange(0,10,dt);w=2*np.pi*.5
    q=.02*t+.009*np.sin(w*t);v=.02+.009*w*np.cos(w*t);acc=-.009*w*w*np.sin(w*t)
    parameters=np.array([.3,.12,.04]);torque=parameters[0]*acc+parameters[1]*v+parameters[2]*np.sign(v)
    Ibody=np.diag([.0003,.0004,.0005])*(mass/.75);localCOM=np.array([.03,.01,.02]);mount=Rotation.from_euler('x',.4).as_matrix();axis=np.array([0.,0.,1.]);rows=[]
    for i in range(len(t)):
        Z=Rotation.from_rotvec(axis*q[i]).as_matrix();R=Z@mount;sensor=Z@np.array([.4,0.,.1]);COM=sensor+R@localCOM;radius=COM.copy();radius[2]=0
        A=acc[i]*np.cross(axis,radius)-v[i]**2*radius;omega=axis*v[i];alpha=axis*acc[i];I=R@Ibody@R.T
        toolforce=mass*A;toolmoment=I@alpha+np.cross(omega,I@omega)+np.cross(COM-sensor,toolforce)
        gravity=mass*np.array([0.,0.,-9.81]);gravM=np.cross(COM-sensor,gravity)
        contactF=torque[i]*np.cross(axis,sensor)/np.linalg.norm(sensor[:2])**2
        worldF=toolforce-gravity+contactF;worldM=toolmoment-gravM;T=np.eye(4);T[:3,:3]=R;T[:3,3]=sensor
        rows.append(dict(t=float(t[i]),door_angle_deg=float(np.rad2deg(q[i])),identification_segment=role,phase='OPEN_CONTINUE',relative_translation_slip_m=0.,forces_n=dict(L=.5,R=.5),force_event=dict(status='NORMAL'),
            wrist_ft=dict(valid=True,raw_wrench_sensor=np.r_[R.T@worldF,R.T@worldM],T_world_sensor=T,tool_gravity_wrench_world=np.r_[gravity,gravM],
                body_states=[dict(name='analytic_tool',mass_kg=mass,T_world_body=T,com_world_m=COM,inertia_world_kg_m2=I)])))
    return rows,v,parameters


PRIOR=dict(joint_type='revolute',axis_world=[0.,0.,1.],origin_world_m=[0.,0.,0.])


def verify_numeric():
    checks=dict(no_Isaac=True,synthetic=[])
    for mass in [0.,.75]:
        rows,v,known=synthetic_rows(mass=mass)
        for window in [61,121,181]:
            d=process(rows,PRIOR,[0.]*6,v,window);m=conditional_model(regression_samples(d),window);fit=np.array(m['parameters']);relative=abs(fit/known-1)
            assert relative.max()<.003,(mass,window,fit)
            covariance=np.array(m['conditional_covariance']);std=np.array(m['conditional_std']);assert np.allclose(np.diag(covariance),std**2,rtol=1e-12,atol=1e-18);assert np.linalg.eigvalsh(covariance).min()>-1e-16
            assert not np.allclose(d['direction'],np.sign(d['qd'])),'friction basis was silently replaced'
            checks['synthetic'].append(dict(tool_mass_kg=mass,window=window,known=known.tolist(),parameters=fit.tolist(),relative_errors=relative.tolist(),covariance_std_consistent=True))
    rows,v,_=synthetic_rows();before=process(rows,PRIOR,[0.]*6,v)
    bad=list(rows);bad[len(rows)//2]=dict(bad[len(rows)//2],identification_segment='VALIDATION_FORBIDDEN')
    try:process(bad,PRIOR,[0.]*6,v);raise AssertionError('mixed segment accepted')
    except ValueError:pass
    bad=list(rows);bad[len(rows)//2]=dict(bad[len(rows)//2],t=rows[len(rows)//2]['t']+.02)
    try:process(bad,PRIOR,[0.]*6,v);raise AssertionError('time gap accepted')
    except ValueError:pass
    samples=regression_samples(before)
    for r in samples:r['qdd']=0.;r['qd']=.02;r['direction']=1.;r['tau']=.04
    constant=conditional_model(samples);assert constant['algebraic_rank']<3 and not any(constant['supported'])
    checks.update(boundary_and_gap_rejection=True,constant_speed_rank_deficiency_reported=True,
        default_filter_lookahead_s=before['processing']['filter_lookahead_s'],tool_compensation_count=before['processing']['tool_compensation_count'])
    for method in ['observe','move_and_observe','build_geometry','validate_execution','acquire','interact','release_park','push','attempt','update_async']:
        assert getattr(OperationsV31,method) is getattr(OperationsV3,method),method
    checks['frozen_operations_inherited_without_override']=True
    import run_informative_probing_v31 as entry
    import interaction_runtime.estimator_v31 as integration
    original_serve=integration.serve
    try:
        integration.serve=lambda runtime,operations_factory:operations_factory
        assert entry.serve_v31(object()) is OperationsV31
    finally:integration.serve=original_serve
    assert 'scripts/update_probing_v31_model.py' in entry.ESTIMATOR_SOURCES
    assert 'scripts/xarm_ft_v31_estimator.py' in entry.ESTIMATOR_SOURCES
    checks['entrypoint_to_factory_to_async_worker_verified']=True
    return checks


def replay(episode,out):
    from diagnose_probing_v3 import columns
    from update_probing_v3_model import update as legacy_update
    from diagnose_inertia_bias_v3 import weak_score
    start=time.perf_counter();out.mkdir(parents=True,exist_ok=True);result=dict(numeric=verify_numeric(),physical_episodes=0)
    archive_hashes={name:hashlib.sha256((episode/name).read_bytes()).hexdigest() for name in ['telemetry/index.json','model/estimate.json','model/updates.json']}
    # Tiny read-only symlink view: no copied measurement blocks, no legacy writes.
    job=out/'async_replay';job.mkdir(exist_ok=True)
    for name in ['telemetry','identification_input.json']:
        link=job/name
        if not link.exists():link.symlink_to((episode/name).resolve(),target_is_directory=name=='telemetry')
    legacy_folder=out/'legacy_replay';legacy_folder.mkdir(exist_ok=True)
    for name in ['telemetry','identification_input.json']:
        link=legacy_folder/name
        if not link.exists():link.symlink_to((episode/name).resolve(),target_is_directory=name=='telemetry')
    legacy_start=time.perf_counter();legacy=legacy_update(legacy_folder);result['legacy_total_wall_s']=time.perf_counter()-legacy_start
    archived=json.loads((episode/'model/estimate.json').read_text());assert np.allclose(legacy['constant_model']['parameters'],archived['constant_model']['parameters'],rtol=0,atol=1e-12)
    result['legacy_exact_reproduction']=dict(parameters=legacy['constant_model']['parameters'],heldout=legacy['constant_model']['heldout_motion'])
    ops=object.__new__(OperationsV31);ops.r=SimpleNamespace(root=ROOT,output=job);ops.worker=concurrent.futures.ThreadPoolExecutor(max_workers=1);ops.updates=[]
    ready=threading.Event();timer=threading.Timer(.05,ready.set);timer.start();ops.update_async(ready);ops.update_async(ready)
    statuses=[future.result(timeout=WORKER_TIMEOUT_S*2+5) for future in ops.updates];ops.close_workers();timer.join()
    assert all(s.get('estimator_ready') and s['wall_s']<WORKER_TIMEOUT_S for s in statuses),statuses
    baseline=json.loads((job/'model/estimate.json').read_text());result['async_production_subprocesses']=statuses;result['production']=baseline
    for s in statuses:
        for file,sha in s['loaded_sources'].items():assert hashlib.sha256((ROOT/file).read_bytes()).hexdigest()==sha
    a,index=columns(episode);original_columns=production.columns
    labels={s['id'] for s in index['segments'] if s['role'].startswith('VALIDATION')};mask=np.isin(a['segment'],list(labels));changed={k:v.copy() for k,v in a.items()}
    changed['raw_ft'][mask]+=np.arange(6)*.07+.03;changed['door_q'][mask]+=.01*np.sin(changed['t'][mask]);changed['door_dq'][mask]*=1.2
    changed['tool_poses'][mask,:,:3]+=.002*np.sin(changed['t'][mask])[:,None,None]
    try:
        production.columns=lambda unused:(changed,index)
        mutation=update(episode,out/'mutated_validation',plot=False)
    finally:production.columns=original_columns
    for field in ['parameters','conditional_std','conditional_covariance','FIT_only_leave_segment_out','information']:
        assert mutation['constant_model'][field]==baseline['constant_model'][field],field
    result['validation_mutation']=dict(FIT_parameters_covariance_information_CV_bit_identical=True,modified_channels=['raw_FT','door_position','door_velocity','tool_poses'],complete_segments=True)
    result['window_sensitivity']={}
    for window in [61,181]:
        value=update(episode,out/('window_%d'%window),window=window,plot=False)
        result['window_sensitivity'][str(window)]=dict(parameters=value['constant_model']['parameters'],training_bins=value['training_bins'],heldout_torque_rmse_nm=value['constant_model']['heldout_torque_rmse_nm'],heldout_motion=value['constant_model']['heldout_motion'],processing_wall_s=value['worker_wall_s'])
    # Independent weak formulation is only a cross-check; no result enters update().
    from diagnose_probing_v3 import rows_at
    from xarm_ft_id_estimator import process as old_process
    settings=json.loads((episode/'identification_input.json').read_text());base=[];common=[]
    for s in index['segments']:
        if not s.get('result',{}).get('ok'):continue
        ix=np.flatnonzero(a['segment']==s['id']);rows=rows_at(a,ix,index['metadata'],s['role']+'_%02d'%s['id']);d=old_process(rows,settings['prior'],settings['bias']);d['native_qd']=a['door_dq'][ix];d['valid'][:121]=False;d['valid'][-121:]=False
        rawtool=d['torque']+np.array([np.asarray(settings['prior']['axis_world'])@(w[3:]+np.cross(np.asarray(row['wrist_ft']['T_world_sensor'])[:3,3]-settings['prior']['origin_world_m'],w[:3])) for w,row in zip(d['inertial_world'],rows)])
        base.append(dict(d=d,rows=rows,dt=1/240,prior=settings['prior'],before_tool=rawtool));common.append(d)
    result['independent_weak_verification']={str(w):weak_score(base,w,common) for w in [.5,1.]}
    result['archive_unchanged']=all(hashlib.sha256((episode/name).read_bytes()).hexdigest()==sha for name,sha in archive_hashes.items());assert result['archive_unchanged']
    result['original_archive_hashes']=archive_hashes
    result['wall_s']=time.perf_counter()-start
    (out/'VALIDATION.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(status='validated',production_parameters=baseline['constant_model']['parameters'],async_statuses=statuses,wall_s=result['wall_s']),indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    if a.episode:replay(a.episode,a.output)
    else:print(json.dumps(verify_numeric(),indent=2))
