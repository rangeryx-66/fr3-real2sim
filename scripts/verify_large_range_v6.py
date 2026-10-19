"""Synthetic above-20deg, source separation, and actual async worker; no Isaac."""
import json,sys,time,threading,concurrent.futures
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from verify_estimator_v31 import synthetic_rows,PRIOR
from xarm_ft_v31_estimator import process,common_filter
from xarm_ft_v6_estimator import basis,fit,analyze,matrix,samples_from_piece,CONFIG

def run(episode,out):
    began=time.perf_counter();checks=[]
    for varying in [False,True]:
        samples=[];controls=.035+(.012*np.exp(-.5*((np.arange(basis([0],10.).shape[1])-4)/.8)**2) if varying else np.zeros(basis([0],10.).shape[1]))
        for k,offset in enumerate([0.,8.,16.,24.,32.]):
            rows,v,_=synthetic_rows(role='FIT_SYNTHETIC_%d'%k,mass=.75)
            q=np.deg2rad([r['door_angle_deg'] for r in rows])+np.deg2rad(offset);C=basis(q,10.)@controls
            for i,row in enumerate(rows):
                row['door_angle_deg']=float(np.rad2deg(q[i]));T=row['wrist_ft']['T_world_sensor'];p=T[:3,3];F=(C[i]-.04)*np.sign(v[i])*np.cross([0,0,1],p)/np.linalg.norm(p[:2])**2;row['wrist_ft']['raw_wrench_sensor'][:3]+=T[:3,:3].T@F
            d=process(rows,PRIOR,[0]*6,v);d['block_id']='FIT_SYNTHETIC_%d'%k;samples+=samples_from_piece(d)
            assert np.max(abs(d['angle_features'][10.]-common_filter(basis(q,10.)*np.sign(v)[:,None])))<1e-13
        m=fit(samples,10.,.0001);assert abs(m['parameters'][0]-.3)<.004,m['parameters'];assert abs(m['parameters'][1]-.12)<.004,m['parameters'];error=max(abs(np.array(m['parameters'][2:])-controls[m['active_controls']]))
        A=matrix(samples,m);predicted=A@np.array(m['parameters']);expected=np.array([s['tau'] for s in samples]);rms=float(np.sqrt(np.mean((predicted-expected)**2)));assert rms<.0002,rms
        a=analyze(samples)
        if not varying:assert a['selected_model']=='constant'
        checks.append(dict(varying=varying,parameters=m['parameters'][:2],control_max_error_nm=float(error),FIT_coverage_deg=a['coverage_deg'],selected=a['selected_model']))
    from interaction_runtime.probing_v6 import OperationsV6
    from interaction_runtime.recovery_v44 import OperationsV44
    for name in ['observe','evaluate_view','move_and_observe','build_geometry','validate_execution','acquire','attempt','interact','release_park','push']:assert getattr(OperationsV6,name) is getattr(OperationsV44,name),name
    # Replay via actual asynchronous production path. Read-only symlinks, no raw copy.
    out.mkdir(parents=True,exist_ok=True);replay=out/'async_replay';replay.mkdir(exist_ok=True)
    for name in ['telemetry','identification_input.json']:
        p=replay/name
        if not p.exists():p.symlink_to((episode/name).resolve(),target_is_directory=name=='telemetry')
    from types import SimpleNamespace
    ops=object.__new__(OperationsV6);ops.r=SimpleNamespace(root=ROOT,output=replay,job=dict(allow_interrupted_fit=True));ops.worker=concurrent.futures.ThreadPoolExecutor(max_workers=1);ops.updates=[];ready=threading.Event();ready.set();ops.update_async(ready);status=ops.updates[0].result(timeout=65);ops.close_workers();assert status['estimator_ready'],status
    baseline=json.loads((replay/'model/estimate.json').read_text())
    import update_probing_v31_model as loader
    from diagnose_probing_v3 import columns
    from update_probing_v6_model import update
    arrays,index=columns(episode);labels={s['id'] for s in index['segments'] if s['role'].startswith('VALIDATION')};mask=np.isin(arrays['segment'],list(labels));changed={k:v.copy() for k,v in arrays.items()};changed['raw_ft'][mask]+=.1;changed['door_q'][mask]+=.02;changed['door_dq'][mask]*=1.2;original=loader.columns
    try:
        loader.columns=lambda p:(changed,index);mutation=update(episode,out/'mutation',plot=False,allow_interrupted_fit=True)
    finally:loader.columns=original
    
    for field in ['parameters','conditional_std','conditional_covariance','cv_rmse_nm','FIT_only_leave_trajectory_out']:
        assert baseline['selected'][field]==mutation['selected'][field],field
    assert baseline['selected_model']==mutation['selected_model']
    for p in ['interaction_runtime/operations.py','interaction_runtime/recovery_v44.py','scripts/native_g1_wrist_input.py','scripts/native_g1_incremental_geometry.py','scripts/xarm_ft_v6_estimator.py','scripts/update_probing_v6_model.py']:
        f=ROOT/p
        if f.exists():assert 'passive_dataset_output' not in f.read_text()
    result=dict(status='PASS_OFFLINE',synthetic=checks,async_worker=status,VALIDATION_mutation_FIT_identical=True,production_operations_identity=True,no_fixed_camera_read_by_online_consumers=True,wall_s=time.perf_counter()-began,configuration=CONFIG);(out/'VERIFICATION.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':run(*map(Path,sys.argv[1:3]))
