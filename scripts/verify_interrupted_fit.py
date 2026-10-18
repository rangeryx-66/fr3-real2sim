"""Actual V3.1 worker input-selection regression; no Isaac and no GT selection."""
import json,sys,tempfile,threading
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.probing_v3 import RecorderV3
from interaction_runtime.estimator_v31 import OperationsV31
from verify_estimator_v31 import synthetic_rows,PRIOR
import update_probing_v31_model as production


def verify(adaptive=False):
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder);record=RecorderV3(p/'telemetry');(p/'identification_input.json').write_text(json.dumps(dict(prior=PRIOR,bias=[0.]*6)))
        rows,vel,beta=synthetic_rows(mass=0.)
        for sid,role in enumerate(['FIT_COMPLETE','FIT_INTERRUPTED','VALIDATION_INTERRUPTED','VALIDATION_COMPLETE']):
            a=dict(role=role,target_deg=10.,profile='constant',max_sim_s=10.,hold=True)
            if sid==1:a['fit_intervals_s']=[[1.,8.]]
            record.begin(a,'HANDLE')
            for i,row in enumerate(rows):
                row=dict(row,t=(sid*len(rows)+i)/240,q=np.zeros(13),joint_velocities=np.zeros(13),command_q_arm=np.zeros(7),command_q_velocity_arm=np.zeros(7),T_tcp=np.eye(4),aperture_m=.02,command_aperture_m=.02,contacts=[dict(force_n=.5)])
                row['telemetry']=dict(hinge_velocity_rad_s=float(vel[i]));record.append(row)
            record.end(dict(ok=sid in (0,3),status='TARGET_REACHED' if sid in (0,3) else ('TRANSMISSION_FAILURE' if adaptive else 'TIMEOUT'),safely_stopped=True,physical_transmission_valid=not adaptive,fit_prefix_transmission_valid=adaptive,fit_valid_until_s=8.5 if adaptive else 9.5))
        record.close()
        strict=production.update(p,p/'strict',plot=False);new=production.update(p,p/'quality',plot=False,allow_interrupted_fit=True)
        assert new['training_bins']>strict['training_bins'] and new['validation_bins']==strict['validation_bins']
        assert np.max(abs(np.array(new['constant_model']['parameters'])/beta-1))<.003
        provenance=new['segments'];assert provenance[1]['quality']['label']==('PREDECLARED_TRANSMITTING_FIT_PREFIX' if adaptive else 'PREDECLARED_SAFE_INTERRUPTED_FIT') and not provenance[1]['quality']['action_ok']
        assert provenance[2]['quality']['label']=='INCOMPLETE_VALIDATION_EXCLUDED'
        # Mutating every validation measurement cannot influence fitting/uncertainty.
        a,index=production.columns(p);mutated={k:v.copy() for k,v in a.items()};mask=np.isin(a['segment'],[3,4]);mutated['raw_ft'][mask]+=10.;mutated['door_q'][mask]+=.01;original=production.columns
        try:
            production.columns=lambda unused:(mutated,index)
            changed=production.update(p,p/'mutated_validation',plot=False,allow_interrupted_fit=True)
        finally:production.columns=original
        for key in ['parameters','conditional_std','conditional_covariance','information']:assert new['constant_model'][key]==changed['constant_model'][key]
        ops=OperationsV31.__new__(OperationsV31);ops.r=SimpleNamespace(root=ROOT,output=p,job=dict(allow_interrupted_fit=True));ready=threading.Event();ready.set();status=ops._update(ready)
        assert status['estimator_ready'] and '--allow-interrupted-fit' in status['command']
        live=json.loads((p/'model/estimate.json').read_text());assert live['constant_model']['parameters']==new['constant_model']['parameters']
        return dict(strict_bins=strict['training_bins'],quality_bins=new['training_bins'],validation_bins=new['validation_bins'],parameters=new['constant_model']['parameters'],synthetic_known_parameters=beta.tolist(),partial_action_ok_remains_false=True,incomplete_VALIDATION_excluded=True,validation_mutation_FIT_parameters_and_covariance_identical=True,async_worker=status,physical_episodes=0)

if __name__=='__main__':
    d=verify();Path(sys.argv[1]).write_text(json.dumps(d,indent=2));print(json.dumps(d,indent=2))
