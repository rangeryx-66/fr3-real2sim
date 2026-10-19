"""Completed-segment V6 worker; inherited V3.1 measurement chain exactly once."""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet
from update_probing_v31_model import load_segments
from xarm_ft_v6_estimator import VERSION,CONFIG,samples_from_piece,analyze,prediction


def update(folder,output=None,plot=True,allow_interrupted_fit=False):
    start=time.perf_counter();folder=Path(folder);out=Path(output or folder/'model');out.mkdir(parents=True,exist_ok=True)
    if (out/'estimate.json').exists() and json.loads((out/'estimate.json').read_text()).get('estimator_version')!=VERSION:raise ValueError('new output required; preserve frozen estimates')
    pieces,_,_,provenance=load_segments(folder,allow_interrupted_fit=allow_interrupted_fit);train=[];test=[]
    for d in pieces:(test if str(d['segments'][0]).startswith('VALIDATION') else train).extend(samples_from_piece(d))
    result=dict(estimator_version=VERSION,configuration=CONFIG,training_bins=len(train),validation_bins=len(test),segments=provenance,hidden_dynamics_used=False,tool_compensation_count=1,friction_basis='K(B_j(q)*sign(native_qd)); not B_j(K(q))*sign(K(qd))',complete_validation_excluded_from_fit_uncertainty_and_selection=True,**analyze(train))
    if result['status']=='updated':
        for name in ['constant_model','extended_model']:
            m=result.get(name)
            if m:m['heldout_motion']={d['block_id']:prediction(d,m) for d in pieces if str(d['segments'][0]).startswith('VALIDATION')}
        # Active probing suggestions contain measured-only information surrogates, not motion permission.
        from xarm_ft_v6_estimator import matrix,basis
        m=result['extended_model'];result['active_information']=[]
        if m:
            resolution=m['resolution_deg'];X=np.array([[s['qdd'],s['qd']]+s['angle_features'][str(resolution)] for s in train]);scale=np.array([1.,.25]+[.045]*(X.shape[1]-2));F=(X*scale).T@(X*scale);current=float(pieces[-1]['q'][-1]);vel=max(1e-4,float(np.quantile(np.abs(np.concatenate([d['native_qd'][d['valid']] for d in pieces if str(d['segments'][0]).startswith('FIT')])),.9)))
            for end_deg in [5.,10.,15.,20.,25.,30.,35.,40.,45.]:
                for factor in [.5,1.]:
                    qs=np.linspace(current,np.deg2rad(end_deg),32);direction=float(np.sign(qs[-1]-qs[0]));A=np.c_[.02*np.cos(np.linspace(0,4*np.pi,32)),np.full(32,direction*vel*factor),basis(qs,resolution)*direction]*scale
                    gain=np.linalg.slogdet(F+A.T@A+1e-8*np.eye(len(scale)))[1]-np.linalg.slogdet(F+1e-8*np.eye(len(scale)))[1]
                    result['active_information'].append(dict(target_deg=end_deg,relative_speed=factor,logdet_gain_surrogate=float(gain),estimated_duration_s=float(abs(qs[-1]-qs[0])/(vel*factor)),requires_current_state_IK_collision_contact_validation=True,unvisited_curve_is_unknown=True,information_prior='full local basis with fixed 1e-8 numerical prior; not calibrated entropy'))
    result['worker_wall_s']=time.perf_counter()-start;result['source_sha256']={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in ['scripts/update_probing_v6_model.py','scripts/xarm_ft_v6_estimator.py','scripts/update_probing_v31_model.py','scripts/xarm_ft_v31_estimator.py']}
    if plot and pieces:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(3,1,figsize=(8,7))
        for d in pieces:ax[0].plot(d['t'],np.rad2deg(d['q']),label=str(d['segments'][0]))
        ax[0].set_ylabel('Measured angle (deg)');ax[0].legend(fontsize=6)
        for name in ['constant_model','extended_model']:
            m=result.get(name)
            if m:
                c=m['resistance_curve'];ax[1].plot(c['angle_deg'],c['resistance_nm'],label=name)
        ax[1].set(xlabel='Angle (deg)',ylabel='Estimated C(q) (Nm)');ax[1].legend()
        if result.get('constant_model'):
            cm=result['constant_model'];X=matrix(train,0);y=np.array([s['tau'] for s in train]);colors=np.array([s['qd'] for s in train]);ax[2].scatter(np.rad2deg([s['q'] for s in train]),(y-X@cm['parameters'])*1000,c=colors,cmap='coolwarm',s=7)
        ax[2].set(xlabel='Measured FIT angle (deg)',ylabel='Constant-model residual (mNm)');fig.tight_layout();fig.savefig(out/'feedback.png',dpi=100);plt.close(fig)
    atomic_json(out/'estimate.json',result_packet(result));h=out/'updates.json';history=json.loads(h.read_text()) if h.exists() else [];history.append(dict(training_bins=len(train),validation_bins=len(test),selected=result.get('selected_model'),worker_wall_s=result['worker_wall_s']));atomic_json(h,history)
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--output',type=Path);p.add_argument('--allow-interrupted-fit',action='store_true');a=p.parse_args();r=update(a.folder,a.output,allow_interrupted_fit=a.allow_interrupted_fit);print(json.dumps(dict(status=r['status'],version=VERSION,worker_wall_s=r['worker_wall_s'])))
