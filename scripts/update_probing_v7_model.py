"""Atomic asynchronous V7 worker. No simulator/scene-truth input."""
import argparse,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet
from interaction_runtime.segment_quality import selection
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v7_estimator import VERSION,process,samples,analyze,prediction


def load_segments(folder,allow_interrupted_fit=False):
    arrays,index=columns(folder);settings=json.loads((folder/'identification_input.json').read_text());meta=index['metadata'];pieces=[];provenance=[]
    if meta['articulation']['joint_type']!=settings['prior']['joint_type']:raise ValueError('Recorded and prior joint types disagree')
    phases=np.array(meta['phases'])[arrays['phase']]
    for segment in index['segments']:
        ix=np.flatnonzero((arrays['segment']==segment['id']) & np.isin(phases,['OPEN_CONTINUE','PANEL_PUSH']))
        ix,quality=selection(segment,arrays,ix,allow_interrupted_fit)
        record=dict(id=segment['id'],role=segment['role'],quality=quality,blocks=[]);provenance.append(record)
        if not len(ix) or not segment['role'].startswith(('FIT','VALIDATION')):continue
        good=np.isfinite(arrays['joint_q'][ix]) & np.isfinite(arrays['joint_dq'][ix]) & np.isfinite(arrays['raw_ft'][ix]).all(1) & np.isfinite(arrays['sensor_pose'][ix]).all(1) & np.isfinite(arrays['tool_poses'][ix]).all((1,2)) & (arrays['ft_valid'][ix]>0)
        ix=ix[good];cuts=np.flatnonzero((np.diff(arrays['step'][ix])!=1)|(abs(np.diff(arrays['t'][ix])-1/240)>1e-7))+1
        for block,run in enumerate(np.split(ix,cuts)):
            if not len(run):continue
            label=segment['role']+'_%02d'%segment['id'];rows=rows_at(arrays,run,meta,label)
            d=process(rows,settings['prior'],settings['bias'],arrays['joint_dq'][run],native_coordinate=arrays['joint_q'][run])
            if d is None:record['blocks'].append(dict(records=len(run),status='insufficient local filter support'));continue
            d['block_id']=label+'#%d'%block;pieces.append(d)
            record['blocks'].append(dict(records=len(run),valid=int(d['valid'].sum()),processing=d['processing']))
    return pieces,settings,provenance


def update(folder,output=None,allow_interrupted_fit=False):
    began=time.perf_counter();folder=Path(folder);out=Path(output or folder/'model');out.mkdir(parents=True,exist_ok=True)
    if (out/'estimate.json').exists() and json.loads((out/'estimate.json').read_text()).get('estimator_version')!=VERSION:raise ValueError('Preserve previous estimator output')
    pieces,settings,provenance=load_segments(folder,allow_interrupted_fit);config=settings['estimation'];train=[];held=[]
    for d in pieces:
        ss=samples(d,config['domain_native'],config['intervals'])
        (held if d['segments'][0].startswith('VALIDATION') else train).extend(ss)
    result=dict(estimator_version=VERSION,configuration=config,segments=provenance,validation_bins=len(held),hidden_dynamics_used=False,tool_compensation_count=1,
        common_filter='SG121 torque, acceleration, velocity, K(B_j(q)*sign(native_qd))',validation_used_for_fit_or_selection=False,**analyze(train,config))
    if result['status']=='updated':
        for name in ['constant_model','extended_model']:
            m=result.get(name)
            if m:
                m['heldout_motion']={d['block_id']:prediction(d,m) for d in pieces if d['segments'][0].startswith('VALIDATION')}
                m['heldout_unfiltered_input_sensitivity']={}
                for d in pieces:
                    if d['segments'][0].startswith('VALIDATION'):
                        raw=dict(d,torque=d['unfiltered_torque']);m['heldout_unfiltered_input_sensitivity'][d['block_id']]=prediction(raw,m)
    result['worker_wall_s']=time.perf_counter()-began
    files=['scripts/update_probing_v7_model.py','scripts/xarm_ft_v7_estimator.py','scripts/xarm_ft_v31_estimator.py','interaction_runtime/segment_quality.py']
    result['source_sha256']={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files}
    if pieces:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,1,figsize=(8,5))
        for d in pieces:
            axes[0].plot(d['t'],d['q'],label=d['block_id']);axes[1].plot(d['t'],d['torque'],lw=.6)
        axes[0].set_ylabel('Joint coordinate ('+config['units']['coordinate']+')');axes[0].legend(fontsize=5)
        axes[1].set_ylabel('Compensated effort ('+config['units']['effort']+')');axes[1].set_xlabel('Simulation time (s)')
        fig.tight_layout();fig.savefig(out/'feedback.png',dpi=100);plt.close(fig)
    atomic_json(out/'estimate.json',result_packet(result))
    path=out/'updates.json';history=json.loads(path.read_text()) if path.exists() else []
    history.append(dict(training_bins=len(train),validation_bins=len(held),parameters=result.get('selected',{}).get('parameters'),worker_wall_s=result['worker_wall_s']))
    atomic_json(path,history);return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--output',type=Path);p.add_argument('--allow-interrupted-fit',action='store_true');a=p.parse_args()
    r=update(a.folder,a.output,a.allow_interrupted_fit);print(json.dumps(dict(status=r['status'],version=VERSION,worker_wall_s=r['worker_wall_s'])))
