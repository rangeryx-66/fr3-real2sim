"""Measurement-only incremental block worker. Hidden plant modules are not imported."""
import argparse,json,pickle
from pathlib import Path
import numpy as np
from xarm_ft_v1_estimator import local_process,analyze,candidate_information,KEYS


def plots(data,model,folder):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from xarm_ft_v1_estimator import features
    fig,ax=plt.subplots(4,1,figsize=(13,11),sharex=True)
    ax[0].plot(data['t'],np.rad2deg(data['q']),lw=.7);ax[0].set_ylabel('known angle deg')
    ax[1].plot(data['t'],data['torque'],lw=.4);ax[1].set_ylabel('wrist F/T torque Nm')
    ax[2].plot(data['t'],data['qd'],lw=.5);ax[2].set_ylabel('rad/s')
    ax[3].plot(data['t'],data['qdd'],lw=.5);ax[3].set_ylabel('rad/s2');ax[3].set_xlabel('simulation seconds')
    for a in ax:a.grid(alpha=.25)
    fig.suptitle('Actual multi-state interaction; strict segment-local derivatives; all native samples retained')
    fig.tight_layout();fig.savefig(folder/'FT_COVERAGE.png',dpi=130);plt.close(fig)
    samples=data['samples'];fig,ax=plt.subplots(2,2,figsize=(13,8))
    for kind,m in model.get('models',{}).items():
        residual=np.array([s['tau'] for s in samples])-features(samples,kind)@m['parameters']
        for sign,marker in [(1,'o'),(-1,'x')]:
            ix=np.array([s['direction']*sign>.5 for s in samples]);q=np.rad2deg([s['q'] for s in samples]);v=np.array([s['qd'] for s in samples]);acc=np.array([s['qdd'] for s in samples])
            ax[0,0].scatter(q[ix],residual[ix],s=3,marker=marker,label=kind+(' open' if sign>0 else ' close'))
            ax[0,1].scatter(v[ix],residual[ix],s=3,marker=marker)
            ax[1,0].scatter(acc[ix],residual[ix],s=3,marker=marker)
    ax[0,0].set_xlabel('angle deg');ax[0,0].legend(fontsize=7);ax[0,1].set_xlabel('velocity rad/s');ax[1,0].set_xlabel('acceleration rad/s2')
    for a in ax.flat:a.grid(alpha=.25);a.set_ylabel('residual Nm')
    for c in model.get('coverage',[]):ax[1,1].scatter([np.mean(c['angle_region_deg'])],[c['acceleration_rms']],label=str(c['angle_region_deg']))
    ax[1,1].set_xlabel('angular region deg');ax[1,1].set_ylabel('measured acceleration RMS rad/s2')
    fig.tight_layout();fig.savefig(folder/'RESIDUALS_INFORMATION.png',dpi=130);plt.close(fig)


def main(folder):
    folder=Path(folder);settings=json.loads((folder/'INPUT.json').read_text())
    with (folder/'measurements.pkl').open('rb') as f:rows=pickle.load(f)
    data,provenance=local_process(rows,settings['prior'],settings['bias'])
    if data is None:
        (folder/'IDENTIFIED_MODEL.json').write_text(json.dumps(dict(status='insufficient local derivative support',configuration=settings),indent=2));return
    np.savez_compressed(folder/'BLOCK_DATA.npz',**{k:data[k] for k in KEYS})
    (folder/'BLOCK_SAMPLES.json').write_text(json.dumps(data['samples']))
    pieces=[];samples=[]
    for previous in settings.get('previous_packets',[]):
        p=Path(previous)
        if not (p/'BLOCK_DATA.npz').exists():continue
        pieces.append(dict(np.load(p/'BLOCK_DATA.npz',allow_pickle=True)));samples+=json.loads((p/'BLOCK_SAMPLES.json').read_text())
    pieces.append(data);samples+=data['samples'];joined={k:np.concatenate([v[k] for v in pieces]) for k in KEYS};joined['samples']=samples
    result=analyze(joined);result.update(prior=settings['prior'],segment_provenance=provenance,previous_packets=settings.get('previous_packets',[]))
    result['candidate_information']=candidate_information(joined,result,settings.get('candidate_blocks',{}))
    (folder/'IDENTIFIED_MODEL.json').write_text(json.dumps(result,indent=2));np.savez_compressed(folder/'CUMULATIVE_DATA.npz',**{k:joined[k] for k in KEYS});(folder/'CUMULATIVE_SAMPLES.json').write_text(json.dumps(samples))
    plots(joined,result,folder)
    print(json.dumps(dict(selected=result.get('selected_model'),coverage=result.get('coverage'),models={k:dict(parameters=v['parameters'],supported=v['supported']) for k,v in result.get('models',{}).items()})))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');main(p.parse_args().folder)
