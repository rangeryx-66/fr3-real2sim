"""Read-only archived contact-state diagnosis, never a replacement estimator."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from update_probing_v7_model import load_segments
from xarm_ft_v7_estimator import samples,fit
from diagnose_probing_v3 import columns


def inspect(folder,out):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a,index=columns(folder);pieces,settings,_=load_segments(folder,True);c=settings['estimation'];train=[]
    # All FIT trajectory exclusions are reported by diagnose_prismatic_v73.
    # This board visualizes the diagnosed second trajectory, not a new fit rule.
    target=next(d for d in pieces if d['segments'][0]=='FIT_OPEN_ACCEL_02')
    for d in pieces:
        if d['segments'][0].startswith('FIT') and d is not target:train.extend(samples(d,c['domain_native'],c['intervals']))
    model=fit(train,c);B=np.asarray(model['parameters']);d=target;t=d['t']-d['t'][0];ix=np.searchsorted(a['t'],d['t']);axis=np.array(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    residual=d['torque']-np.c_[d['qdd'],d['qd'],d['direction']]@B
    ci=json.loads((folder/'contact_evidence/index.json').read_text());data={}
    for ch in ci['files']:
        with np.load(folder/'contact_evidence'/ch['file']) as z:
            for k in z.files:data.setdefault(k,[]).append(z[k])
    data={k:np.concatenate(v) for k,v in data.items()};ct=data['step']*ci['physics_dt_s']-d['t'][0];inside=(ct>=0)&(ct<=t[-1])&(data['kind']==0)
    fig,ax=plt.subplots(5,1,figsize=(10,11),sharex=True);ax[0].plot(t,residual,lw=.5,label='wrist effort minus other-FIT model');ax[0].set_ylabel('Residual N')
    ax[1].plot(t,d['qdd'],lw=.5,label='measured SG121 acceleration');ax[1].set_ylabel('m/s²')
    measured=a['tcp_pose'][ix,:3]@axis;command=a['command_TCP_pose'][ix,:3]@axis
    ax[2].plot(t,(command-measured)*1000,label='command - measured TCP')
    ax[2].plot(t,((measured-measured[0])-(d['q']-d['q'][0]))*1000,label='measured TCP - drawer delta');ax[2].set_ylabel('Separate errors mm')
    for f,color in [(0,'blue'),(1,'orange')]:
        ax[3].plot(t,a['pad_load'][ix,f],c=color,lw=.5,label=('L' if f==0 else 'R')+' raw normal')
        good=inside&(data['finger']==f);ax[4].scatter(ct[good],data['point_pad'][good,1]*1000,s=.4,c=color,label=('L' if f==0 else 'R')+' actual longitudinal point')
    ax[3].set_ylabel('Pad normal N');ax[4].set_ylabel('Pad longitudinal mm');ax[4].set_xlabel('Seconds within second FIT trajectory')
    windows=[]
    for start in np.arange(0,t[-1]-2+1e-6,2):
        valid=(t>=start)&(t<start+2)&d['valid'];points=inside&(ct>=start)&(ct<start+2)
        row=dict(interval_s=[float(start),float(start+2)],q_range_m=[float(d['q'][valid].min()),float(d['q'][valid].max())] if valid.any() else None,pad=[])
        for f in (0,1):
            hit=points&(data['finger']==f)
            row['pad'].append(dict(points=int(hit.sum()),centroid_pad_m=data['point_pad'][hit].mean(0).tolist() if hit.any() else None,longitudinal_margin_p10_m=float(np.quantile(data['longitudinal_margin_m'][hit],.1)) if hit.any() else None,lateral_margin_p10_m=float(np.quantile(data['lateral_margin_m'][hit],.1)) if hit.any() else None,targets=[ci['identities'][v] for v in np.unique(data['target'][hit])]))
        windows.append(row)
    for axisplot in ax:axisplot.legend(fontsize=7);axisplot.grid(alpha=.2);axisplot.axvline(6,color='gray',ls=':')
    fig.suptitle('45290_0 archived second FIT: contact-state change versus tracking\nDiagnostic other-FIT model, no hidden truth; positive-normal samples are not pressure areas')
    fig.tight_layout();out.mkdir(exist_ok=True,parents=True);fig.savefig(out/'45290_contact_transition.png',dpi=130);plt.close(fig)
    (out/'45290_CONTACT_TRANSITION.json').write_text(json.dumps(dict(diagnostic_model=B.tolist(),not_production_replacement=True,hidden_truth_used=False,windows=windows,limitations=['full moving-link pose and non-pad object/frame contact reactions not retained in original compact recorder','cannot uniquely separate guide constraint/compliance/solver effects from these archives']),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('output',type=Path);a=p.parse_args();inspect(a.folder,a.output)
