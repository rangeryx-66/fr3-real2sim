"""Repeat/zero-drive calibration; never consumes candidate-fitting residuals."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
from .metrics import onset


def read(p):return json.loads(Path(p).read_text())
def digest(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def extract(root,config,output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    destination=output/'calibration_observations.json'
    if destination.exists():return read(destination)
    groups=[];sources=[]
    for item in config['robot_only_logs']:
        path=Path(root)/item;log=read(path)
        groups.append({'name':item,'kind':'robot_only','time_s':log['time_s'],
                       'T':log['signals']['ee_T_world_tcp'],'command_hash':log['provenance']['command_applied_sha256']})
        sources.append({'path':str(path),'sha256':digest(path),'role':'calibration only'})
    for item in config['contact_repeats']+[config['zero_drive']]:
        path=Path(root)/item/'observations.json'
        print('Calibration source',item,flush=True)
        rows=read(path);phases=['ZERO_PROBE'] if item==config['zero_drive'] else ['FORCE_HOLD','COMPLIANT_SETTLE','EXPLORATORY']
        for phase in phases:
            chosen=[r for r in rows if r['phase']==phase]
            if len(chosen)<config['filter_window_samples']*2:continue
            # No object transform, angle, force manifold or fitted axis is exported.
            # Commands are used only to certify comparability of repeat prefixes.
            command=[{k:r.get('constrained_drive',{}).get(k) for k in
                      ['direction_world','reference_world_m','active','drive_reference_world_m']} for r in chosen]
            groups.append({'name':item+'/'+phase,'kind':'zero_drive' if phase=='ZERO_PROBE' else 'contact_repeat',
                           'phase':phase,'time_s':[r['t']-chosen[0]['t'] for r in chosen],
                           'T':[r['T_tcp'] for r in chosen],
                           'command_hash':hashlib.sha256(json.dumps(command,sort_keys=True).encode()).hexdigest(),
                           'command_certificate_complete':all(c['direction_world'] is not None and c['drive_reference_world_m'] is not None for c in command)})
        sources.append({'path':str(path),'sha256':digest(path),'role':'repeat/zero-drive observations, not model residuals'})
        del rows
    data={'groups':groups,'sources':sources,'GT_inputs':False}
    destination.write_text(json.dumps(data));return data


def _innovations(group,config):
    T=np.asarray(group['T']);t=np.asarray(group['time_s']);R=T[:,:3,:3]
    xyz=T[:,:3,3];rv=Rotation.from_matrix(R@R[0].T).as_rotvec()
    w=config['filter_window_samples'];order=config['filter_polynomial_order']
    pos=xyz-savgol_filter(xyz,w,order,axis=0)
    rot=rv-savgol_filter(rv,w,order,axis=0)
    vel=np.gradient(pos,t,axis=0)
    local=lambda x:np.einsum('nji,nj->ni',R,x)
    n=w//2
    return np.c_[local(pos),local(rot),local(vel)][n:-n]


def calibrate(data,config,output):
    groups=data['groups'];samples=[];rows=[];repeat_rows=[]
    for g in groups:
        e=_innovations(g,config);e-=e.mean(0);samples.append(e)
        rows.append({'source':g['name'],'count':len(e),'highpass_rms':np.sqrt(np.mean(e*e,axis=0)).tolist()})
    # Genuine repeat residuals retain low-frequency repeat differences. Identical
    # deterministic replays are reported as zero, never treated as infinite precision.
    for i,a in enumerate(groups):
        for b in groups[i+1:]:
            if a['kind']!=b['kind'] or a.get('phase')!=b.get('phase'):continue
            if a['command_hash']!=b['command_hash'] or len(a['T'])!=len(b['T']):continue
            if a['kind']=='contact_repeat' and not (a.get('command_certificate_complete') and b.get('command_certificate_complete')):continue
            A=np.asarray(a['T']);B=np.asarray(b['T']);R=A[:,:3,:3];t=np.asarray(a['time_s'])
            d=B[:,:3,3]-A[:,:3,3];dr=Rotation.from_matrix(B[:,:3,:3]@R.transpose(0,2,1)).as_rotvec()
            local=lambda x:np.einsum('nji,nj->ni',R,x)
            e=np.c_[local(d),local(dr),local(np.gradient(d,t,axis=0))]/np.sqrt(2.)
            samples.append(e)
            repeat_rows.append({'a':a['name'],'b':b['name'],'rms':np.sqrt(np.mean(e*e,axis=0)).tolist(),'commands_match':True})
    # Equal-size temporal blocks estimate an envelope of observed repeatability.
    # No model fitting residual, TEST object, or future test log enters this API.
    blocks=[];w=config['filter_window_samples']
    for e in samples:
        blocks.extend(e[i:i+w] for i in range(0,len(e)-w+1,w))
    block_rms=np.asarray([np.sqrt(np.mean(b*b,axis=0)) for b in blocks])
    scales=np.quantile(block_rms,config['repeatability_quantile'],axis=0)
    if np.any(scales<=0):raise ValueError('CALIBRATION_UNOBSERVABLE_ZERO_NOISE_DIMENSION')
    X=np.concatenate(samples)/scales
    X-=X.mean(0);C=X.T@X/len(X)
    sd=np.sqrt(np.diag(C));corr=C/np.outer(sd,sd)
    # OAS covariance shrinkage with temporal blocks as independent sample count.
    n=len(blocks);dim=9;mu=np.trace(corr)/dim;alpha=np.mean(corr*corr)
    denominator=(n+1)*(alpha-mu*mu/dim)
    shrink=min(1.,(alpha+mu*mu)/denominator) if denominator>0 else 1.
    corr=(1-shrink)*corr+shrink*mu*np.eye(dim)
    cov=corr*np.outer(scales,scales)
    sigma_p=float(np.sqrt(np.trace(cov[:3,:3])/3))
    threshold=config['onset_sigma_multiplier']*sigma_p
    # Propagate observed pose fluctuations through the onset detector at both
    # existing probe speeds, with no measured object-response curve as input.
    rng=np.random.default_rng(config['seed']);dt=config['source_timestep_s'];t=np.arange(0,4,dt)
    active=(t>=1)&(t<3.5);noise=np.concatenate(samples)[:,:3];onsets=[]
    for speed in [.00025,.0005]:
        values=[]
        for _ in range(config['bootstrap_count']):
            ids=rng.integers(0,len(blocks),int(np.ceil(len(t)/w)))
            jitter=np.concatenate([blocks[j][:,:3] for j in ids])[:len(t)]
            p=jitter.copy();u=np.maximum(t-1.,0.)
            p[:,0]+=speed*np.where(u<2,u*u/4,u-1)
            v=onset(t,p,active,threshold,config['onset_hold_s'])
            if v is not None:values.append(v)
        onsets.append({'speed_m_s':speed,'valid_count':len(values),
                       'onset_std_s':float(np.std(values)) if values else None,
                       'onset_95_width_s':float(np.ptp(np.quantile(values,[.025,.975]))) if values else None})
    if any(r['valid_count']<.95*config['bootstrap_count'] for r in onsets):raise ValueError('CALIBRATION_ONSET_NOT_OBSERVABLE')
    delay_scale=max(dt,max(r['onset_std_s'] for r in onsets))
    result={'response_covariance':cov.tolist(),'component_scales':scales.tolist(),
            'order':['p_x','p_y','p_z','rotation_x','rotation_y','rotation_z','v_x','v_y','v_z'],
            'coordinate_frame':'instantaneous measured EE frame','start_time_scale_s':delay_scale,
            'onset_threshold_m':threshold,'onset_hold_s':config['onset_hold_s'],
            'block_count':n,'oas_shrinkage':float(shrink),'source_innovations':rows,'repeat_pairs':repeat_rows,
            'onset_bootstrap':onsets,'source_files':data['sources'],'configuration':config,
            'hardware_accuracy_claim':False,'model_residual_used':False,'test_asset_used':False,
            'semantics':'empirical repeatability and short-timescale fluctuation envelope; not true sensor accuracy or model-discrepancy allowance'}
    result['calibration_sha256']=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()
    Path(output,'selection_noise.json').write_text(json.dumps(result,indent=2))
    return result
