"""Retrospective A/B/C comparison under privileged native object-state feedback.

This is explicitly NOT a visual-estimator validation or fresh independent
experiment. A uses F/T; B uses its previously frozen effort-only cache; C has
no observed input and cannot identify absolute physical parameter scale.
No object truth is read. Model complexity/configuration is fixed, not selected
on the old validation trajectories now designated development diagnostics.
"""
import argparse,copy,hashlib,json
from pathlib import Path
import numpy as np
from update_probing_v7_model import load_segments
from update_prismatic_v73_model import qualify
from xarm_ft_v7_estimator import samples,fit,prediction
from interaction_runtime.serialization import result_packet


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def errors(x):
    return dict(samples=len(x),RMSE=float(np.sqrt(np.mean(x*x))),MAE=float(np.mean(abs(x))),bias=float(np.mean(x))) if len(x) else dict(samples=0,RMSE=None,MAE=None,bias=None)


def summarize_strata(q,v,acc,error,kind):
    factor=180/np.pi if kind=='revolute' else 1000
    bands=[(0,30),(30,60),(60,75),(75,100)] if kind=='revolute' else [(0,50),(50,100),(100,150),(150,200)]
    out={f'position_{lo}_{hi}':errors(error[(q*factor>=lo)&(q*factor<hi)])for lo,hi in bands}
    for name,mask in [('forward',v>0),('reverse',v<0),('abs_speed_below_.01',abs(v)<.01),('abs_speed_at_least_.01',abs(v)>=.01),('abs_acc_below_.005',abs(acc)<.005),('abs_acc_at_least_.005',abs(acc)>=.005)]:out[name]=errors(error[mask])
    return out


def half_step_sensitivity(piece,model):
    mass,damping,resistance=model['parameters']
    if mass<=1e-8:return dict(status='unsupported inertial term')
    q=float(piece['q'][0]);v=float(piece['native_qd'][0]);prediction=[q]
    for j in range(1,len(piece['t'])):
        dt=(piece['t'][j]-piece['t'][j-1])/2;effort=piece['torque'][j-1]
        for _ in range(2):
            momentum=mass*v+dt*effort
            v=float(np.sign(momentum)*max(abs(momentum)-dt*resistance,0)/(mass+dt*damping));q+=dt*v
        prediction.append(q)
    error=np.asarray(prediction)-piece['q']
    return dict(status='predicted',coordinate_rmse=float(np.sqrt(np.mean(error*error))),
        final_coordinate_error=float(error[-1]),scope='Numerical half-steps with zero-order-held same measured effort; no extra excitation or state correction.')


def compare(episode,cache,output):
    assert (episode/'PROCESS_COMPLETION.json').exists()
    pieces,settings,quality=load_segments(episode,True)
    kind=settings['prior']['joint_type'];config=settings['estimation']
    with np.load(cache/'effort_only.npz',allow_pickle=False)as f:bc={k:f[k]for k in f.files}
    paired=[];train={'A':[],'B':[]};rows=[];series=[]
    for d in pieces:
        # Existing eligibility is shared; native joint-state access is privileged
        # and disclosed. B's robot compensation and virtual-work estimator have
        # already been frozen without these object-state or F/T channels.
        if kind=='prismatic':qualify(d)
        ii=np.searchsorted(bc['t'],d['t'])
        assert np.all(ii<len(bc['t'])) and np.max(abs(bc['t'][ii]-d['t']))<1e-10
        b=bc['generalized_effort'][ii];common=np.isfinite(b)&np.isfinite(d['torque'])
        edge=d['processing']['boundary_exclusion_samples'];support=np.arange(len(common))
        common &= (support>=edge)&(support<len(common)-edge)
        assert not np.any(np.diff(ii)!=1), 'sampling gaps must remain separate pieces'
        valid=d['valid']&common
        rr=b[valid]-d['torque'][valid];q=d['q'][valid];v=d['native_qd'][valid];acc=d['qdd'][valid]
        rows.append(dict(block=d['block_id'],role=str(d['segments'][0]),comparison_to_FT=errors(rr),
            strata=summarize_strata(q,v,acc,rr,kind),native_qdd_RMS=float(np.sqrt(np.mean(acc**2)))if len(acc)else None,
            native_velocity_RMS=float(np.sqrt(np.mean(v**2)))if len(v)else None))
        # Filter has already run; masks and timestamps are identical for A/B.
        p=dict(d,valid=valid.copy());pb=dict(p,torque=b.copy())
        for label,piece in [('A',p),('B',pb)]:
            ss=samples(piece,config['domain_native'],[])
            if str(d['segments'][0]).startswith('FIT'):train[label].extend(ss)
        paired.append((p,pb,common))
        series.append(dict(block=d['block_id'],t=d['t'],q=d['q'],A=d['torque'],B=b))
    assert len(train['A'])==len(train['B'])
    models={label:fit(records,config,intervals=0,smoothness=0)for label,records in train.items()}
    frozen=dict(schema='v85-native-state-development-only',episode_id=episode.name,joint_type=kind,
        F_T_input_used_by_A=True,F_T_input_used_by_B_effort_estimator=False,
        B_fit_scope='Retrospective common-cohort fit, using privileged native object state; not the isolated sensorless estimator or visual validation.',
        B_robot_compensation_sha256=sha(cache.parent/'ROBOT_COMPENSATION_FROZEN.json'),
        B_effort_cache_sha256=sha(cache/'effort_only.npz'),
        formal_independent_validation=False,old_validation_reclassified='DEVELOPMENT_DIAGNOSTICS_ALREADY_EXAMINED',
        hidden_object_GT_used=False,common_FIT_bins=len(train['A']),configuration=config,models=models,
        C=dict(status='STRUCTURALLY_UNIDENTIFIABLE_WITH_UNOBSERVED_ACTIVE_CONTACT_INPUT',
               parameters=None,prediction_metrics=None,reason='Scaling inertia, damping, resistance and the unobserved force together preserves the same active kinematics.'))
    output.mkdir(parents=True,exist_ok=False)
    modelpath=output/'MODELS_FROZEN.json';modelpath.write_text(json.dumps(result_packet(frozen),indent=2,allow_nan=False))
    scores=[]
    for pa,pb,common in paired:
        if not str(pa['segments'][0]).startswith('VALIDATION'):continue
        # Both methods start at the same first common supported state. No later
        # real-state correction. Unsupported/missing horizons remain explicit.
        ids=np.flatnonzero(common);runs=np.split(ids,np.flatnonzero(np.diff(ids)!=1)+1)
        for run in runs:
            if len(run)<2:continue
            row=dict(block=pa['block_id'],samples=len(run),duration_s=float(pa['t'][run[-1]]-pa['t'][run[0]]),models={})
            for label,source in [('A',pa),('B',pb)]:
                piece={k:(v[run]if isinstance(v,np.ndarray)and v.shape[0]==len(common) else v)for k,v in source.items()}
                piece['processing']=dict(piece['processing'],boundary_exclusion_samples=0)
                # Centered derivative support was excluded by common already.
                full=prediction(piece,models[label]);short={}
                for horizon in [.5,2.,5.]:
                    end=np.searchsorted(piece['t'],piece['t'][0]+horizon,side='right')
                    if end<2 or piece['t'][-1]-piece['t'][0]+1e-8<horizon:short[str(horizon)]=dict(status='insufficient common horizon');continue
                    cut={k:(v[:end]if isinstance(v,np.ndarray)and v.shape[0]==len(run)else v)for k,v in piece.items()}
                    short[str(horizon)]=prediction(cut,models[label])
                row['models'][label]=dict(full_rollout=full,short_horizons_s=short,numerical_half_step=half_step_sensitivity(piece,models[label]))
            scores.append(row)
    q=np.concatenate([d['q'][d['valid']]for d,_,_ in paired]);v=np.concatenate([d['native_qd'][d['valid']]for d,_,_ in paired]);acc=np.concatenate([d['qdd'][d['valid']]for d,_,_ in paired]);error=np.concatenate([b['torque'][a['valid']]-a['torque'][a['valid']]for a,b,_ in paired])
    report=dict(schema='v85-input-comparison-development-v1',formal_independent_validation=False,
        state_source='PRIVILEGED_SIMULATOR_JOINT_STATE_NOT_RGBD',joint_type=kind,
        generalized_effort_units='N m'if kind=='revolute'else'N',
        model_sha256=sha(modelpath),common_FIT_bins=len(train['A']),force_disagreement_to_FT=errors(error),
        disagreement_strata=summarize_strata(q,v,acc,error,kind),segments=rows,predictions=scores,
        FT_is_not_GT=True,parameters_GT_evaluated_here=False,quality=quality,
        diagnostic_limitations=['Common cohort uses legacy F/T availability flag; it is not a certified F/T-free visual model fit.',
        'B force-cache computation and robot calibration never read F/T or object native q.',
        'No candidate selection, refit or structure update after these scores.'])
    (output/'COMPARISON.json').write_text(json.dumps(result_packet(report),indent=2,allow_nan=False))
    # Plot inputs/labels are post-hoc only, never sent to a physical Agent.
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(10,6),layout='constrained')
    for s in series:
        axes[0].plot(s['t'],s['A'],lw=.6,color='tab:blue');axes[0].plot(s['t'],s['B'],lw=.6,color='tab:orange')
        axes[1].plot(s['t'],s['B']-s['A'],lw=.6)
    axes[0].set_ylabel('Generalized effort ('+report['generalized_effort_units']+')');axes[0].set_title('A: F/T blue; B: frozen effort-only orange | retrospective native-state diagnostic')
    axes[1].set_ylabel('B minus A');axes[1].set_xlabel('Advancing simulation time (s); gaps retained')
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(output/'FORCE_COMPARISON.png',dpi=140);plt.close(fig)
    return dict(episode=episode.name,force_disagreement=report['force_disagreement_to_FT'],models={k:dict(parameters=m['parameters'],conditional_std=m['conditional_std'],condition=m['scaled_condition'])for k,m in models.items()},predictions=scores)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('cache',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    print(json.dumps(result_packet(compare(a.episode,a.cache,a.output)),indent=2))
