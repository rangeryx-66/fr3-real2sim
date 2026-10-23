"""Post-seal full-range summaries; never imported by the live Agent/runtime."""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np

def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def motion_figure(p, root, output):
    """Plot measurements only; rendering never changes retained samples."""
    sys.path[:0]=[str(root),str(root/'scripts')]
    from diagnose_probing_v3 import columns
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a,index=columns(p)
    joint=index['metadata']['articulation']['joint_type']
    scale=180/np.pi if joint=='revolute' else 1000.
    unit='deg' if joint=='revolute' else 'mm'
    target=90. if joint=='revolute' else 150.
    fig,axes=plt.subplots(1,2,figsize=(11.6,3.5),layout='constrained')
    for ax in axes:
        ax.plot(a['t'],a['joint_q']*scale,color='#236fa1',lw=1.3,label='Measured joint state')
        ax.set(xlabel='Physics simulation time (s)',ylabel='Joint coordinate ('+unit+')')
        ax.grid(alpha=.2)
    axes[0].axhline(target,color='#c54845',ls='--',lw=1,label='Required full-range gate')
    axes[0].set_ylim(-target*.035,target*1.08)
    axes[0].set_title('Full-range criterion: NOT PASS')
    if 'reference_q' in a:
        mask=a['segment']>0
        reference=np.where(mask,a['reference_q']*scale,np.nan)
        axes[1].plot(a['t'],reference,color='#d08025',lw=.8,alpha=.8,label='Active segment reference')
    roles=set()
    for s in index['segments']:
        ix=np.flatnonzero(a['segment']==s['id'])
        if not len(ix):continue
        role='VALIDATION' if s['role'].startswith('VALIDATION') else 'FIT'
        if s.get('result',{}).get('status')!='TARGET_REACHED':role='Interrupted FIT'
        color={'VALIDATION':'#45ac77','FIT':'#76b7dd','Interrupted FIT':'#edb968'}[role]
        axes[1].axvspan(a['t'][ix[0]],a['t'][ix[-1]],color=color,alpha=.2,label=role if role not in roles else None)
        roles.add(role)
    axes[1].set_title('Same-scene measured range (zoom)')
    for ax in axes:ax.legend(loc='upper left',fontsize=7)
    fig.suptitle(p.name.replace('v81_full_range_blind_','').replace('_runtime_repair','')+' | fresh blind dynamics',fontsize=11)
    fig.savefig(output,dpi=145)
    plt.close(fig)

def summarize(p, root):
    sys.path[:0]=[str(root),str(root/'scripts')]
    from agentic_interaction_v81.evaluate import evaluate
    from diagnose_probing_v3 import columns
    assert json.loads((p/'PROCESS_COMPLETION.json').read_text())['actor_exited']
    sealed=json.loads((p/'ONLINE_FROZEN_MANIFEST.json').read_text())
    for f,h in sealed['sha256'].items():
        assert digest(p/f)==h, ('POST_SEAL_MUTATION',f)
    d=evaluate(p);a,index=columns(p)
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text())
    runtime=json.loads((p/'RUNTIME_RESULT.json').read_text())
    joint=d['joint_type'];scale=180/np.pi if joint=='revolute' else 1000.
    physical=[]
    for s in d['segments']:
        r=s['result'];physical.append(dict(id=s['id'],role=s['role'],
            start=s['start'],end=s['end'],minimum=s['minimum'],maximum=s['maximum'],
            target_reached=s['target_reached'],records=s['records'],status=r.get('status'),
            physical_duration_s=r.get('elapsed_sim_s'),physical_transmission_valid=r.get('physical_transmission_valid'),
            transmission=r.get('transmission'),tracking_limited_fraction=r.get('tracking_limited_fraction')))
    validations=[]
    for seal in d['prevalidation_sealed_models']:
        sid=int(Path(seal['seal_file']).stem)
        primary={k:v for k,v in seal['heldout'].items()if k.split('#')[0].endswith('_%02d'%sid)}
        validations.append(dict(seal_file=seal['seal_file'],model_sha256=seal['model_sha256'],
            parameters=seal['parameters'],conditional_std=seal['conditional_std'],
            heldout=primary,validation_used_for_fit=False,
            note='Only the matching pre-execution seal is primary; future/other-model replays omitted.'))
    acquisitions=[];relocations=[];decisions=[];rejections=[]
    for e in trace:
        c=e['command'];r=e['result'];op=c['operation']
        decisions.append(dict(sequence=c['sequence'],operation=op,reason=c.get('reason'),
            inspected_image_sha256=c.get('inspected_image_sha256'),wall_s=e['wall_s'],
            unloaded_wait_s=e.get('unloaded_wait_s'),reported_status=r.get('status'),reason_returned=r.get('reason')))
        if op=='attempt_grasp_and_probe':
            acquisitions.append(dict(sequence=c['sequence'],physical_grasp_started=r.get('physical_grasp_started'),
                acquisition=r.get('acquisition'),family=r.get('family'),
                requested_preload_n=c.get('native_g1_preload_request_n'),
                continuations=r.get('continuations'),
                safe_exit=bool(r.get('safe_exit',{}).get('safe')),
                parking=r.get('safe_exit',{}).get('park')))
            rejections.extend(r.get('continuations')or[])
        if op=='relocate_station':move=r
        elif op=='relocate_observe_prepare':move=r.get('movement',{})
        else:continue
        relocations.append(dict(sequence=c['sequence'],reached=move.get('reached'),executed=move.get('executed'),
            initial_base=move.get('initial_base'),actual_base=move.get('actual_base'),
            object_coordinate=move.get('object_coordinate'),position_error_m=move.get('position_error_m')))
    finite=a['joint_q'][np.isfinite(a['joint_q'])]
    maxq=float(finite.max()) if len(finite) else 0.
    reverse=sum(max(0.,s['start']-s['end'])for s in physical if s['start']is not None and s['end']is not None)
    forward=sum(max(0.,s['end']-s['start'])for s in physical if s['start']is not None and s['end']is not None)
    final_exit=trace[-1]['result'].get('safe_exit',{})if trace else{}
    if not final_exit and acquisitions:final_exit=dict(safe=acquisitions[-1]['safe_exit'],park=acquisitions[-1]['parking'])
    readiness=p/'model/FIT_READINESS_V81.json';fit=json.loads(readiness.read_text())if readiness.exists()else{}
    model=p/'model/estimate.json';estimate=json.loads(model.read_text())if model.exists()else{}
    selected=estimate.get('selected')or{}
    gate=dict(actual_range=maxq>=(np.pi/2 if joint=='revolute' else .15),
        meaningful_reverse=reverse>=(np.pi/8 if joint=='revolute'else .0375),
        qualified_bidirectional_FIT=bool(fit.get('qualified_bidirectional')),
        complete_independent_VALIDATION=any(s['target_reached']and s['role'].startswith('VALIDATION')for s in physical),
        final_safe_park=bool(final_exit.get('safe')and final_exit.get('park',{}).get('reached')and final_exit.get('park',{}).get('unloaded')))
    result=dict(episode_id=p.name,joint_type=joint,units=d['units'],display_units='deg'if joint=='revolute'else'mm',
        maximum_actual=maxq,maximum_display=maxq*scale,reverse_travel=reverse,reverse_display=reverse*scale,
        forward_travel=forward,forward_display=forward*scale,gate=gate,PASS=all(gate.values()),
        segments=physical,acquisitions=acquisitions,base_relocations=relocations,decisions=decisions,
        continuation_rejections=rejections,validations=validations,FIT_readiness=fit,
        final_online_parameters=selected.get('parameters'),final_online_conditional_std=selected.get('conditional_std'),
        final_online_model_sha256=digest(model)if model.exists()else None,
        final_park=final_exit.get('park'),
        safety_stop_samples=int(np.count_nonzero(a.get('safety_stop',[]))),
        load_warning_samples=int(np.count_nonzero(a.get('load_warning',[]))),
        profile=runtime.get('profile'),process=d['process'],
        total_storage_bytes=d['total_storage_bytes'],storage_bytes_by_category=d['storage_bytes_by_category'],
        online_frozen_manifest_sha256=digest(p/'ONLINE_FROZEN_MANIFEST.json'),
        source_commit=json.loads((p/'PROCESS.json').read_text())['source_commit'],
        physics_commitment=json.loads((p/'V81_PRELAUNCH.json').read_text())['spec']['physics_sha256'],
        truth_not_used_for_processing=True,authored_accuracy_requires_separate_posthoc_evaluator=True)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('episode',type=Path);parser.add_argument('--root',required=True,type=Path);parser.add_argument('--output',required=True,type=Path);parser.add_argument('--figure',type=Path)
    args=parser.parse_args();out=summarize(args.episode,args.root)
    args.output.write_text(json.dumps(out,indent=2,allow_nan=False))
    if args.figure:motion_figure(args.episode,args.root,args.figure)
    print(json.dumps({k:out[k]for k in ['episode_id','maximum_display','reverse_display','gate','PASS','total_storage_bytes']}))
