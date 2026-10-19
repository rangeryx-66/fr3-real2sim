"""Post-exit evaluation only; never imported by actor or online worker."""
import argparse,copy,datetime,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def evaluate(root):
    root=Path(root);p=root/('episode_01' if (root/'episode_01').exists() else 'episode_00')
    done=json.loads((p/'PROCESS_COMPLETION.json').read_text());pid=json.loads((p/'PROCESS.json').read_text())['pid']
    if not done['actor_exited'] or Path('/proc/%d'%pid).exists():raise RuntimeError('Truth stays hidden until actor exit and all online decisions end')
    from diagnose_probing_v3 import columns
    from update_probing_v31_model import load_segments,XY,motion_prediction
    from evaluate_xarm_ft_identification import effective_inertia
    from summarize_efficient_probing_v2 import summarize
    from interaction_runtime.segment_quality import selection
    from interaction_runtime.compact_v2 import storage
    from interaction_runtime.serialization import result_packet
    h=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    online_path=p/'model/estimate.json';original_hash=h(online_path) if online_path.exists() else None
    online=json.loads(online_path.read_text()) if online_path.exists() else dict(status='no online model')
    a,index=columns(p);post=root/'posthoc';post.mkdir(exist_ok=True)
    pieces,train,test,quality=load_segments(p,allow_interrupted_fit=True) if (p/'identification_input.json').exists() else ([],[],[],[])
    accessed=datetime.datetime.now(datetime.timezone.utc).isoformat()
    truth=json.loads((p/'HIDDEN_AUTHORED_PHYSICS.json').read_text());job=json.loads((p/'job.json').read_text())
    prior=json.loads((p/'identification_input.json').read_text())['prior'] if (p/'identification_input.json').exists() else {}
    known=np.array([effective_inertia(truth,job['asset_root'],prior),truth['configuration']['b'],truth['configuration']['tau_c']]) if prior else None
    legacy={};legacy_time=None
    if train:
        import update_probing_v3_model as old
        from diagnose_probing_v3 import rows_at
        settings=json.loads((p/'identification_input.json').read_text());legacy_train=[];legacy_test=[]
        started=time.perf_counter()
        # Selection adapter only: frozen V3 numerical functions on exactly the
        # same qualified raw blocks. Failed actions retain their status. This
        # is not the legacy production worker's complete-action admission rule.
        for record in quality:
            for block in record.get('blocks',[]):
                if not block.get('bins'):continue
                ix=np.flatnonzero((a['segment']==record['id'])&(a['step']>=block['first_step'])&(a['step']<=block['last_step']))
                label=record['role']+'_%02d'%record['id']
                d=old.process(rows_at(a,ix,index['metadata'],label),settings['prior'],settings['bias']);d['valid'][:121]=False;d['valid'][-121:]=False
                samples=old.regression_samples(d)
                (legacy_train if label.startswith('FIT') else legacy_test).extend(samples)
        if len(legacy_train)>=8:
            cm=old.conditional_model(legacy_train)
            if legacy_test:
                X,y=old.XY(legacy_test);cm['heldout_torque_rmse_nm']=float(np.sqrt(np.mean((y-X@cm['parameters'])**2)))
            legacy=dict(status='offline frozen V3 numerics / same qualified raw intervals',constant_model=cm,training_bins=len(legacy_train),validation_bins=len(legacy_test),selection_only_adapter=True,failed_actions_not_relabelled=True,truth_used_for_fitting=False)
        legacy_time=time.perf_counter()-started
        target=post/'legacy_same_intervals';target.mkdir(exist_ok=True);(target/'estimate.json').write_text(json.dumps(result_packet(legacy),indent=2))
    comparison={}
    for name,m in [('V31_online',online),('V3_offline_same_intervals',legacy)]:
        c=m.get('constant_model',{});beta=np.array(c.get('parameters',[np.nan]*3))
        row=dict(status=m.get('status'),parameters=c.get('parameters'),conditional_std=c.get('conditional_std'),supported=c.get('supported'),training_bins=m.get('training_bins'),validation_bins=m.get('validation_bins'),heldout_torque_rmse_nm=c.get('heldout_torque_rmse_nm'),heldout_motion=c.get('heldout_motion'),full_segment_boundary_sensitivity=c.get('full_segment_boundary_sensitivity'),information=c.get('information',m.get('information')))
        if known is not None and np.isfinite(beta).all():row.update(absolute_error=(beta-known).tolist(),relative_error_percent=(100*(beta/known-1)).tolist())
        if train and test and c.get('parameters'):
            X,y=XY(test);row['common_V31_heldout_torque_rmse_nm']=float(np.sqrt(np.mean((y-X@beta)**2)))
            shared=dict(c,training_angle_range_deg=np.rad2deg([min(s['q'] for s in train),max(s['q'] for s in train)]).tolist());pred={}
            for d in pieces:
                if not str(d['segments'][0]).startswith('VALIDATION'):continue
                clone=dict(d,qd=d['native_qd'].copy());edge=d['processing']['boundary_exclusion_samples'];pred[d['block_id']]=motion_prediction(clone,np.arange(edge,len(d['t'])-edge),shared)
            row['common_V31_heldout_motion']=pred
        comparison[name]=row
    excitation=[]
    for d in pieces:
        g=d['valid'];X=np.c_[d['qdd'][g],d['qd'][g],d['direction'][g]]
        excitation.append(dict(segment=d['block_id'],duration_s=float(d['t'][-1]-d['t'][0]),angle_range_deg=np.rad2deg([d['q'].min(),d['q'].max()]).tolist(),qdd_rms=float(np.sqrt(np.mean(X[:,0]**2))),qd_rms=float(np.sqrt(np.mean(X[:,1]**2))),velocity_quantiles=np.quantile(d['native_qd'][g],[.05,.5,.95]).tolist(),regressor_singular_values=np.linalg.svd(X,compute_uv=False).tolist(),processing=d['processing']))
    actual=summarize(p)
    intact=all(h(p/'telemetry'/c['file'])==c['sha256'] for c in index['chunks'])
    result=dict(reference='aec1458',registration='f7dfebc',label='AGENT_WRIST_RGBD / KNOWN_HINGE_STATE / IDEAL_LOAD_CELL / APPROXIMATE_ACTUATOR_SIMULATION',authored_reference=known.tolist() if known is not None else None,authored_configuration=truth['configuration'],authored_not_independently_verified_effective_PhysX_inertia=True,comparison=comparison,quality=quality,excitation=excitation,actual=actual,online_model=online,completion=done,episode_result=json.loads((p/'EPISODE_RESULT.json').read_text()),runtime=json.loads((p/'RUNTIME_RESULT.json').read_text()),storage=storage(p),online_model_sha256=original_hash,online_model_preserved=(h(online_path)==original_hash if original_hash else None),all_numeric_chunks_intact=intact,source_provenance=json.loads((p/'SOURCE_PROVENANCE.json').read_text()),legacy_wall_s=legacy_time,truth_revealed_after_exit_UTC=accessed,private_evidence_root=str(p),raw_published=False)
    access=dict(actor_exited=True,accessed_UTC=accessed,configuration_commitment_matches=h(root/'HIDDEN_CONFIGURATIONS.json')==json.loads((root/'CONFIGURATION_COMMITMENT.json').read_text())['configuration_sha256'],online_model_preserved=result['online_model_preserved'])
    (root/'TRUTH_ACCESS.json').write_text(json.dumps(access,indent=2));(root/'RESULT.json').write_text(json.dumps(result_packet(result),indent=2,allow_nan=False))
    print(json.dumps(result_packet(dict(actual=actual,comparison=comparison,storage=result['storage'],completion=done)),indent=2))
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);evaluate(parser.parse_args().root)
