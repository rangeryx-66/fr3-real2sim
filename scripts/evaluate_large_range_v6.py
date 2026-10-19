"""Post-exit only independent V6 evaluation. No fitting or model selection."""
import argparse,datetime,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def evaluate(root):
    root=Path(root);p=root/'episode_00';done=json.loads((p/'PROCESS_COMPLETION.json').read_text());pid=json.loads((p/'PROCESS.json').read_text())['pid']
    if not done['actor_exited'] or Path('/proc/%d'%pid).exists():raise RuntimeError('truth remains hidden until actor and Agent finish')
    ep=json.loads((p/'EPISODE_RESULT.json').read_text());online_path=p/'model/estimate.json';online=json.loads(online_path.read_text()) if online_path.exists() else {};before=hashlib.sha256(online_path.read_bytes()).hexdigest() if online else None
    previous_access=root/'TRUTH_ACCESS.json';accessed=json.loads(previous_access.read_text())['accessed_UTC'] if previous_access.exists() else datetime.datetime.now(datetime.timezone.utc).isoformat();truth=json.loads((p/'HIDDEN_AUTHORED_PHYSICS.json').read_text());private=(root/'HIDDEN_CONFIGURATIONS.json').read_bytes();commitment=json.loads((root/'CONFIGURATION_COMMITMENT.json').read_text());assert hashlib.sha256(private).hexdigest()==commitment['configuration_sha256']
    from evaluate_xarm_ft_identification import effective_inertia
    from update_probing_v31_model import load_segments
    from xarm_ft_v6_estimator import samples_from_piece,basis,prediction,matrix
    from summarize_efficient_probing_v2 import summarize
    from interaction_runtime.compact_v2 import storage
    from diagnose_probing_v3 import columns
    prior=json.loads((p/'identification_input.json').read_text())['prior'] if (p/'identification_input.json').exists() else None
    knownI=effective_inertia(truth,json.loads((p/'job.json').read_text())['asset_root'],prior) if prior else None
    pieces,_,_,quality=load_segments(p,allow_interrupted_fit=True) if prior else ([],[],[],[]);train=[];test=[]
    for d in pieces:(test if str(d['segments'][0]).startswith('VALIDATION') else train).extend(samples_from_piece(d))
    frozen=json.loads((p/'PREVALIDATION_ONLINE_MODEL.json').read_text()) if (p/'PREVALIDATION_ONLINE_MODEL.json').exists() else None
    result=dict(experiment='V6',truth_access_UTC=accessed,episode_result=ep,completion=done,online_model=online,quality=quality,authored_configuration=truth['configuration'],authored_inertia=knownI,authored_not_independent_PhysX_measurement=True,storage=storage(p),actual=summarize(p),comparison={},raw_published=False)
    from xarm_ft_v6_estimator import support_intervals
    merged=support_intervals(train)
    result['qualified_FIT_support_intervals_deg']=merged
    law=truth['configuration']['law'];curve=lambda q:law['base_nm']+law['amplitude_nm']*np.exp(-.5*((q-law['center_rad'])/law['width_rad'])**2)
    if frozen:
        assert frozen['validation_bins']==0
        for key in ['selected_model','selected','candidates']:
            def strip(x):
                if isinstance(x,dict):return {k:strip(v) for k,v in x.items() if k!='heldout_motion'}
                if isinstance(x,list):return [strip(v) for v in x]
                return x
            result.setdefault('prevalidation_immutability',{})[key]=strip(frozen.get(key))==strip(online.get(key))
        for name in ['constant_model','extended_model']:
            m=frozen.get(name)
            if not m:continue
            model_intervals=[[max(a,m['training_angle_range_deg'][0]),min(b,m['training_angle_range_deg'][1])] for a,b in merged if max(a,m['training_angle_range_deg'][0])<min(b,m['training_angle_range_deg'][1])];q=np.concatenate([np.linspace(*np.deg2rad(interval),121) for interval in model_intervals]);C=basis(q,m['resolution_deg'])[:,m['active_controls']]@m['parameters'][2:] if m['resolution_deg'] else np.full(len(q),m['parameters'][2]);gt=curve(q)
            result['comparison'][name]=dict(resolution_deg=m['resolution_deg'],active_controls=m['active_controls'],parameters=m['parameters'],conditional_std=m['conditional_std'],cv_rmse_nm=m['cv_rmse_nm'],support_deg=m['training_angle_range_deg'],resistance_curve_rmse_nm=float(np.sqrt(np.mean((C-gt)**2))),resistance_estimated_peak_to_peak_nm=float(np.ptp(C)),authored_resistance_peak_to_peak_nm=float(np.ptp(gt)),inertia_relative_error_pct=100*(m['parameters'][0]/knownI-1),damping_relative_error_pct=100*(m['parameters'][1]/truth['configuration']['b']-1),heldout={d['block_id']:prediction(d,m) for d in pieces if str(d['segments'][0]).startswith('VALIDATION')})
    a,idx=columns(p);intact=all(hashlib.sha256((p/'telemetry'/c['file']).read_bytes()).hexdigest()==c['sha256'] for c in idx['chunks']);result['numeric_chunks_intact']=intact
    result['online_model_preserved']=before==hashlib.sha256(online_path.read_bytes()).hexdigest() if online else None
    result['excitation']=[dict(segment=d['block_id'],role=str(d['segments'][0]),range_deg=np.rad2deg([d['q'].min(),d['q'].max()]).tolist(),velocity_rms_rad_s=float(np.sqrt(np.mean(d['native_qd'][d['valid']]**2))),acceleration_rms_rad_s2=float(np.sqrt(np.mean(d['qdd'][d['valid']]**2))),records=len(d['q']),valid=int(d['valid'].sum())) for d in pieces]
    reads=[json.loads(v) for v in (p/'PRIVATE_NATIVE_RESISTANCE.jsonl').read_text().splitlines()];result['native_law_application']=dict(method='native tensor DOF static/dynamic friction update and backend readback; viscous preserved',readbacks=len(reads),C_range_nm=[min(v['law_nm'] for v in reads),max(v['law_nm'] for v in reads)],property_max_abs_readback_error=float(max(abs(v['native_properties'][0]-v['law_nm']) for v in reads)),no_object_joint_target_or_effort=True)
    out=root/'public_derived';out.mkdir(exist_ok=True);(out/'REVEALED_CONFIGURATIONS.json').write_bytes(private)
    dataset=root/'passive_geometry_dataset';camera_index=json.loads((dataset/'index.json').read_text());privileged=json.loads((root/'fixed_camera_evaluation/privileged_states.json').read_text());manifest={str(f.relative_to(dataset)):hashlib.sha256(f.read_bytes()).hexdigest() for f in dataset.rglob('*') if f.is_file() and f.name!='integrity.json'};atomic_json(dataset/'integrity.json',dict(files=manifest,no_privileged_state=True));result['passive_camera_dataset']=dict(pairs=camera_index['total_pairs'],valid_synchronized_pairs=camera_index['valid_synchronized_pairs'],capture_wall_s=camera_index['capture_wall_s'],observed_states_evaluation_only=[dict(sequence=s['sequence'],angle_deg=float(np.rad2deg(s['joint_angle_rad']))) for s in privileged['states']],pose_max_abs_error=max(c['pose_max_abs_error'] for pair in camera_index['pairs'] for c in pair['cameras']),minimum_valid_depth_fraction=min(c['valid_depth_fraction'] for pair in camera_index['pairs'] for c in pair['cameras']),online_consumers=camera_index['online_consumers'],geometry_package_contains_GT_state=False,files=len(manifest),bytes=sum(f.stat().st_size for f in dataset.rglob('*') if f.is_file()))
    result['postvalidation_FIT_declared']=any(str(s['role']).startswith('FIT') and s['id']>min([v['id'] for v in quality if str(v['role']).startswith('VALIDATION')],default=10**9) for s in quality)
    result['prevalidation_comparison_note']='Primary heldout comparison uses immutable pre-VALIDATION snapshot. A predeclared later FIT may legitimately update the final online model; VALIDATION never fits/selects.'
    if online.get('selected'):
        m=online['selected'];q=np.concatenate([np.linspace(*np.deg2rad(interval),121) for interval in merged]);C=basis(q,m['resolution_deg'])[:,m['active_controls']]@m['parameters'][2:] if m['resolution_deg'] else np.full(len(q),m['parameters'][2]);result['final_online_comparison']=dict(parameters=m['parameters'],conditional_std=m['conditional_std'],resolution_deg=m['resolution_deg'],scaled_condition=m['scaled_condition'],inertia_relative_error_pct=100*(m['parameters'][0]/knownI-1),damping_relative_error_pct=100*(m['parameters'][1]/truth['configuration']['b']-1),observed_support_curve_rmse_nm=float(np.sqrt(np.mean((C-curve(q))**2))),heldout=m.get('heldout_motion',{}))
    result['combined_storage_bytes']=sum(f.stat().st_size for f in root.rglob('*') if f.is_file())
    result['assessment']='REQUIRES_REVIEW: actual 30deg bidirectional motion, complete independent validation and isolated passive dataset required'
    atomic_json(root/'RESULT.json',result_packet(result));atomic_json(root/'TRUTH_ACCESS.json',dict(actor_exited=True,accessed_UTC=accessed,configuration_commitment_matches=True,online_model_preserved=result['online_model_preserved']))
    if frozen:
        model=online.get('selected',{});export=dict(estimator_version=online.get('estimator_version'),basis=model.get('basis'),knots_rad=model.get('knots_rad'),active_controls=model.get('active_controls'),parameters=model.get('parameters'),conditional_covariance=model.get('conditional_covariance'),qualified_FIT_support_intervals_deg=merged,unobserved_regions='unknown; smooth interpolation is a model assumption, not measured support',sensor='ideal simulated wrist load cell; known hinge/state; approximate actuators',prior_geometry='registered cabinet identity and known joint prior; geometry reconstruction not performed',source_online_sha256=before,validation_used_for_fitting=False)
        atomic_json(out/'estimated_physics_model.json',result_packet(export))
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,2,figsize=(11,7))
        for segment in idx['segments']:
            selected=a['segment']==segment['id'];axes[0,0].plot(a['t'][selected],np.rad2deg(a['door_q'][selected]),label=segment['role'])
        for d in pieces:axes[1,0].scatter(np.rad2deg(d['q'][d['valid']]),d['torque'][d['valid']]*1000,s=1,label=str(d['segments'][0]))
        axes[0,0].set(ylabel='Full physical segment angle (deg)',xlabel='Simulation time (s)');axes[0,0].legend(fontsize=5);axes[1,0].set(xlabel='Angle (deg)',ylabel='Wrist-F/T hinge torque (mNm)')
        for name in ['constant_model','extended_model']:
            m=frozen.get(name)
            if not m:continue
            c=m['resistance_curve'];x=np.array(c['angle_deg']);y=np.array([np.nan if v is None else v for v in c['resistance_nm']]);sd=np.array([np.nan if v is None else v for v in c['conditional_std_nm']]);supported=np.array([any(lo<=v<=hi for lo,hi in merged) for v in x]);y[~supported]=np.nan;sd[~supported]=np.nan;axes[0,1].plot(x,y*1000,label=name);axes[0,1].fill_between(x,(y-2*sd)*1000,(y+2*sd)*1000,alpha=.15)
            for d in pieces:
                if str(d['segments'][0]).startswith('VALIDATION'):
                    g=d['valid'];resolution=m['resolution_deg'];X=np.c_[d['qdd'],d['qd'],d['angle_features'][resolution][:,m['active_controls']]] if resolution else np.c_[d['qdd'],d['qd'],d['direction']];axes[1,1].plot(d['t'][g],(d['torque'][g]-X[g]@m['parameters'])*1000,label=name)
        lo,hi=frozen['constant_model']['training_angle_range_deg'];q=np.linspace(lo,hi,301);gtcurve=curve(np.deg2rad(q))*1000;gtcurve[~np.array([any(a<=v<=b for a,b in merged) for v in q])]=np.nan;axes[0,1].plot(q,gtcurve,'k--',label='Authored law: revealed after exit');axes[0,1].set(xlabel='Observed-support angle (deg)',ylabel='C(q), mNm');axes[0,1].legend(fontsize=6);axes[1,1].set(xlabel='Validation time (s)',ylabel='Held-out torque residual (mNm)');axes[1,1].legend(fontsize=6);fig.tight_layout();fig.savefig(out/'angle_resistance_validation.png',dpi=130);plt.close(fig)
    atomic_json(out/'METRICS.json',result_packet({k:result[k] for k in ['experiment','truth_access_UTC','completion','comparison','excitation','native_law_application','prevalidation_immutability','numeric_chunks_intact','online_model_preserved','storage','actual','assessment','passive_camera_dataset','combined_storage_bytes','qualified_FIT_support_intervals_deg','final_online_comparison','postvalidation_FIT_declared','prevalidation_comparison_note'] if k in result}))
    print(json.dumps(result_packet(dict(comparison=result['comparison'],actual=result['actual'],completion=done,storage=result['storage'])),indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);evaluate(p.parse_args().root)
