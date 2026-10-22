"""Read-only SEALED episode analysis. No fitting, simulation or truth-based tuning."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from diagnose_probing_v3 import columns,rows_at


def analyze(episode,output):
    p=Path(episode);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    assert json.loads((p/'PROCESS_COMPLETION.json').read_text())['actor_exited']
    seal=json.loads((p.parent.parent/'derived/ONLINE_SEAL_R3.json').read_text())
    assert seal['online_finished']and all(x['completion']['actor_exited']for x in seal['episodes'])
    sealed=next(x for x in seal['episodes']if x['episode']==p.name)
    assert hashlib.sha256((p/'model/estimate.json').read_bytes()).hexdigest()==sealed['sha256']['model/estimate.json']
    a,index=columns(p);settings=json.loads((p/'identification_input.json').read_text())
    axis=np.asarray(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    phases=np.asarray(index['metadata']['phases'])[a['phase']]
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text())
    attempts=[x for x in trace if x['command']['operation']=='attempt_grasp_and_probe']
    if len(attempts)!=len(index['segments']):raise ValueError('This R3 diagnostic requires one recorded probe segment per acquisition')
    authored=json.loads((p/'HIDDEN_AUTHORED_PHYSICS.json').read_text())
    model_path=p/'model/estimate.json';model_hash=hashlib.sha256(model_path.read_bytes()).hexdigest()
    model=json.loads(model_path.read_text())
    report=dict(episode=p.name,analysis_only=True,physical_attempts=[],
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        index_sha256=hashlib.sha256((p/'telemetry/index.json').read_bytes()).hexdigest(),
        frozen_online_model_sha256=model_hash,online_model_status=model['status'],
        eligible_FIT_bins=model.get('training_bins'),validation_bins=model.get('validation_bins'),
        parameter_estimates=None,parameter_errors=None,heldout_prediction_errors=None,
        posthoc_authored_reference=dict(moving_mass_kg=sum(x['mass_kg']for x in authored['bodies']),
            damping_Ns_m=authored['configuration']['b'],friction_N=authored['configuration']['tau_c'],
            law=authored['law'],not_independently_measured_effective_PhysX_dynamics=True),
        maximum_measured_coordinate_mm=float(np.max(a['joint_q'])*1000),
        complete_independent_VALIDATION=False,not_a_matched_preload_AB=True)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,len(attempts),figsize=(11,9),squeeze=False)
    for j,entry in enumerate(attempts):
        result=entry['result'];s=index['segments'][j]
        ix=np.flatnonzero((a['step']>=s['first_step'])&(a['step']<=s['last_step']))
        rows=rows_at(a,ix,index['metadata'],'POST_SEAL_DIAGNOSTIC')
        R=np.array([r['wrist_ft']['T_world_sensor'][:3,:3]for r in rows]);raw=a['raw_ft'][ix]-settings['bias']
        w=np.c_[np.einsum('nij,nj->ni',R,raw[:,:3]),np.einsum('nij,nj->ni',R,raw[:,3:])]
        w+=np.array([r['wrist_ft']['tool_gravity_wrench_world']for r in rows])
        rail=w[:,:3]@axis;trans=np.linalg.norm(w[:,:3]-rail[:,None]*axis,axis=1)
        t=a['t'][ix]-a['t'][ix[0]];q=a['joint_q'][ix];actual=a['tcp_pose'][ix,:3];command=a['command_TCP_pose'][ix,:3]
        track=((command-command[0])-(actual-actual[0]))@axis
        relative=(actual-actual[0])@axis-(q-q[0])
        active=np.isin(phases[ix],['CONTACT_PRELOAD','OPEN_CONTINUE','TRANSMISSION_PROBE'])
        # The legacy reference field persists during closure; never portray
        # that inactive old value as a current motion command.
        reference=np.where(active,a['reference_q'][ix],np.nan)
        axes[0,j].plot(t,(q-q[0])*1000,label='actual drawer')
        axes[0,j].plot(t,(reference-q[0])*1000,label='active reference only')
        axes[0,j].set_title('Attempt %d: %.1f N request'%(j+1,entry['command']['native_g1_preload_request_n']))
        axes[1,j].plot(t,track*1000,label='command - measured TCP')
        axes[1,j].plot(t,relative*1000,label='measured TCP - drawer')
        axes[2,j].plot(t,rail,label='rail force');axes[2,j].plot(t,trans,label='transverse force')
        moment=np.linalg.norm(w[:,3:],axis=1);axes[3,j].plot(t,moment,label='sensor-origin moment')
        axes[3,j].axhline(.6,color='red',ls='--',label='original sustained band')
        r=dict(attempt=j+1,request_N=entry['command']['native_g1_preload_request_n'],family=result['family'],
            acquired=result.get('acquisition',{}).get('acquired'),status=s['result']['status'],reason=s['result']['reason'],
            measured_segment_travel_mm=float((q[-1]-q[0])*1000),
            tracking_deficit_change_mm=float(track[-1]*1000),TCP_drawer_relative_change_mm=float(relative[-1]*1000),
            raw_reported_normal_peak_N=a['pad_load'][(a['step']>=result['first_record'])&(a['step']<result['last_record'])].max(0).tolist(),
            safe_release_and_park=result.get('safe_exit',{}).get('safe'),first_positive_pad_reports=[])
        pre=json.loads((p/'attempt_inputs'/('%02d.json'%j)).read_text())['selected']
        closing=json.loads((p/('CLOSING_REVALIDATION_%02d.json'%j)).read_text())
        r['pregrasp_refinement_translation_mm']=float(np.linalg.norm(np.array(pre['T'])[:3,3]-np.array(closing['refined_target_TCP'])[:3,3])*1000)
        close=np.flatnonzero((a['step']>=result['first_record'])&(a['step']<result['last_record'])&np.isin(phases,['SLOW_CLOSE','CLOSE','CENTER','FORCE_HOLD','HOLD','GRASP_VERIFY']))
        for side in range(2):
            hits=close[a['pad_load'][close,side]>0]
            r['first_positive_pad_reports'].append(None if not len(hits)else dict(aperture_mm=float(a['aperture'][hits[0]]*1000),load_N=float(a['pad_load'][hits[0],side]),definition='first strictly positive aggregate normal report; not a calibrated geometric contact threshold'))
        margin=s['result'].get('load_margin')
        if margin:
            stop=margin['t'];ids=np.flatnonzero((a['t'][ix]>stop-.15)&(a['t'][ix]<=stop))
            tcp_m=w[ids,3:]+np.cross(a['sensor_pose'][ix[ids],:3]-a['tcp_pose'][ix[ids],:3],w[ids,:3])
            r.update(original_forecast_event=margin,late_guard_window=dict(samples=len(ids),
                signed_rail_force_mean_N=float(rail[ids].mean()),transverse_force_norm_mean_N=float(trans[ids].mean()),
                sensor_origin_moment_norm_mean_Nm=float(moment[ids].mean()),TCP_origin_moment_norm_mean_Nm=float(np.linalg.norm(tcp_m,axis=1).mean()),
                compensation='bias + tool gravity; exactly the causal protection convention, not centered inertia compensation'))
            for ax in axes[:,j]:ax.axvline(stop-a['t'][ix[0]],color='black',lw=.7,ls=':')
        report['physical_attempts'].append(r)
    for j in range(len(attempts)):
        for k,label in enumerate(['Motion (mm)','Two separate errors (mm)','Force (N)','Moment (N m)']):
            axes[k,j].set_ylabel(label);axes[k,j].grid(alpha=.2);axes[k,j].legend(fontsize=6)
        axes[-1,j].set_xlabel('Seconds from physical segment start')
    fig.suptitle('R3 preserved first failure and final recovery partial progress; no independent VALIDATION')
    fig.tight_layout();fig.savefig(output/'R3_CONTACT_AND_LOAD.png',dpi=135);plt.close(fig)
    assert hashlib.sha256(model_path.read_bytes()).hexdigest()==model_hash
    (output/'R3_OUTCOME_DIAGNOSTIC.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=analyze(a.episode,a.output)
    print(json.dumps(r,indent=2))
