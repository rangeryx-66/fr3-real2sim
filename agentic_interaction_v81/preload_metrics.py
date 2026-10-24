"""Read-only matched preload measurements; no online action or hidden truth."""
import json
from pathlib import Path
import numpy as np


def summarize(folder):
    from diagnose_probing_v3 import columns
    from .feedback import motion_errors
    from agent_command_protocol_v12 import atomic_json
    folder=Path(folder);a,index=columns(folder)
    settings=folder/'identification_input.json'
    prior=(json.loads(settings.read_text())['prior'] if settings.exists() else
        dict(joint_type='prismatic',axis_world=json.loads((folder/'contact_evidence/index.json').read_text())['axis_world']))
    axis=np.asarray(prior['axis_world'],float);axis/=np.linalg.norm(axis)
    api_valid=a['friction_api_valid'].all(axis=1)
    valid=api_valid&(a['friction_anchor_counts'].sum(axis=1)>0)
    point_index=folder/'contact_evidence/index.json';points=None
    if point_index.exists():
        point_manifest=json.loads(point_index.read_text());pieces=[]
        for entry in point_manifest['files']:
            with np.load(point_index.parent/entry['file'],allow_pickle=False)as data:
                pieces.append({k:data[k].copy()for k in ('step','finger','kind','point_pad','impulse_world','longitudinal_margin_m','lateral_margin_m')})
        if pieces:points={k:np.concatenate([x[k]for x in pieces])for k in pieces[0]}
    # Native API records forces ON fingers. The drawer receives the opposite
    # reaction. Normal-only readings are not a complete contact wrench.
    on_fingers=(a['normal_force_on_fingers_world_N']+a['friction_force_on_fingers_world_N'])
    object_force=-on_fingers.sum(axis=1)
    axial=object_force@axis;transverse=np.linalg.norm(object_force-axial[:,None]*axis,axis=1)
    items=[]
    for segment in index['segments']:
        ix=np.flatnonzero((a['step']>=segment['first_step'])&(a['step']<=segment['last_step']))
        if not len(ix):continue
        errors=motion_errors(a['command_TCP_pose'][ix,:3],a['tcp_pose'][ix,:3],a['joint_q'][ix],prior)
        direction=np.sign(segment['target_q']-a['joint_q'][ix[0]])
        traction=direction*axial[ix]
        useful=valid[ix]
        outcome=segment.get('result',{})
        locations=[]
        if points is not None:
            for finger in (0,1):
                use=((points['step']>=segment['first_step'])&(points['step']<=segment['last_step'])
                     &(points['kind']==0)&(points['finger']==finger))
                weights=np.linalg.norm(points['impulse_world'][use],axis=1)
                if weights.sum()>0:
                    locations.append(dict(finger=finger,native_normal_report_points=int(use.sum()),
                        impulse_weighted_pad_centroid_m=np.average(points['point_pad'][use],axis=0,weights=weights).tolist(),
                        longitudinal_edge_margin_p05_m=float(np.quantile(points['longitudinal_margin_m'][use],.05)),
                        lateral_edge_margin_p05_m=float(np.quantile(points['lateral_margin_m'][use],.05)),
                        meaning='reported native normal-contact locations, not pressure distribution or proof of full contact area'))
        items.append(dict(id=segment['id'],role=segment['role'],native_contact_locations=locations,
            result={key:outcome.get(key) for key in ('status','reason','requested_target_reached','physical_transmission_valid')},
            first_q_m=float(a['joint_q'][ix[0]]),last_q_m=float(a['joint_q'][ix[-1]]),
            actual_range_m=float(np.ptp(a['joint_q'][ix])),duration_s=float(np.ptp(a['t'][ix])),
            measured_pad_mean_n=a['pad_load'][ix].mean(0).tolist(),
            measured_pad_peak_n=a['pad_load'][ix].max(0).tolist(),
            measured_aperture_start_end_m=a['aperture'][ix[[0,-1]]].tolist(),
            commanded_aperture_start_end_m=a['command_aperture'][ix[[0,-1]]].tolist(),
            aperture_drift_m=float(a['aperture'][ix[-1]]-a['aperture'][ix[0]]),
            native_friction_api_valid_fraction=float(api_valid[ix].mean()),
            native_traction_observable_fraction=float(useful.mean()),
            reaction_traction_mean_n=float(np.mean(traction[useful])) if useful.any() else None,
            reaction_traction_peak_n=float(np.max(traction[useful])) if useful.any() else None,
            reaction_transverse_mean_n=float(np.mean(transverse[ix][useful])) if useful.any() else None,
            max_incremental_command_tracking_m=float(np.linalg.norm(errors['incremental_servo_error_world_m'],axis=1).max()),
            max_measured_TCP_drawer_residual_m=float(np.linalg.norm(errors['measured_minus_rigid_articulation_world_m'],axis=1).max())))
    active=a['segment']>0
    result=dict(episode=folder.name,segments=items,physics_dt_s=index['metadata'].get('physics_dt_s',1/240),
        maximum_drawer_q_m=float(a['joint_q'].max()),
        requested_preload_n=sorted(set(float(v)for v in a['preload_request_n'])),
        measured_motor_force='unavailable; master target/cap are commands, not measured motor effort',
        native_contact_forces='diagnostic cross-check only; opposite of force on fingers, not estimator input',
        incomplete_validation_is_not_PASS=True)
    phases=index['metadata']['phases']
    holding=(a['phase']==phases.index('FORCE_HOLD')) if 'FORCE_HOLD' in phases else np.zeros(len(active),bool)
    holding&=~active
    if holding.any():
        ix=np.flatnonzero(holding);ix=ix[a['t'][ix]>=a['t'][ix[-1]]-1.]
        result['acquisition_hold']=dict(measured_pad_mean_n=a['pad_load'][ix].mean(0).tolist(),
            measured_pad_peak_n=a['pad_load'][ix].max(0).tolist(),
            aperture_start_end_m=a['aperture'][ix[[0,-1]]].tolist())
        outcome=folder/'MATCHED_CONTACT_RESULT.json'
        acquired=json.loads(outcome.read_text()).get('acquisition',{}) if outcome.exists() else {}
        result['acquisition_hold']['completed_original_one_second_hold']=bool(acquired.get('acquired') and acquired.get('hold_s')==1.)
    for key in ('closure_internal_effort_request_n','native_master_target_rad','native_master_measured_rad',
                'mapping_unclipped_master_rad','native_master_cap_nm'):
        if key in a:
            values=a[key][np.isfinite(a[key])]
            result[key+'_range']=[float(values.min()),float(values.max())] if len(values) else None
    if 'native_position_mapping_limit' in a:
        result['native_position_mapping_limit_samples']=int(a['native_position_mapping_limit'].sum())
    out=folder/'derived_preload';out.mkdir(exist_ok=True)
    atomic_json(out/'metrics.json',result)
    if active.any():
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        ix=np.flatnonzero(active);t=a['t'][ix]-a['t'][ix[0]]
        fig,ax=plt.subplots(5,1,figsize=(10,11),sharex=True)
        ax[0].plot(t,a['pad_load'][ix,0],lw=.6,label='left native normal');ax[0].plot(t,a['pad_load'][ix,1],lw=.6,label='right native normal');ax[0].set_ylabel('N')
        ax[1].plot(t,a['aperture'][ix]*1000,lw=.6,label='measured');ax[1].plot(t,a['command_aperture'][ix]*1000,lw=.6,label='command');ax[1].set_ylabel('Aperture mm')
        ax[2].plot(t,a['joint_q'][ix]*1000,lw=.6,label='drawer');ax[2].plot(t,a['reference_q'][ix]*1000,lw=.6,label='reference');ax[2].set_ylabel('Travel mm')
        ax[3].plot(t,np.where(valid[ix],axial[ix],np.nan),lw=.6,label='signed rail reaction');ax[3].plot(t,np.where(valid[ix],transverse[ix],np.nan),lw=.6,label='transverse reaction');ax[3].set_ylabel('Native force N')
        ax[4].plot(t,a['relative_slip_m'][ix]*1000,lw=.6,label='TCP/object relative displacement');ax[4].set_ylabel('Relative mm');ax[4].set_xlabel('Seconds from first interaction sample')
        for plot in ax:plot.grid(alpha=.2);plot.legend(fontsize=7)
        fig.suptitle(folder.name+' | raw 240 Hz samples, no peak averaging');fig.tight_layout();fig.savefig(out/'contact.png',dpi=120);plt.close(fig)
    return result


if __name__=='__main__':
    import sys
    print(json.dumps(summarize(sys.argv[1]),indent=2))
