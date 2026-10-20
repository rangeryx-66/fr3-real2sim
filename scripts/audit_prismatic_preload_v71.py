"""Compact measured preload/traction comparison; no scene truth or physics launch."""
import argparse,json
from pathlib import Path
import numpy as np
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v7_estimator import process


def summary(folder,plot=True):
    p=Path(folder);a,index=columns(p);settings=json.loads((p/'identification_input.json').read_text())
    prior=settings['prior'];axis=np.asarray(prior['axis_world']);axis/=np.linalg.norm(axis)
    phases=np.asarray(index['metadata']['phases'])[a['phase']]
    normal=-a['normal_force_on_fingers_world_N']@axis
    friction=-a['friction_force_on_fingers_world_N']@axis
    available=np.all(a['friction_api_valid']>0,axis=1)&np.any(a['friction_anchor_counts']>0,axis=1)
    friction=friction.copy();friction[~available]=np.nan
    cfg=json.loads((p/'PRELOAD_CONFIGURATION.json').read_text())
    out=dict(preload_request_n=cfg['active_policy']['preload_n'],force_request_is_not_measurement=True,
             actual_motor_effort='unavailable',normal_and_friction_separate=True,segments=[])
    out['recorded_preload_requests_n']=np.unique(a['preload_request_n']).tolist()
    out['normal_raw_peak_N']=a['pad_load'].max(0).tolist()
    out['measured_aperture_range_m']=[float(a['aperture'].min()),float(a['aperture'].max())]
    hold=np.flatnonzero(phases=='FORCE_HOLD')
    if len(hold):
        out['hold']=dict(duration_s=float(a['t'][hold[-1]]-a['t'][hold[0]]),
            normal_mean_N=a['pad_load'][hold].mean(0).tolist(),
            normal_P10_N=np.quantile(a['pad_load'][hold],.1,axis=0).tolist(),
            aperture_start_end_m=a['aperture'][hold[[0,-1]]].tolist())
    rows=[];wrist_effort=np.full(len(a['t']),np.nan)
    for s in index['segments']:
        ix=np.flatnonzero(a['segment']==s['id'])
        if not len(ix):continue
        x=a['tcp_pose'][ix,:3]@axis;u=a['command_TCP_pose'][ix,:3]@axis;q=a['joint_q'][ix]
        start=int(ix[0]);t=a['t'][ix]-a['t'][start]
        r=dict(id=s['id'],role=s['role'],status=s.get('result',{}).get('status'),
            recorded_preload_requests_n=np.unique(a['preload_request_n'][ix]).tolist(),
            reason=s.get('result',{}).get('reason'),duration_s=float(t[-1]),
            actual_drawer_gain_m=float(q[-1]-q[0]),actual_TCP_gain_m=float(x[-1]-x[0]),
            command_TCP_gain_m=float(u[-1]-u[0]),incremental_servo_deficit_m=float((u[-1]-u[0])-(x[-1]-x[0])),
            actual_TCP_drawer_relative_m=float((x[-1]-x[0])-(q[-1]-q[0])),
            pad_normal_mean_N=a['pad_load'][ix].mean(0).tolist(),pad_normal_peak_N=a['pad_load'][ix].max(0).tolist(),
            native_normal_rail_mean_N=normal[ix].mean(0).tolist(),
            native_tangential_rail_mean_N=np.nanmean(friction[ix],axis=0).tolist(),
            native_total_rail_quantiles_N=np.nanquantile((friction[ix]+normal[ix]).sum(1),[.1,.5,.9]).tolist(),
            native_friction_available_samples=int(available[ix].sum()),
            final_aperture_command_measured_m=[float(a['command_aperture'][ix[-1]]),float(a['aperture'][ix[-1]])],
            master_cap_range_nm=[float(a['master_effort_cap_nm'][ix].min()),float(a['master_effort_cap_nm'][ix].max())],
            safety_samples=int(a['safety_stop'][ix].sum()))
        # Diagnostic processing uses the unchanged common-filter measurement
        # chain, independently per complete recorded segment. It does not admit
        # failed intervals to fitting or turn incomplete validation into PASS.
        d=process(rows_at(a,ix,index['metadata'],'DIAGNOSTIC_ONLY'),prior,settings['bias'],
                  a['joint_dq'][ix],native_coordinate=q)
        if d is not None:
            edge=d['processing']['boundary_exclusion_samples']
            inside=np.arange(edge,len(ix)-edge)
            inside=inside[np.isfinite(d['unfiltered_torque'][inside])]
            wrist_effort[ix[inside]]=d['unfiltered_torque'][inside]
            r['compensated_wrist_rail_interior_N_quantiles']=np.quantile(d['unfiltered_torque'][inside],[.1,.5,.9]).tolist() if len(inside) else None
            r['native_total_rail_same_interior_N_quantiles']=np.nanquantile((friction[ix[inside]]+normal[ix[inside]]).sum(1),[.1,.5,.9]).tolist() if len(inside) else None
            good=d['valid'];r['compensated_wrist_motion_qualified_N_quantiles']=np.quantile(d['unfiltered_torque'][good],[.1,.5,.9]).tolist() if good.any() else None
            r['FT_processing']='original V3.1 segment-local single tool compensation; interior and motion-qualified statistics explicitly separated; no failed-sample admission to FIT'
        out['segments'].append(r);rows.append((ix,t))
    result_path=p/'PRELOAD_CONDITION_RESULT.json'
    if result_path.exists():
        r=json.loads(result_path.read_text());out.update(acquired=r.get('acquisition',{}).get('acquired',False),safe_exit=r.get('safe_exit',{}).get('safe',False),error=r.get('error'))
    trace_path=p/'AGENT_ACTION_TRACE.json'
    if trace_path.exists():
        out['Agent_attempts']=[dict(sequence=e['command']['sequence'],
            requested_preload_n=e['command'].get('native_g1_preload_request_n'),
            family=e['result'].get('family'),acquisition=e['result'].get('acquisition'),
            physical_grasp_started=e['result'].get('physical_grasp_started',False),
            safe_exit=e['result'].get('safe_exit',{}).get('safe',False),
            error=e['result'].get('error'),
            segments=[dict(status=s.get('status'),reason=s.get('reason'),role=s.get('role'))
                      for s in e['result'].get('segments',[])])
            for e in json.loads(trace_path.read_text())
            if e['command']['operation']=='attempt_grasp_and_probe']
    out['independent_validation_complete']=any(s['role'].startswith('VALIDATION') and s['status']=='TARGET_REACHED' for s in out['segments'])
    if plot:
        import matplotlib;matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        active=np.flatnonzero((a['segment']>0)|(phases=='FORCE_HOLD'))
        if not len(active):active=np.flatnonzero(a['positive_contacts']>0)
        if len(active):
            ix=np.arange(active[0],active[-1]+1);t=a['t'][ix]-a['t'][ix[0]]
            fig,axs=plt.subplots(5,1,figsize=(9,10),sharex=True)
            for j,name in enumerate(['Left','Right']):
                axs[0].plot(t,a['pad_load'][ix,j],lw=.65,label=name+' normal')
                axs[1].plot(t,friction[ix,j],lw=.65,label=name+' tangential rail')
            axs[1].plot(t,(normal[ix]+friction[ix]).sum(1),lw=.8,label='Total native rail')
            axs[1].plot(t,wrist_effort[ix],lw=.7,label='Compensated wrist rail, segment interior')
            axs[1].axhline(.3,ls=':',color='gray',label='Known engineering C=0.3N (not estimator input)')
            axs[2].plot(t,(a['joint_q'][ix]-a['joint_q'][ix[0]])*1000,label='Drawer')
            axs[2].plot(t,((a['tcp_pose'][ix,:3]-a['tcp_pose'][ix[0],:3])@axis)*1000,label='Actual TCP')
            axs[2].plot(t,((a['command_TCP_pose'][ix,:3]-a['command_TCP_pose'][ix[0],:3])@axis)*1000,label='Command TCP')
            axs[3].plot(t,a['command_aperture'][ix]*1000,label='Command aperture')
            axs[3].plot(t,a['aperture'][ix]*1000,label='Actual aperture')
            axs[4].plot(t,a['relative_slip_m'][ix]*1000,label='TCP/handle relative translation')
            for ax,label in zip(axs,['Pad normal N','Signed rail force N','Rail displacement mm','Aperture mm','Relative motion mm']):
                ax.set_ylabel(label);ax.legend(fontsize=7);ax.grid(alpha=.2)
            axs[-1].set_xlabel('Simulation seconds from hold/contact interval')
            fig.suptitle(f'Native G1 preload request {out["preload_request_n"]} N: measured contact and motion\nNo normal reports is not proof of separation; motor force is unavailable')
            fig.tight_layout();fig.savefig(p/'PRELOAD_FEEDBACK.png',dpi=110);plt.close(fig)
    (p/'PRELOAD_DERIVED.json').write_text(json.dumps(out,indent=2));return out


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);args=parser.parse_args()
    print(json.dumps(summary(args.folder),indent=2))
