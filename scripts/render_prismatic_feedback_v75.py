"""Attempt-scoped measured feedback, never borrowed from a preceding segment.

The centered dynamics trace is a POST-OPERATION diagnostic only. It is computed
inside contiguous native-rate runs and is not connected to any protection.
"""
import argparse, hashlib, json, sys
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.prismatic_lifecycle_v75 import attempt_indices,provenance
from interaction_runtime.serialization import result_packet


def contact_focus(a, ix, metadata):
    """Presentation window only; never changes recording or FIT eligibility."""
    phases=np.asarray(metadata['phases'])[a['phase'][ix]]
    driving=np.flatnonzero(np.isin(phases,['OPEN_CONTINUE','TRANSMISSION_PROBE']))
    contact=np.flatnonzero(np.any(a['pad_load'][ix]>0,axis=1))
    if len(driving):
        first=int(min(driving[0],contact[0]if len(contact)else driving[0]))
        last=int(driving[-1])
    elif len(contact):first,last=int(contact[0]),int(contact[-1])
    else:
        closing=np.flatnonzero(np.isin(phases,['SLOW_CLOSE','CLOSE','CENTER','GRASP_VERIFY','HOLD']))
        first,last=(int(closing[0]),int(closing[-1]))if len(closing)else(0,len(ix)-1)
    t=a['t'][ix]
    start=int(np.searchsorted(t,t[first]-.5));end=min(len(ix),int(np.searchsorted(t,t[last]+.5,side='right')))
    return start,end


def contact_geometry(folder, destination, identity):
    """Only the requested attempt AND time interval; absent points stay absent."""
    import matplotlib.pyplot as plt
    p=folder/'contact_evidence/index.json'
    if not p.exists():return dict(available=False,reason='POINT_CONTACT_RECORD_UNAVAILABLE')
    index=json.loads(p.read_text());data={};sources=[]
    for chunk in index['files']:
        path=p.parent/chunk['file']
        with np.load(path,allow_pickle=False) as x:
            keep=(x['attempt']==identity.get('native_point_attempt_id',identity['attempt_id']))&(x['step']>=identity['first_step'])&(x['step']<identity['last_step_exclusive'])
            if not keep.any():continue
            sources.append(dict(file=str(path.relative_to(folder)),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            for k in x.files:data.setdefault(k,[]).append(x[k][keep])
    if not data:return dict(available=False,reason='NO_POINT_CONTACTS_IN_CURRENT_ATTEMPT',previous_attempt_fallback=False)
    data={k:np.concatenate(v)for k,v in data.items()};fig,axes=plt.subplots(1,2,figsize=(8,4));summary=[]
    for j,(ax,name) in enumerate(zip(axes,['gripper_link1','gripper_link2'])):
        bounds=np.asarray(index['pad_bounds'][name])*1000
        ax.plot(bounds[[0,1,1,0,0],0],bounds[[0,0,1,1,0],1],color='black')
        ix=np.flatnonzero((data['finger']==j)&(data['kind']==0))
        if len(ix):
            points=data['point_pad'][ix]*1000
            ax.scatter(points[:,0],points[:,1],c=data['step'][ix]*index['physics_dt_s'],s=2)
        summary.append(dict(finger=name,normal_point_samples=len(ix),missing_positive_normal_is_not_proven_separation=True))
        ax.set(title=name,xlabel='pad lateral (mm)',ylabel='pad longitudinal (mm)');ax.axis('equal');ax.grid(alpha=.2)
    fig.suptitle('Current attempt only: native contact points are not pressure areas');fig.tight_layout()
    path=destination/'CONTACT_GEOMETRY.png';fig.savefig(path,dpi=110);plt.close(fig)
    return dict(available=True,path=str(path),summary=summary,source_files=sources,previous_attempt_fallback=False)


def render(folder, identity):
    from diagnose_probing_v3 import columns,rows_at
    from xarm_ft_v31_estimator import process
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(folder);a,index=columns(folder);ix=attempt_indices(a,identity)
    meta=provenance(folder,index,ix,a,identity)
    meta.update(created_utc=datetime.now(timezone.utc).isoformat(),renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    destination=folder/'contact_feedback'/('attempt_%02d_generation_%02d_steps_%d_%d'%(identity['attempt_id'],identity['observation_generation'],identity['first_step'],identity['last_step_exclusive']))
    destination=destination/meta['renderer_sha256'][:12]
    destination.mkdir(parents=True,exist_ok=True)
    cache=destination/'INDEX.json'
    if cache.exists():
        old=json.loads(cache.read_text())
        if old.get('source_files')==meta['source_files']:return old
    if not len(ix):
        result=dict(meta,available=False,reason='NO_CURRENT_ATTEMPT_NUMERIC_SAMPLES')
        (destination/'INDEX.json').write_text(json.dumps(result_packet(result),indent=2)+'\n');return result
    settings=json.loads((folder/'identification_input.json').read_text());axis=np.asarray(settings['prior']['axis_world'],float);axis/=np.linalg.norm(axis)
    rows=rows_at(a,ix,index['metadata'],'POST_OPERATION_DIAGNOSTIC')
    ft=[r['wrist_ft']for r in rows];R=np.array([f['T_world_sensor'][:3,:3]for f in ft])
    raw=a['raw_ft'][ix];world_raw=np.c_[np.einsum('nij,nj->ni',R,raw[:,:3]),np.einsum('nij,nj->ni',R,raw[:,3:])]
    unbiased=raw-settings['bias'];world=np.c_[np.einsum('nij,nj->ni',R,unbiased[:,:3]),np.einsum('nij,nj->ni',R,unbiased[:,3:])]
    gravity=world+np.array([f['tool_gravity_wrench_world']for f in ft]);full=np.full_like(gravity,np.nan)
    # Numerical dynamics compensation requires contiguous native-rate data.
    runs=np.split(np.arange(len(ix)),np.flatnonzero((np.diff(a['step'][ix])!=1)|(abs(np.diff(a['t'][ix])-1/240)>1e-7))+1)
    for run in runs:
        if len(run)<247:continue
        rr=[rows[j]for j in run];q=a['joint_q'][ix[run]];v=a['joint_dq'][ix[run]]
        p=process(rr,settings['prior'],settings['bias'],v,native_coordinate=q)
        # Move only inertia moment from the estimator's fixed hinge reference
        # back to the SENSOR origin, preserving the same complete six-vector.
        inertia=p['inertial_world_hinge'].copy()
        offset=np.array([r['wrist_ft']['T_world_sensor'][:3,3]for r in rr])-settings['prior']['origin_world_m']
        inertia[:,3:]-=np.cross(offset,inertia[:,:3])
        interior=run[121:-121];full[interior]=gravity[interior]-inertia[121:-121]
    focus_start,focus_end=contact_focus(a,ix,index['metadata'])
    t=a['t'][ix]-a['t'][ix[0]];fig,ax=plt.subplots(8,1,figsize=(10,14),sharex=True)
    for j,name in enumerate(['L raw normal','R raw normal']):ax[0].plot(t,a['pad_load'][ix,j],label=name,lw=.7)
    ax[0].set_ylabel('Pad (N)')
    for field,label in [('aperture','measured'),('command_aperture','command')]:ax[1].plot(t,a[field][ix]*1000,label=label)
    ax[1].set_ylabel('Jaw (mm)')
    ax[2].plot(t,a['joint_q'][ix]*1000,label='measured');ax[2].plot(t,a['reference_q'][ix]*1000,label='reference');ax[2].set_ylabel('Drawer (mm)')
    if 'command_TCP_pose' in a:
        measured=a['tcp_pose'][ix,:3]@axis;command=a['command_TCP_pose'][ix,:3]@axis
        ax[3].plot(t,(command-measured)*1000,label='command - measured TCP, rail')
        ax[3].plot(t,((measured-measured[focus_start])-(a['joint_q'][ix]-a['joint_q'][ix[focus_start]]))*1000,label='measured TCP - drawer travel, contact-window zero')
    ax[3].set_ylabel('Two errors (mm)')
    for label,w in [('raw',world_raw),('bias+gravity',gravity),('full dynamic (offline)',full)]:
        rail=w[:,:3]@axis;trans=w[:,:3]-rail[:,None]*axis
        ax[4].plot(t,rail,label=label,lw=.6);ax[5].plot(t,np.linalg.norm(trans,axis=1),label=label,lw=.6)
        ax[6].plot(t,np.linalg.norm(w[:,3:],axis=1),label=label,lw=.6)
    ax[4].set_ylabel('Rail force (N)');ax[5].set_ylabel('Transverse (N)');ax[6].set_ylabel('Sensor torque (Nm)')
    if 'normal_force_on_fingers_world_N' in a:
        normal=-a['normal_force_on_fingers_world_N'][ix].sum(1)@axis
        ax[7].plot(t,normal,label='native normal reaction, rail',lw=.7)
    if 'friction_force_on_fingers_world_N' in a:
        friction=-a['friction_force_on_fingers_world_N'][ix].sum(1)@axis
        valid=np.all(a['friction_api_valid'][ix]>0,axis=1)
        ax[7].plot(t,np.where(valid,friction,np.nan),label='native anchor reaction, rail',lw=.7)
    ax[7].set_ylabel('Diagnostic traction (N)');ax[7].set_xlabel('Seconds from THIS attempt start')
    for panel in ax:
        panel.grid(alpha=.2)
        panel.set_xlim(t[focus_start],t[focus_end-1])
        if panel.get_legend_handles_labels()[0]:panel.legend(fontsize=7)
    # Set y limits from the displayed native samples, not the hundreds of mm of
    # unloaded approach/return. No averaging, decimation or clipping of peaks.
    for panel in ax:
        values=[]
        for line in panel.lines:
            y=np.asarray(line.get_ydata())[focus_start:focus_end];values.extend(y[np.isfinite(y)])
        if len(values):
            lo,hi=float(min(values)),float(max(values));pad=max((hi-lo)*.06,1e-6)
            panel.set_ylim(lo-pad,hi+pad)
    fig.suptitle('Attempt %d / observation generation %d | acquisition/loaded window, all native samples\nFull compensation is offline only; missing normal reports do not prove separation'%(identity['attempt_id'],identity['observation_generation']))
    fig.tight_layout();path=destination/'CONTACT_FEEDBACK.png';fig.savefig(path,dpi=110);plt.close(fig)
    geometry=contact_geometry(folder,destination,identity)
    diagnostics=[]
    for segment in index.get('segments',[]):
        jj=np.flatnonzero((a['step'][ix]>=segment['first_step'])&(a['step'][ix]<segment.get('last_step',identity['last_step_exclusive'])))
        if len(jj)<2:continue
        start,end=jj[0],jj[-1];q=a['joint_q'][ix]
        row=dict(role=segment['role'],status=segment.get('result',{}).get('status'),
            measured_drawer_travel_m=float(q[end]-q[start]),duration_s=float(t[end]-t[start]),
            last_pad_normal_mean_n=a['pad_load'][ix[jj[-36:]]].mean(0).tolist())
        if 'command_TCP_pose'in a:
            tcp=a['tcp_pose'][ix,:3]@axis;cmd=a['command_TCP_pose'][ix,:3]@axis
            row.update(command_TCP_travel_m=float(cmd[end]-cmd[start]),measured_TCP_travel_m=float(tcp[end]-tcp[start]),
                command_to_measured_TCP_error_change_m=float((cmd[end]-tcp[end])-(cmd[start]-tcp[start])),
                measured_TCP_to_drawer_relative_travel_m=float((tcp[end]-tcp[start])-(q[end]-q[start])))
        diagnostics.append(row)
    result=dict(meta,available=True,path=str(path),contact_geometry=geometry,measured_only=True,
        displayed_window_relative_s=[float(t[focus_start]),float(t[focus_end-1])],
        presentation_only_no_recording_or_FIT_changes=True,segment_motion_diagnostics=diagnostics,
        raw_normal_peaks_n=a['pad_load'][ix].max(0).tolist(),joint_travel_m=float(np.ptp(a['joint_q'][ix])),
        full_compensation_samples=int(np.isfinite(full[:,0]).sum()),
        origin='instantaneous sensor origin; world-oriented six-axis wrench',
        diagnostic_is_not_online_safety_channel=True,full_compensation_requires_contiguous_native_rate=True)
    (destination/'INDEX.json').write_text(json.dumps(result_packet(result),indent=2)+'\n');return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('identity',type=Path);a=p.parse_args()
    print(json.dumps(result_packet(render(a.episode,json.loads(a.identity.read_text()))),indent=2))
