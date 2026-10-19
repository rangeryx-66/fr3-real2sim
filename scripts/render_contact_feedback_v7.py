"""Measured-only Agent contact panel, including failed/no-FIT operations."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def render(folder):
    from diagnose_probing_v3 import columns,rows_at
    from xarm_wrist_ft import wrench_at
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(folder);a,index=columns(folder);phases=np.array(index['metadata']['phases'])[a['phase']]
    active=np.flatnonzero(a['segment']>0)
    if not len(active):
        active=np.flatnonzero(a['positive_contacts']>0)
        if not len(active):return dict(available=False,reason='no interaction or positive normal contact yet')
        end=active[-1];active=active[a['t'][active]>=a['t'][end]-120];sid=0
    else:
        end=active[-1];sid=a['segment'][end];active=np.flatnonzero(a['segment']==sid)
    start=active[0]
    ix=np.flatnonzero((a['t']>=a['t'][start]-2)&(a['t']<=a['t'][end]+.5));t=a['t'][ix]-a['t'][start]
    settings=json.loads((folder/'identification_input.json').read_text());units=settings['estimation']['units'];linear=units['coordinate']=='m';factor=1000 if linear else 180/np.pi
    rows=rows_at(a,ix,index['metadata'],'AGENT_DIAGNOSTIC_ONLY');w=[]
    for row in rows:
        ft=row['wrist_ft'];T=ft['T_world_sensor'];w.append(wrench_at(np.asarray(ft['raw_wrench_sensor'])-settings['bias'],T,T[:3,3])+ft['tool_gravity_wrench_world'])
    w=np.asarray(w);axis=np.asarray(settings['prior']['axis_world']);axis/=np.linalg.norm(axis)
    fig,ax=plt.subplots(6,1,figsize=(10,12),sharex=True)
    for j,name in enumerate(['L raw normal','R raw normal']):ax[0].plot(t,a['pad_load'][ix,j],label=name,lw=.7)
    ax[0].set_ylabel('Pad normal (N)');ax[0].legend(fontsize=8)
    ax[1].plot(t,a['aperture'][ix]*1000,label='measured');ax[1].plot(t,a['command_aperture'][ix]*1000,label='command');ax[1].set_ylabel('Aperture (mm)');ax[1].legend(fontsize=8)
    ax[2].plot(t,a['joint_q'][ix]*factor,label='actual joint');ax[2].plot(t,a['reference_q'][ix]*factor,label='reference');ax[2].set_ylabel('Joint ('+('mm' if linear else 'deg')+')');ax[2].legend(fontsize=8)
    ax[3].plot(t,a['relative_slip_m'][ix]*1000,label='TCP/handle translation');ax[3].set_ylabel('Relative drift (mm)')
    if 'command_TCP_pose' in a:
        command=a['command_TCP_pose'][ix,:3]@axis;measured=a['tcp_pose'][ix,:3]@axis
        ax[4].plot(t,(command-measured)*1000,label='command - measured TCP, rail')
        ax[4].plot(t,((measured-measured[0])-(a['joint_q'][ix]-a['joint_q'][ix[0]]))*1000,label='measured TCP - drawer travel')
        ax[4].set_ylabel('Separate errors (mm)');ax[4].legend(fontsize=7)
    for j,name in enumerate(['Fx','Fy','Fz']):ax[5].plot(t,w[:,j],label=name,lw=.7)
    if 'friction_force_on_fingers_world_N' in a:
        friction=-a['friction_force_on_fingers_world_N'][ix].sum(1)@axis
        valid=np.any(a['friction_anchor_counts'][ix]>0,axis=1)&np.all(a['friction_api_valid'][ix]>0,axis=1)
        ax[5].plot(t,np.where(valid,friction,np.nan),label='native friction reaction, rail',lw=1)
    ax[5].set_ylabel('Wrist / traction (N)');ax[5].legend(fontsize=7);ax[5].set_xlabel('Seconds from latest segment start')
    for axis_plot in ax:axis_plot.axvline(0,color='gray',ls=':');axis_plot.grid(alpha=.2)
    fig.suptitle('Actual native contact / motion feedback | force: bias + gravity compensated, tool inertia retained\nNo positive normal report does not prove separation; no hidden dynamics/GT contact geometry')
    fig.tight_layout();dest=folder/'CONTACT_FEEDBACK.png';fig.savefig(dest,dpi=120);plt.close(fig)
    contact_geometry=render_contact_geometry(folder)
    return dict(available=True,path=str(dest),contact_geometry=contact_geometry,segment=int(sid),plot_samples=len(ix),measured_only=True,force_processing='native wrist cut, known bias and gravity only; dynamic tool loading retained',raw_normal_peaks_n=a['pad_load'][ix].max(0).tolist(),joint_travel_native=float(np.ptp(a['joint_q'][active])),max_relative_drift_m=float(a['relative_slip_m'][ix].max()))


def render_contact_geometry(folder):
    import matplotlib.pyplot as plt
    p=folder/'contact_evidence/index.json'
    if not p.exists():return dict(available=False,reason='point-level native contacts were not recorded')
    index=json.loads(p.read_text());data={}
    for chunk in index['files']:
        with np.load(p.parent/chunk['file']) as x:
            for k in x.files:data.setdefault(k,[]).append(x[k])
    if not data:return dict(available=False,reason='no native point samples')
    data={k:np.concatenate(v) for k,v in data.items()};normal=(data['kind']==0)&(data['attempt']==data['attempt'].max())
    fig,axes=plt.subplots(1,2,figsize=(10,5));summary=[]
    for j,(ax,name) in enumerate(zip(axes,['gripper_link1','gripper_link2'])):
        ix=np.flatnonzero(normal&(data['finger']==j));b=np.array(index['pad_bounds'][name])*1000
        ax.plot([b[0,0],b[1,0],b[1,0],b[0,0],b[0,0]],[b[0,1],b[0,1],b[1,1],b[1,1],b[0,1]],color='black',label='official inner pad bounds')
        if len(ix):
            points=data['point_pad'][ix]*1000;time=data['step'][ix]*index['physics_dt_s'];artist=ax.scatter(points[:,0],points[:,1],c=time,s=2,cmap='viridis');fig.colorbar(artist,ax=ax,label='simulation time (s)')
            summary.append(dict(finger=name,native_normal_points=len(ix),minimum_longitudinal_margin_mm=float(data['longitudinal_margin_m'][ix].min()*1000),minimum_lateral_margin_mm=float(data['lateral_margin_m'][ix].min()*1000),native_normals_preserved=True))
        ax.set(title=name,xlabel='pad lateral (mm)',ylabel='pad longitudinal (mm)');ax.axis('equal');ax.grid(alpha=.2)
    fig.suptitle('Archived actual contact locations in measured finger frames\nPoint samples are not pressure areas; pad edges are diagnostic, not new execution vetoes');fig.tight_layout();dest=folder/'CONTACT_GEOMETRY.png';fig.savefig(dest,dpi=120);plt.close(fig)
    return dict(available=True,path=str(dest),summary=summary)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();print(json.dumps(render(a.folder),indent=2))
