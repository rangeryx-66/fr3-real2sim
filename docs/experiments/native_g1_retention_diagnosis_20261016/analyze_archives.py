"""Native-rate, offline archive comparison. No Isaac imports or execution.

Raw archives and full-resolution NPZ/contact products are private. Figures plot
every retained 240 Hz sample; summaries do not replace or edit raw evidence.
"""
import argparse, gzip, hashlib, json, pickle
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LABELS=['gripper_link1','gripper_link2']
COLORS=['#2769a8','#d15a24']

def read(p): return json.loads(p.read_text())
def dump(p,x): p.write_text(json.dumps(x,indent=2,allow_nan=False))
def records(ep,name,lo=-np.inf,hi=np.inf):
    folder=ep/(name+'_chunks');out=[]
    for c in read(folder/'index.json')['chunks']:
        rows=pickle.load(gzip.open(folder/c['file'],'rb'))
        if rows[0]['t']>hi:break
        if rows[-1]['t']<lo:continue
        out.extend(r for r in rows if lo<=r['t']<=hi)
    return out
def point(T,p):return (np.asarray(T)@np.r_[p,1.])[:3]
def cooked_mesh(s):
    return trimesh.util.concatenate([trimesh.Trimesh(c['vertices'],[[p[0],p[i],p[i+1]] for p in c['polygons'] for i in range(1,len(p)-1)],process=False) for c in s['convexes']])
def transform(mesh,T):
    m=mesh.copy();m.apply_transform(T);return m
def polygon_axis_margins(p,poly):
    """Signed separate x/y distances to a convex pad-face boundary; diagnostic."""
    from scipy.spatial import ConvexHull
    eq=ConvexHull(poly[:,:2]).equations
    value=eq[:,:2]@p[:2]+eq[:,2];out=[]
    for axis in [0,1]:
        candidates=[-v/abs(e[axis]) for e,v in zip(eq,value) if abs(e[axis])>1e-10]
        out.append(min(candidates))
    return out
def intervals(t,mask,dt=1/240):
    edges=np.diff(np.r_[False,mask,False].astype(int));return [[float(t[i]),float(t[j-1]+dt)] for i,j in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1))]

def analyze(root,label,cal,private,figures):
    ep=root/'episode';report=read(ep/'report.json');acq=report['grasp_acquisition'];zero=acq['t']
    events=read(ep/'events.json');events=events if isinstance(events,list) else events['events']
    opening=next(e for e in events if e['event']=='SHORT_OPENING_START');start=opening['t']-zero
    obs=records(ep,'observations',zero-10);phy=records(ep,'physics_steps',zero-10)
    t=np.array([r['t']-zero for r in obs]);tp=np.array([r['t']-zero for r in phy]);q=np.array([r['q'] for r in obs]);meta=read(ep/'sensor_metadata.json')
    raw=np.array([[r['telemetry']['pad_load_raw_n'][n] for n in LABELS] for r in obs]);filtered=np.array([r['telemetry']['pad_load_filtered_n'] for r in obs])
    cmd=np.array([r['telemetry']['command_aperture_m']*1000 for r in obs]);ap=np.array([r['aperture_m']*1000 for r in obs])
    vel=np.array([r['telemetry']['joint_velocities'] for r in obs]);eff=np.array([r['telemetry']['measured_joint_efforts'] for r in obs]);app=np.array([r['telemetry']['applied_joint_efforts'] for r in obs]);qcmd=np.array([r['telemetry']['command_q_all'] for r in obs])
    ref=int(np.argmin(abs(t)));pr=int(np.argmin(abs(tp)));Trel=np.array([np.linalg.inv(r['T_moving_link'])@np.asarray(r['T_tcp']) for r in obs]);motion=(Trel[:,:3,3]-Trel[ref,:3,3])*1000
    rot=np.rad2deg(Rotation.from_matrix(Trel[:,:3,:3]@Trel[ref,:3,:3].T).magnitude())
    speed=np.r_[np.nan,np.linalg.norm(np.diff(np.array([r['T_tcp'] for r in obs])[:,:3,3],axis=0),axis=1)/np.diff(t)*1000]
    track=np.array([np.linalg.norm(np.asarray(r['T_tcp'])[:3,3]-np.asarray(r['planned_T_tcp'])[:3,3])*1000 if r.get('planned_T_tcp') is not None else np.nan for r in obs])
    pads={n:np.asarray(cal['pads'][n]) for n in LABELS};pose_contacts=[];centers=[];seps=[];loaded=[]
    cooked=read(ep/'cooked_initial.json');finger_solids={};cooked_pads={}
    for n,body in zip(LABELS,['left_finger','right_finger']):
        s=next(s for s in cooked['shapes'] if s.get('rigid_body_path','').split('/')[-1]==body)
        finger_solids[n]=transform(cooked_mesh(s),np.linalg.inv(cal['PAD_FRAMES'][body])@np.linalg.inv(s['rigid_body_world_transform'])@np.asarray(s['world_transform']))
        vertices=finger_solids[n].vertices;z=vertices[:,2].min();cooked_pads[n]=vertices[vertices[:,2]<z+1e-6]
    first=records(ep,'physics_steps',hi=1/240)[0];M0=np.asarray(first['T_moving_link'])
    allowed=meta['allowed_handle_colliders'];handle=trimesh.util.concatenate([transform(cooked_mesh(s),np.linalg.inv(M0)@np.asarray(s['world_transform'])) for s in cooked['shapes'] if s['path'] in allowed])
    # Handle-relative diagnostic frame: longitudinal PCA; transverse global-X projection.
    _,_,V=np.linalg.svd(handle.vertices-handle.vertices.mean(0),full_matrices=False);long=V[0]
    if (M0[:3,:3]@long)[2]<0:long=-long
    cross=M0[:3,:3].T@np.array([1.,0,0]);cross-=long*(cross@long);cross/=np.linalg.norm(cross);depth=np.cross(long,cross)
    H=np.eye(4);H[:3,:3]=np.column_stack((cross,depth,long));H[:3,3]=handle.vertices.mean(0);Hi=np.linalg.inv(H)
    handle_h=transform(handle,Hi);handle_h.export(private/(label+'_handle_relative_cooked.ply'))
    for i,r in enumerate(phy):
        D=np.linalg.inv(r['T_moving_link']);centers.append([point(D@np.asarray(r['finger_world_poses'][n]),pads[n].mean(0)) for n in LABELS])
        side_sep=[];side_load=[]
        for n in LABELS:
            P=np.asarray(r['finger_world_poses'][n]);cs=[c for c in r.get('contacts',[]) if c.get('finger')==n and c.get('allowed_pad_target')]
            side_sep.append(min([c['separation_m'] for c in cs],default=np.nan)*1000);side_load.append(sum(c.get('force_n',0) for c in cs))
            for c in cs:
                pw=np.asarray(c['contact_point_world_m']);pl=point(np.linalg.inv(P),pw);ph=point(Hi@D,pw)
                # Native report normal sign is collider-order dependent; orient consistently as impulse on finger when nonzero.
                imp=np.asarray(c['impulse_world_ns']);nv=np.asarray(c['native_normal_world']);norm=np.linalg.norm(imp)
                onfinger=imp/norm if norm>0 else None
                cooked_margin=polygon_axis_margins(pl,cooked_pads[n])
                pose_contacts.append(dict(t=float(tp[i]),finger=n,force_n=c.get('force_n',0),positive=c.get('force_n',0)>0,
                    pad_local_mm=(pl*1000).tolist(),handle_local_mm=(ph*1000).tolist(),
                    longitudinal_edge_margin_mm=float(min(pl[1]-pads[n][0,1],pads[n][1,1]-pl[1])*1000),
                    lateral_edge_margin_mm=float(min(pl[0]-pads[n][0,0],pads[n][1,0]-pl[0])*1000),
                    cooked_lateral_edge_margin_mm=float(cooked_margin[0]*1000),cooked_longitudinal_edge_margin_mm=float(cooked_margin[1]*1000),
                    pad_plane_gap_mm=float((pl[2]-pads[n][0,2])*1000),separation_mm=c['separation_m']*1000,
                    native_normal_world=nv.tolist(),normal_in_pad=(P[:3,:3].T@nv).tolist(),
                    impulse_direction_in_handle=None if onfinger is None else (Hi[:3,:3]@D[:3,:3]@onfinger).tolist(),
                    collider0=c['collider0'],collider1=c['collider1']))
        seps.append(side_sep);loaded.append(side_load)
    centers=np.array(centers);seps=np.array(seps);loaded=np.array(loaded)
    axis=centers[pr,0]-centers[pr,1];axis/=np.linalg.norm(axis)
    disp=(centers-centers[pr])*1000;closing_disp=disp@axis;gap=(centers[:,0]-centers[:,1])@axis*1000
    driver=meta['joint_names'].index('drive_joint');angle=np.array(cal['width_angle_grid_rad']);width=np.array(cal['width_grid_m']);jac=np.gradient(width,angle);caps=30*abs(np.interp(q[:,driver],angle,jac))
    mimic=q[:,7:]-q[:,driver,None];hold=(t>=0)&(t<start);after=t>=0
    pdelta=closing_disp[(tp>=0)&(tp<start)]
    marks=[0,.05,.10,.20,.2291666667,.2541666667,.5,start,min(start+.32,t[-1])]
    states=[]
    for z in marks:
        i=int(np.argmin(abs(t-z)));k=int(np.argmin(abs(tp-z)));r=obs[i]
        pos=[c for c in pose_contacts if abs(c['t']-tp[k])<1e-8]
        states.append(dict(requested_after_verification_s=z,t=float(t[i]),physics_t=float(tp[k]),phase=r['phase'],
            aperture_mm=float(ap[i]),command_mm=float(cmd[i]),raw_n=raw[i].tolist(),filtered_n=filtered[i].tolist(),
            q=q[i,7:].tolist(),q_change_rad=(q[i,7:]-q[ref,7:]).tolist(),q_velocity=vel[i,7:].tolist(),q_effort_nm=eff[i,7:].tolist(),
            mimic_error_rad=mimic[i].tolist(),master_cap_nm=float(caps[i]),coupled_pad_gap_mm=float(gap[k]),
            relative_translation_mm=motion[i].tolist(),relative_rotation_deg=float(rot[i]),pad_closing_displacement_mm=closing_disp[k].tolist(),
            separation_report_min_mm=[None if not np.isfinite(v) else float(v) for v in seps[k]],contacts=pos))
    contact_summary={}
    for phase,lo,hi in [('acquisition',-.02,.02),('hold_early',0,.23),('hold_late',.26,start),('opening',start,t[-1]+1e-4)]:
        contact_summary[phase]={}
        for n in LABELS:
            cs=[c for c in pose_contacts if c['finger']==n and lo<=c['t']<hi and c['positive']]
            if not cs:contact_summary[phase][n]=dict(positive_records=0);continue
            contact_summary[phase][n]=dict(positive_records=len(cs),
                longitudinal_margin_mm=[float(np.min([c['longitudinal_edge_margin_mm'] for c in cs])),float(np.median([c['longitudinal_edge_margin_mm'] for c in cs]))],
                lateral_margin_mm=[float(np.min([c['lateral_edge_margin_mm'] for c in cs])),float(np.median([c['lateral_edge_margin_mm'] for c in cs]))],
                cooked_longitudinal_margin_mm=[min(c['cooked_longitudinal_edge_margin_mm'] for c in cs),float(np.median([c['cooked_longitudinal_edge_margin_mm'] for c in cs]))],
                cooked_lateral_margin_mm=[min(c['cooked_lateral_edge_margin_mm'] for c in cs),float(np.median([c['cooked_lateral_edge_margin_mm'] for c in cs]))],
                pad_local_range_mm=np.array([c['pad_local_mm'] for c in cs]).min(0).tolist()+np.array([c['pad_local_mm'] for c in cs]).max(0).tolist(),
                separation_mm_range=[min(c['separation_mm'] for c in cs),max(c['separation_mm'] for c in cs)])
    stats=dict(label=label,root=str(root),acquisition=acq,opening_start_after_verification_s=start,
        observed_rows=len(obs),physics_rows=len(phy),full_resolution_hz=240,averaging_added=False,
        hold_command_mm=[float(cmd[hold].min()),float(cmd[hold].max())],
        hold_measured_mm=[float(ap[hold][0]),float(ap[hold][-1]),float(ap[hold].min()),float(ap[hold].max())],
        hold_gap_actual_pad_centers_mm=[float(gap[np.flatnonzero(tp>=0)[0]]),float(gap[np.flatnonzero(tp<start)[-1]])],
        hold_pad_center_displacement_closing_axis_mm=pdelta[-1].tolist(),
        hold_max_mimic_error_rad=float(abs(mimic[hold]).max()),hold_mimic_error_range_rad=np.array([mimic[hold].min(0),mimic[hold].max(0)]).tolist(),
        hold_master_effort_nm=[float(eff[hold,driver].min()),float(eff[hold,driver].max())],
        master_cap_nm_range=[float(caps[hold].min()),float(caps[hold].max())],
        hold_max_measured_master_effort_over_cap=float(np.max(abs(eff[hold,driver])/caps[hold])),
        measured_effort_is_generalized_constraint_reaction_not_isolated_motor_torque=True,
        hold_arm_command_max_change_rad=float(abs(qcmd[hold,:7]-qcmd[ref,:7]).max()),
        hold_arm_measured_max_change_rad=float(abs(q[hold,:7]-q[ref,:7]).max()),
        applied_external_effort_max=float(abs(app[hold,7:]).max()),
        hold_relative_translation_max_mm=float(np.linalg.norm(motion[hold],axis=1).max()),hold_relative_rotation_max_deg=float(rot[hold].max()),
        post_acquisition_max_raw_n=raw[after].max(0).tolist(),post_acquisition_max_filtered_n=filtered[after].max(0).tolist(),
        post_acquisition_gain_deg=float(max(r['door_angle_deg'] for r in obs if r['t']>=zero)-acq['angle_deg']),
        no_positive_normal_intervals_after_acquisition_s={n:intervals(tp,(tp>=0)&(loaded[:,k]==0)) for k,n in enumerate(LABELS)},
        contact_geometry_summary=contact_summary,states=states,
        force_events=[dict(**r['force_event'],aligned_t=r['t']-zero) for r in obs if (r.get('force_event') or {}).get('status') in ['SOFT_FORCE_WARNING','HARD_FORCE_STOP_SUSTAINED']],
        pad_bounds_mm={n:(b*1000).tolist() for n,b in pads.items()},cooked_inner_face_vertices_mm={n:(b*1000).tolist() for n,b in cooked_pads.items()})
    dump(private/(label+'_CONTACTS.json'),pose_contacts);dump(private/(label+'_SUMMARY_FULL.json'),stats)
    np.savez_compressed(private/(label+'_NATIVE_RATE.npz'),t=t,tp=tp,raw=raw,filtered=filtered,q=q,qcmd=qcmd,velocity=vel,effort=eff,command_mm=cmd,aperture_mm=ap,master_cap_nm=caps,relative_mm=motion,relative_deg=rot,closing_displacement_mm=closing_disp,pad_gap_mm=gap,separation_mm=seps,normal_load=loaded,tcp_speed_mm_s=speed,tcp_tracking_mm=track,door=np.array([r['door_angle_deg']-acq['angle_deg'] for r in obs]))
    return dict(stats=stats,t=t,tp=tp,raw=raw,filtered=filtered,q=q,qcmd=qcmd,vel=vel,eff=eff,ap=ap,cmd=cmd,motion=motion,rot=rot,speed=speed,track=track,door=np.array([r['door_angle_deg']-acq['angle_deg'] for r in obs]),mimic=mimic,caps=caps,centers=centers,disp=closing_disp,gap=gap,seps=seps,contacts=pose_contacts,handle=handle_h,finger_solids=finger_solids,pads=pads,cooked_pads=cooked_pads,meta=meta)

def plot(runs,out):
    for view,xlim in [('ACQUISITION_HOLD',(-.6,1.4)),('ENTIRE_POST_GRASP',(-10,None))]:
        fig,axs=plt.subplots(7,2,figsize=(14,17),sharex='col')
        for col,(label,d) in enumerate(runs.items()):
            t=d['t'];ax=axs[:,col]
            for k,n in enumerate(LABELS):
                ax[k].plot(t,d['raw'][:,k],color=COLORS[k],lw=.65,alpha=.65,label='raw 240 Hz');ax[k].plot(t,d['filtered'][:,k],color='black',lw=1,label='existing filtered')
                ax[k].set_ylabel(n+' normal (N)');ax[k].legend(fontsize=7)
            ax[2].plot(t,d['ap'],label='measured (master-angle map)',lw=.8);ax[2].plot(t,d['cmd'],label='command',lw=.9);ax[2].set_ylabel('Aperture (mm)');ax[2].legend(fontsize=7)
            ax[3].plot(t,np.linalg.norm(d['motion'],axis=1),label='translation mm');ax[3].plot(t,d['rot'],label='rotation deg');ax[3].set_ylabel('TCP/handle change');ax[3].legend(fontsize=7)
            ax[4].plot(t,d['door'],lw=.8);ax[4].set_ylabel('Net actual opening (deg)')
            ax[5].plot(d['tp'],d['disp'][:,0],label='left');ax[5].plot(d['tp'],d['disp'][:,1],label='right');ax[5].set_ylabel('Pad shift / closing axis (mm)');ax[5].legend(fontsize=7)
            for k in range(2):ax[6].plot(d['tp'],d['seps'][:,k],color=COLORS[k],lw=.8,label=['left','right'][k])
            ax[6].set_ylabel('Min report separation (mm)');ax[6].legend(fontsize=7)
            for a in ax:
                a.axvline(0,color='grey',ls=':');a.axvline(d['stats']['opening_start_after_verification_s'],color='#648a39',ls='--');a.grid(alpha=.2)
                hi=xlim[1] if xlim[1] else t[-1]
                for line in a.lines:
                    xx=np.asarray(line.get_xdata());yy=np.asarray(line.get_ydata())
                    if len(xx)>2:
                        keep=(xx>=xlim[0])&(xx<=hi);line.set_data(xx[keep],yy[keep])
                a.relim();a.autoscale_view();a.set_xlim(xlim[0],hi);a.set_xlabel('Seconds from GRASP_VERIFIED')
                statuses={}
                for event in d['stats']['force_events']:statuses.setdefault(event['status'],event['aligned_t'])
                for status,color in [('SOFT_FORCE_WARNING','#c68c32'),('HARD_FORCE_STOP_SUSTAINED','#ba2929')]:
                    if status in statuses:a.axvline(statuses[status],color=color,ls=':',lw=.7)
            ax[0].set_title(label+' / every recorded sample')
        fig.suptitle('Aligned native 240 Hz evidence; raw peaks retained. Dashed = opening start.\nNo-positive normal impulse is not, by itself, proof of separation.',fontsize=13)
        fig.tight_layout();fig.savefig(out/(view+'.png'),dpi=150);plt.close(fig)
    fig,axs=plt.subplots(4,2,figsize=(15,11),sharex=True)
    for col,(label,d) in enumerate(runs.items()):
        t=d['t'];ref=np.argmin(abs(t));names=d['meta']['joint_names'][7:]
        for k,n in enumerate(names):
            axs[0,col].plot(t,(d['q'][:,7+k]-d['q'][ref,7+k])*1000,lw=.8,label=n)
            axs[1,col].plot(t,d['vel'][:,7+k],lw=.6,label=n)
            axs[2,col].plot(t,d['eff'][:,7+k],lw=.7,label=n)
            axs[3,col].plot(t,d['mimic'][:,k]*1000,lw=.7,label=n)
        for row,name in enumerate(['Joint change (mrad)','Native velocity (rad/s)','Measured generalized effort (Nm)','Joint - master (mrad)']):
            a=axs[row,col];a.set_ylabel(name)
            for line in a.lines:
                xx=np.asarray(line.get_xdata());yy=np.asarray(line.get_ydata());keep=(xx>=-.3)&(xx<=1.4);line.set_data(xx[keep],yy[keep])
            a.relim();a.autoscale_view();a.set_xlim(-.3,1.4);a.axvline(0,color='grey',ls=':');a.axvline(d['stats']['opening_start_after_verification_s'],color='grey',ls='--');a.grid(alpha=.2)
        axs[0,col].set_title(label);axs[0,col].legend(fontsize=6,ncol=2)
    fig.suptitle('Native G1: one position motor + five mimic constraints; reported efforts include constraint effects')
    fig.tight_layout();fig.savefig(out/'COUPLED_JOINTS.png',dpi=170);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(13,10))
    for col,(label,d) in enumerate(runs.items()):
        for row,n in enumerate(LABELS):
            ax=axs[row,col];b=d['pads'][n]*1000
            from matplotlib.patches import Rectangle
            ax.add_patch(Rectangle(b[0,:2],*(b[1,:2]-b[0,:2]),fill=False,color='black',lw=1.5))
            from scipy.spatial import ConvexHull
            poly=d['cooked_pads'][n][:,:2]*1000;v=poly[ConvexHull(poly).vertices];v=np.vstack([v,v[0]]);ax.plot(v[:,0],v[:,1],ls='--',color='#7b3e99',lw=1,label='cooked face boundary')
            for phase,lo,hi,marker,c in [('acquisition',-.02,.02,'o','#2384ad'),('hold',.02,1.00417,'.','#df8424'),('opening',1.00417,d['t'][-1]+.01,'x','#a62c2c')]:
                cs=[z for z in d['contacts'] if z['finger']==n and z['positive'] and lo<=z['t']<hi]
                if cs:
                    P=np.array([z['pad_local_mm'] for z in cs]);ax.scatter(P[:,0],P[:,1],s=9,marker=marker,alpha=.4,label=phase)
            ax.set_title(label+' / '+n);ax.set_xlabel('Pad lateral x (mm)');ax.set_ylabel('Pad longitudinal y (mm)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Actual positive-normal locations in measured logical pad frames\nBlack = planner pad rectangle; purple = cooked inner face. Points are not pressure area.')
    fig.tight_layout();fig.savefig(out/'PAD_CONTACT_LOCATIONS.png',dpi=170);plt.close(fig)
    fig=plt.figure(figsize=(13,6))
    for i,(label,d) in enumerate(runs.items()):
        ax=fig.add_subplot(1,2,i+1,projection='3d');m=d['handle'];verts=m.vertices*1000
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        ax.add_collection3d(Poly3DCollection(verts[m.faces],alpha=.12,facecolor='grey',edgecolor='grey',linewidth=.2))
        for k,n in enumerate(LABELS):
            cs=[z for z in d['contacts'] if z['positive'] and z['finger']==n and 0<=z['t']<=1.4]
            if cs:
                P=np.array([z['handle_local_mm'] for z in cs]);ax.scatter(*P.T,s=4,color=COLORS[k],label=n)
                z=cs[0];v=np.asarray(z['impulse_direction_in_handle'])*8;ax.quiver(*P[0],*v,color=COLORS[k])
        ax.set(xlim=(verts[:,0].min()-5,verts[:,0].max()+5),ylim=(verts[:,1].min()-5,verts[:,1].max()+5),zlim=(verts[:,2].min(),verts[:,2].max()),xlabel='Cross-handle x mm',ylabel='Depth y mm',zlabel='Handle length z mm',title=label)
        ax.set_box_aspect([1,1,3]);ax.legend(fontsize=7)
    fig.suptitle('Same handle-relative diagnostic frame; arrows = normal-impulse direction on finger\nGT cooked handle is shown only in post-run evaluation, never selection.')
    fig.tight_layout();fig.savefig(out/'HANDLE_CONTACT_FRAME.png',dpi=170);plt.close(fig)

def main(a):
    a.private.mkdir(parents=True,exist_ok=True);a.public.mkdir(parents=True,exist_ok=True);fig=a.public/'figures';fig.mkdir(exist_ok=True)
    cal=read(a.calibration);runs={}
    for label,root in [('success_a47fe09',a.success),('failed_f980076',a.failed)]:runs[label]=analyze(root,label,cal,a.private,fig)
    plot(runs,fig)
    # Public numerical summaries omit raw point records and private state streams.
    compact={k:{n:v for n,v in d['stats'].items() if n not in ['states','root']} for k,d in runs.items()}
    for stats in compact.values():
        events=stats['force_events'];first={};counts={}
        for event in events:
            first.setdefault(event['status'],event);counts[event['status']]=counts.get(event['status'],0)+1
        stats['force_events']=list(first.values());stats['force_status_record_counts']=counts
    dump(a.public/'ALIGNED_SUMMARY.json',compact)
    print(json.dumps({k:{n:v for n,v in d['stats'].items() if n in ['hold_measured_mm','hold_gap_actual_pad_centers_mm','hold_pad_center_displacement_closing_axis_mm','hold_max_mimic_error_rad','hold_master_effort_nm','master_cap_nm_range','hold_max_measured_master_effort_over_cap','hold_relative_translation_max_mm','contact_geometry_summary']} for k,d in runs.items()},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--success',type=Path,required=True);p.add_argument('--failed',type=Path,required=True);p.add_argument('--calibration',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--public',type=Path,required=True)
    main(p.parse_args())
