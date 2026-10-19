"""Offline surface diagnosis figure; GT/native geometry never enters execution."""
import json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from diagnose_contact_recovery_v44 import patch_saved
from verify_contact_recovery_v44 import state
from piper_mobile_demo.owned_scene import meshes
from articulated_demo.kinematics import transform


def plot(episode,cooked,output):
    g,c,s=state(episode,cooked,output.parent/'plot_context');M=g['moving']();scene=g['collision'];P,center,n,meta=patch_saved(episode/'rgbd/generation_01_agent_02',[760,320,940,540]);Q,cc,nn,_=patch_saved(episode/'rgbd/generation_01_agent_00',[305,360,360,430])
    fig=plt.figure(figsize=(10,5));ax=fig.add_subplot(121,projection='3d');bx=fig.add_subplot(122,projection='3d');native=[]
    for path,e in scene.entries.items():
        if e.get('rigid_body_path','').split('/')[-1]!=scene.moving_link:continue
        local=np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform'])
        for mesh in meshes(e):
            mesh.apply_transform(M@local);native.append(mesh)
            for a in [ax,bx]:a.plot_trisurf(*mesh.vertices.T,triangles=mesh.faces,color='#629baf',alpha=.12,linewidth=.2)
    for a in [ax,bx]:
        a.scatter(*P[::4].T,s=2,c='#d75b54',label='rejected observed inset')
        a.scatter(*Q[::2].T,s=3,c='#2d9b50',label='observed moving border')
        a.set(xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)',zlim=(.25,.75));a.legend(fontsize=7)
    ax.view_init(16,-95);bx.view_init(18,-25);fig.suptitle('Observed inset has no native contact surface; measured border matches unchanged collider',fontsize=10);fig.tight_layout();fig.savefig(output,dpi=130,bbox_inches='tight');plt.close(fig)
    root=Path(g['job']['asset_root']);urdf=next((root/'urdf').glob('*.urdf'));link=ET.parse(urdf).getroot().find("link[@name='%s']"%scene.moving_link);result=[]
    for v in link.findall('visual'):
        e=v.find('geometry/mesh');name=e.get('filename')
        if not any(x in name for x in ['original-36','original-37']):continue
        mesh=trimesh.load(urdf.parent/name,force='mesh',process=False);mesh.apply_scale(np.fromstring(e.get('scale','1 1 1'),sep=' '));o=v.find('origin');xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ') if o is not None else np.zeros(3);rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ') if o is not None else np.zeros(3);mesh.apply_transform(M@transform(xyz,rpy));_,distance,_=trimesh.proximity.closest_point(mesh,P[::4]);result.append(dict(visual_mesh=name,point_surface_mm=(1000*np.quantile(distance,[0,.5,.9,1])).tolist(),has_authored_collision=False))
    output.with_suffix('.json').write_text(json.dumps(dict(offline_only=True,omitted_visual_surfaces=result,points_never_snapped=True),indent=2));print(result)

def transmission(episode,output):
    from diagnose_probing_v3 import columns,rows_at
    from xarm_ft_v31_estimator import process
    a,index=columns(episode);s=index['segments'][0];ix=np.flatnonzero(a['segment']==s['id']);t=a['t'][ix]-a['t'][ix[0]]
    settings=json.loads((episode/'identification_input.json').read_text());d=process(rows_at(a,ix,index['metadata'],s['role']),settings['prior'],settings['bias'],a['door_dq'][ix])
    c=json.loads((episode/'EFFECTIVE_RUNTIME.json').read_text());fig,axes=plt.subplots(3,2,figsize=(10,8),constrained_layout=True)
    axes[0,0].plot(t,np.rad2deg(a['door_q'][ix]),label='actual door');axes[0,0].plot(t,np.rad2deg(a['reference_q'][ix]),label='reference');axes[0,0].set_ylabel('Angle (deg)')
    axes[0,1].plot(t,a['relative_slip_m'][ix]*1000,label='TCP/handle relative translation');axes[0,1].set_ylabel('Relative displacement (mm)')
    for k,name in enumerate(['L normal','R normal']):axes[1,0].plot(t,a['pad_load'][ix,k],lw=.7,label=name)
    axes[1,0].set_ylabel('Native normal report (N)')
    axes[1,1].plot(t,a['command_aperture'][ix]*1000,label='command');axes[1,1].plot(t,a['aperture'][ix]*1000,label='measured');axes[1,1].set_ylabel('Aperture (mm)')
    axes[2,0].plot(t,d['unfiltered_torque']*1000,lw=.4,alpha=.45,label='compensated 240Hz');axes[2,0].plot(t,d['torque']*1000,lw=.9,label='V3.1 common filter');axes[2,0].set_ylabel('Wrist-derived hinge torque (mN m)')
    axes[2,1].plot(t,np.max(abs(a['command_q'][ix]-a['q'][ix][:,c['arm_indices']]),axis=1),lw=.7,label='max arm joint error');axes[2,1].set_ylabel('Joint command error (rad)')
    for ax in axes.flat:
        ax.axvline(9.5,color='gray',ls='--',lw=.8,label='late plateau begins');ax.grid(alpha=.2);ax.set_xlabel('Interaction time (s)');ax.legend(fontsize=7)
    fig.suptitle('Archived V4.3: motion plateaus while contact reports and wrist torque persist',fontsize=11);fig.savefig(output,dpi=150,bbox_inches='tight');plt.close(fig)

def contacts(episode,output):
    from interaction_runtime.kinematics import FastNativeModel
    j=json.loads((episode/'job.json').read_text());c=json.loads((episode/'EFFECTIVE_RUNTIME.json').read_text());base=json.loads((episode/'visual_plan.json').read_text())['base']
    model=FastNativeModel(None,Path(j['asset_root']),json.loads((Path(j['source'])/'report.json').read_text()));rows=json.loads((episode/'CONTACT_CHECKPOINTS.json').read_text())['samples'];data={}
    fig,axes=plt.subplots(1,2,figsize=(9,4),constrained_layout=True);report={}
    for name,ax in zip(['gripper_link1','gripper_link2'],axes):
        pad=model.pads[name];lo=pad.min(0);hi=pad.max(0);dims=np.argsort(hi-lo)[-2:];values=[]
        for row in rows:
            if row['phase']!='OPEN_CONTINUE':continue
            q=np.array(row['q']);P=model.poses(q[c['arm_indices']],base,finger_q=q[c['finger_indices']])[name]
            for contact in row['contacts']:
                if contact.get('finger')!=name or 'contact_point_world_m' not in contact:continue
                p=(np.linalg.inv(P)@np.r_[contact['contact_point_world_m'],1])[:3];values.append([row['t'],*p])
        a=np.array(values);t=a[:,0]-a[0,0];local=a[:,1:];sc=ax.scatter(local[:,dims[0]]*1000,local[:,dims[1]]*1000,c=t,s=9,cmap='viridis')
        rect=np.array([[lo[dims[0]],lo[dims[1]]],[hi[dims[0]],lo[dims[1]]],[hi[dims[0]],hi[dims[1]]],[lo[dims[0]],hi[dims[1]]],[lo[dims[0]],lo[dims[1]]]])*1000
        ax.plot(rect[:,0],rect[:,1],color='gray',label='nominal pad bounds');ax.set(title=name,xlabel='Local axis %d (mm)'%dims[0],ylabel='Local axis %d (mm)'%dims[1]);ax.legend(fontsize=7);fig.colorbar(sc,ax=ax,label='segment time (s)')
        margins=np.minimum(local[:,dims]-lo[dims],hi[dims]-local[:,dims])*1000
        report[name]=dict(checkpoint_contacts=len(a),local_axes=dims.tolist(),pad_bounds_mm=(1000*np.array([lo,hi])).tolist(),early_margin_P05_P50_mm=np.quantile(margins[t<2],[.05,.5],axis=0).tolist(),late_margin_P05_P50_mm=np.quantile(margins[t>9.5],[.05,.5],axis=0).tolist(),local_point_change_first_last_mm=((local[-1]-local[0])*1000).tolist())
    fig.suptitle('Native failure reproduction: actual contact points in measured finger frames',fontsize=10);fig.savefig(output,dpi=150,bbox_inches='tight');plt.close(fig)
    output.with_suffix('.json').write_text(json.dumps(dict(engineering_reproduction=True,contact_checkpoint_rate_hz=10,not_historical_point_records=True,normal_reports_not_pressure_areas=True,fingers=report),indent=2))

if __name__=='__main__':
    if sys.argv[1]=='transmission':transmission(*map(Path,sys.argv[2:4]))
    elif sys.argv[1]=='contacts':contacts(*map(Path,sys.argv[2:4]))
    else:plot(*map(Path,sys.argv[1:4]))
