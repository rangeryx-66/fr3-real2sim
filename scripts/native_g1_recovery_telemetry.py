"""Passive native-G1 interface/contact feedback. Never a manipulation gate."""
import json,time
from pathlib import Path
import numpy as np


def available(fn):
    try:
        value=fn()
        if hasattr(value,'tolist'):value=value.tolist()
        return dict(available=value is not None,value=value)
    except Exception as error:return dict(available=False,reason=repr(error))


def interface_definition(g):
    from pxr import UsdPhysics,PhysxSchema
    names=g['names'];drives=[]
    for p in g['stage'].Traverse():
        if not str(p.GetPath()).startswith('/World/Piper') or p.GetName() not in g['controller'].geometry.c['gripper_joint_names']:continue
        if not p.IsA(UsdPhysics.Joint):continue
        d=UsdPhysics.DriveAPI.Get(p,'angular')
        attrs={k:available(lambda key=k: p.GetAttribute(key).Get()) for k in ['drive:angular:physics:targetPosition','drive:angular:physics:targetVelocity','drive:angular:physics:stiffness','drive:angular:physics:damping','drive:angular:physics:maxForce','drive:angular:physics:type']}
        mimic=PhysxSchema.PhysxMimicJointAPI.Get(p,'rotX')
        coupling=available(lambda:dict(reference=[str(x) for x in mimic.GetReferenceJointRel().GetTargets()],gearing=mimic.GetGearingAttr().Get(),offset=mimic.GetOffsetAttr().Get()))
        drives.append(dict(name=p.GetName(),path=str(p.GetPath()),USD_attributes=attrs,mimic=coupling))
    return dict(joint_order=names,master_index=names.index('drive_joint'),
        actuator='unchanged finite implicit position drive; one motor/five mimic constraints',
        runtime_gains=available(lambda:[np.asarray(x).tolist() for x in g['controller'].raw.get_gains()]),
        runtime_max_efforts=available(lambda:g['robot']._articulation_view.get_max_efforts()),
        USD_drives=drives,USD_angular_target_units='degrees; runtime articulation positions/velocities are rad, rad/s',
        independent_motor_drive_force=dict(available=False,reason='Existing public articulation getters expose explicit command or projected joint reaction, not isolated implicit-drive output; neither is relabeled as motor force.'),
        constraint_impulses=dict(available=False,reason='Per-mimic solver impulses not exposed by the current archived/public interface; authored constraint definitions are recorded.'),
        measured_joint_effort_semantics='solver joint reaction projected along DOF; not isolated drive torque',
        explicit_effort_semantics='explicit external effort command only; zero does not mean position drive is off',
        all_optional_fields_passive=True)


def install(g):
    old=g['observe_step']
    def observe_step(state,contacts):
        old(state,contacts)
        try:
            action=g['controller'].raw.get_applied_action()
            targets={k:available(lambda key=k:getattr(action,key)) for k in ['joint_positions','joint_velocities','joint_efforts','joint_indices']}
            body={}
            for name in ['left_finger','right_finger']:
                view=g['actual_views'].get(name)
                body[name]=available(lambda v=view:g['matrix'](*[z[0] for z in v.get_world_poses()]))
            state['native_g1_interface']=dict(applied_action_readback=targets,
                runtime_max_efforts=available(lambda:g['robot']._articulation_view.get_max_efforts()),
                actual_finger_body_world_poses=body,reported_native_feedback=g['controller'].feedback(),
                independent_drive_force_available=False,constraint_impulses_available=False,
                recovery_state=g.get('recovery_state'),attempt_index=g.get('attempt_index',0))
        except Exception as error:g['diagnostic_issue']('passive_native_G1_interface',error)
    g['observe_step']=observe_step
    try:(g['a'].output/'NATIVE_G1_INTERFACE_DEFINITION.json').write_text(json.dumps(interface_definition(g),indent=2))
    except Exception as error:g['diagnostic_issue']('native_G1_interface_definition',error)


def snapshot(g,folder,label):
    """Quick image acquisition only; expensive interpretation waits for unload."""
    try:
        import cv2
        folder.mkdir(parents=True,exist_ok=True)
        for camera,name in [(g['wrist_sensor'],'wrist'),(g['near'],'diagnostic')]:
            rgb=np.asarray(camera.get_rgba())[...,:3]
            cv2.imwrite(str(folder/(label+'_'+name+'.png')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
    except Exception as error:g['diagnostic_issue']('contact_snapshot',error)


def inspect_result(g,attempt):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder=Path(attempt['folder']);lo=attempt['first_row'];hi=attempt['last_row'];rows=g['rows'][lo:hi]
    if not rows:return dict(available=False,reason='No completed physics samples for attempt')
    acquired=next((e for e in g['telemetry_events'] if e.get('event')=='GRASP_VERIFIED' and e['t']>=attempt['start_sim_s']),None)
    zero=acquired['t'] if acquired else rows[0]['t'];t=np.array([r['t']-zero for r in rows]);keep=t>=(-1 if acquired else -np.inf)
    selected=[r for r,k in zip(rows,keep) if k];tv=t[keep]
    raw=np.array([[r['telemetry']['pad_load_raw_n'][n] for n in g['FINGERS']] for r in selected]);filtered=np.array([r['telemetry']['pad_load_filtered_n'] for r in selected])
    fig,axs=plt.subplots(3,2,figsize=(13,10));colors=['#2769a8','#d15a24']
    for i in range(2):
        axs[0,0].plot(tv,raw[:,i],lw=.6,color=colors[i],label=['left raw','right raw'][i]);axs[0,0].plot(tv,filtered[:,i],ls='--',lw=.9,color=colors[i])
    axs[0,0].set_ylabel('Native normal load N; dashed=existing filter')
    axs[0,1].plot(tv,[r['aperture_m']*1000 for r in selected],label='master-derived measured');axs[0,1].plot(tv,[r['telemetry']['command_aperture_m']*1000 for r in selected],label='command');axs[0,1].set_ylabel('Aperture mm')
    axs[1,0].plot(tv,[r['door_angle_deg']-(acquired['angle_deg'] if acquired else rows[0]['door_angle_deg']) for r in selected],label='actual hinge progress');axs[1,0].set_ylabel('Known-state opening gain deg')
    rel=[np.linalg.inv(r['T_moving_link'])@np.asarray(r['T_tcp']) for r in selected];origin=rel[max(0,int(np.argmin(abs(tv))))]
    axs[1,1].plot(tv,[np.linalg.norm(r[:3,3]-origin[:3,3])*1000 for r in rel],label='TCP/body relative change');axs[1,1].set_ylabel('Relative translation mm (inferred slip proxy)')
    last=selected[-1];inv=np.linalg.inv(last['T_tcp']);cs=[c for c in last['contacts'] if c.get('allowed_pad_target')]
    for i,n in enumerate(g['FINGERS']):
        pts=[(inv@np.r_[c['contact_point_world_m'],1])[:3]*1000 for c in cs if c['finger']==n]
        if pts:
            p=np.asarray(pts);axs[2,0].scatter(p[:,0],p[:,1],color=colors[i],label=n+' reported points')
        axs[2,1].step(tv,(raw[:,i]>0).astype(float)-i*1.3,where='post',color=colors[i],label=n+' positive normal (offset)')
    axs[2,0].set(xlabel='TCP x mm',ylabel='TCP y mm',title='Native contact points, not pressure areas; no GT mesh')
    for ax in axs.flat:ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Contact feedback / measured normals and aperture / known-state progress\nNo positive normal report alone proves neither separation nor failure; no complete wrench is available')
    fig.tight_layout();fig.savefig(folder/'CONTACT_RESULT.png',dpi=140);plt.close(fig)
    packet=dict(measured=['wrist RGB','native normal impulses/points','master position','all coupled q/dq','arm TCP','applied-action target readback'],
        inferred=['FK pad placement','TCP/body relative motion as slip proxy','master-angle aperture assumes nominal coupling'],
        unavailable=['isolated implicit-drive motor torque','mimic solver impulses','friction anchor forces','pressure distribution'],
        known_state_dependency=['hinge progress','moving-body pose'],result={k:v for k,v in attempt.items() if k!='inspection'},image_board=str(folder/'CONTACT_RESULT.png'),
        wrist_image=str(folder/'stopped_wrist.png'),diagnostic_image=str(folder/'stopped_diagnostic.png'),
        inspected_only_after_unloaded_safe_exit=True)
    (folder/'CONTACT_INSPECTION.json').write_text(json.dumps(packet,indent=2));return packet
