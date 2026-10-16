"""Commissioning only. Raw native-rate records remain immutable and private."""
import argparse,gzip,json,pickle
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
from xarm_wrist_ft import wrench_at, tool_inertial_world


def analyze(root):
    root=Path(root);idx=json.loads((root/'physics_chunks/index.json').read_text());rows=[]
    for c in idx['chunks']:
        with gzip.open(root/'physics_chunks'/c['file'],'rb') as f:rows.extend(pickle.load(f))
    if not rows:raise ValueError('No physics sensor samples')
    t=np.array([r['t'] for r in rows]);dt=float(np.median(np.diff(t)))
    raw=np.array([r['raw_wrench_sensor'] for r in rows]);T=np.array([r['T_world_sensor'] for r in rows]);R=T[:,:3,:3]
    gravity=np.array([r['tool_gravity_wrench_world'] for r in rows])
    world=np.concatenate([np.einsum('nij,nj->ni',R,raw[:,:3]),np.einsum('nij,nj->ni',R,raw[:,3:])],axis=1)
    phases=np.array([r['phase'] for r in rows]);phase_masks={}
    for phase in dict.fromkeys(phases):
        m=phases==phase;start=t[m][0];phase_masks[phase]=m & (t>=start+.5)
    reference=phase_masks['STATIC_ORIENTATION_0']
    g_sensor=np.concatenate([np.einsum('nji,nj->ni',R,gravity[:,:3]),np.einsum('nji,nj->ni',R,gravity[:,3:])],axis=1)
    # Fit only initial unloaded static bias; other orientations are validation.
    # Determine action-reaction convention using separate known loads below.
    errors={}
    for sign in [1,-1]:
        bias=np.mean(raw[reference]+sign*g_sensor[reference],axis=0)
        compensated=raw-bias+sign*g_sensor
        errors[str(sign)]=float(np.sqrt(np.mean(compensated[phase_masks['STATIC_ORIENTATION_1'],:3]**2)))
    # Supported-by-gravity candidate is explicitly reported; known loads must
    # independently agree before anyone uses it for object identification.
    sign=min([1,-1],key=lambda s:errors[str(s)])
    bias=np.mean(raw[reference]+sign*g_sensor[reference],axis=0)
    raw_unbiased=raw-bias
    compensated_world=np.concatenate([np.einsum('nij,nj->ni',R,raw_unbiased[:,:3]),np.einsum('nij,nj->ni',R,raw_unbiased[:,3:])],axis=1)+sign*gravity
    # Measured COM trajectory derivatives, not the discrepant native dq field.
    # Offline centered .504 s SG window. The .104 s comparison did not remove
    # the dynamic residual; its mechanism remains unresolved. Keep raw data.
    inertial=np.zeros_like(world);names=[b['name'] for b in rows[0]['body_states']]
    states=[[] for _ in rows]
    for j,n in enumerate(names):
        b=[r['body_states'][j] for r in rows];pos=np.array([x['com_world_m'] for x in b])
        rot=np.array([x['T_world_body'] for x in b])[:,:3,:3]
        accel=savgol_filter(pos,121,3,deriv=2,delta=dt,axis=0)
        omega=np.vstack([np.zeros(3),Rotation.from_matrix(rot[1:]@rot[:-1].transpose(0,2,1)).as_rotvec()/dt])
        omega=savgol_filter(omega,121,3,axis=0);alpha=savgol_filter(omega,121,3,deriv=1,delta=dt,axis=0)
        for i,x in enumerate(b):states[i].append(dict(**x,com_acceleration_world_m_s2=accel[i],omega_world_rad_s=omega[i],alpha_world_rad_s2=alpha[i]))
    for i in range(len(rows)):inertial[i]=tool_inertial_world(states[i],T[i,:3,3])
    dynamic_compensated=compensated_world-sign*inertial
    summary=[]
    for phase,m in phase_masks.items():
        if not np.any(m):continue
        indices=np.flatnonzero(m)
        f=np.mean([rows[i]['independently_applied_force_world_n'] for i in indices],axis=0)
        moment=np.mean([np.array(rows[i]['independently_applied_torque_world_nm'])+np.cross(np.array(rows[i]['independently_applied_position_world_m'])-T[i,:3,3],rows[i]['independently_applied_force_world_n']) for i in indices],axis=0)
        expected=-sign*np.r_[f,moment]
        residual=dynamic_compensated[m]-expected
        summary.append(dict(phase=phase,samples=int(m.sum()),duration_s=float((phases==phase).sum()*dt),
            mean_raw_sensor=raw[m].mean(axis=0).tolist(),mean_gravity_only_compensated_world=compensated_world[m].mean(axis=0).tolist(),
            mean_inertia_compensated_world=dynamic_compensated[m].mean(axis=0).tolist(),expected_calibration_reaction_world=expected.tolist(),
            force_residual_rms_n=float(np.sqrt(np.mean(np.sum(residual[:,:3]**2,axis=1)))),torque_residual_rms_nm=float(np.sqrt(np.mean(np.sum(residual[:,3:]**2,axis=1)))),
            force_residual_max_n=float(np.linalg.norm(residual[:,:3],axis=1).max()),torque_residual_max_nm=float(np.linalg.norm(residual[:,3:],axis=1).max())))
    report=dict(records=len(rows),native_rate_hz=1/dt,valid_samples=int(np.isfinite(raw).all(axis=1).sum()),
        bias_sensor=bias.tolist(),bias_fit='initial unloaded orientation only; other orientations held out',
        gravity_sign_hypothesis=sign,gravity_sign_hypothesis_errors_n=errors,phases=summary,
        inertia_compensation='per-body measured COM/rotation derivatives; centered SG121/order3 at240Hz OFFLINE; delay/edge validity required online',
        maximum_fk_position_error_m=max(r['fk_position_error_m'] for r in rows),maximum_fk_rotation_error_rad=max(r['fk_rotation_error_rad'] for r in rows),
        contacts=sum(len(r['contacts']) for r in rows),actuator_output_not_claimed=True,
        commissioning_pass='REQUIRES review of independent known force/torque response and dynamic residuals; no automatic pass')
    (root/'SENSOR_ANALYSIS.json').write_text(json.dumps(report,indent=2))
    np.savez_compressed(root/'processed_wrench.npz',t=t,raw_sensor=raw,gravity_only_compensated_world=compensated_world,
        gravity_and_inertia_compensated_world=dynamic_compensated,tool_gravity_world=gravity,tool_inertial_world=inertial,phase=phases,T_world_sensor=T,bias_sensor=bias)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(14,9),sharex=True)
    for j,ax in enumerate(axes.flat):
        ax.plot(t,world[:,j],lw=.5,label='raw world at sensor')
        ax.plot(t,compensated_world[:,j],lw=.5,label='bias+gravity compensated')
        ax.plot(t,dynamic_compensated[:,j],lw=.5,label='+tool inertia compensated')
        ax.set_ylabel(['Fx N','Fy N','Fz N','Tx Nm','Ty Nm','Tz Nm'][j]);ax.grid(alpha=.3)
    axes[0,0].legend(fontsize=7);axes[-1,0].set_xlabel('simulation seconds');axes[-1,1].set_xlabel('simulation seconds')
    fig.tight_layout();fig.savefig(root/'SENSOR_TRACES.png',dpi=160);plt.close(fig)
    print(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root');analyze(p.parse_args().root)
