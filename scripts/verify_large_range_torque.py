"""Archived exact handover replay + FK/Jacobian/frame/model checks; no Isaac."""
import argparse,json,pickle
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pinocchio as pin
from cross_robot_adapter import RobotPinkIK
from interaction_reference_handover import handover,seed_pose,contact_policy_source
from torque_components_v1 import fit,features


def verify(episode,pink):
    rows=pickle.load((episode/'hybrid_identification/00/measurements.pkl').open('rb'))
    c=json.loads((episode/'sensor_assembly/SENSOR_CONFIG.json').read_text());c['urdf']=str(episode/'sensor_assembly/sim_urdf.urdf');solver=RobotPinkIK(c)
    meta=json.loads((episode/'sensor_metadata.json').read_text());arm=[meta['joint_names'].index(n) for n in c['arm_joint_names']]
    i=next(i for i in range(1,len(rows)) if rows[i-1]['phase']=='LOCAL_CENTERING' and rows[i]['phase']=='TRANSMISSION_PROBE');r=rows[i-1];n=rows[i]
    cmd=np.array(r['command_q_arm']);measured=np.array(r['q'])[arm];base=r['base'];dt=1/240;oldref=np.array(n['planned_T_tcp']);bias=cmd-measured
    command_pose=seed_pose(solver,base,cmd);oldq=solver.velocity_step(oldref,base,measured,dt)+bias
    oldv=np.linalg.norm(seed_pose(solver,base,oldq)[:3,3]-command_pose[:3,3])/dt*1000
    qtarget=np.array(r['q']);qtarget[arm]=cmd
    g=dict(robot=SimpleNamespace(get_joint_positions=lambda:np.array(r['q'])),arm=arm,qtarget=qtarget,ref=oldref,solver=solver,base=base,
        actual=lambda:r['door_angle_deg']*np.pi/180,tick=0,dt=dt,telemetry_events=[])
    handover(g,'archived_center_to_probe');new=[]
    for direction in [1.,-1.,0.]:
        target=g['ref'].copy();target[0,3]+=direction*.002*dt**2
        q=solver.velocity_step(target,base,g['seed'],dt)+g['bias']
        new.append(dict(direction=direction,command_fk_mm_s=float(np.linalg.norm(seed_pose(solver,base,q)[:3,3]-command_pose[:3,3])/dt*1000)))
    # Numerical local Jacobian check on the actual archived branch.
    data=solver.robot.createData();frame=solver.robot.getFrameId('tcp_link');J=pin.computeFrameJacobian(solver.robot,data,measured,frame,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    eps=1e-7;R=np.eye(3);from articulated_demo.kinematics import transform
    R=transform(base[:3],[0,0,np.deg2rad(base[3])])[:3,:3];numeric=np.zeros((3,7))
    for j in range(7):
        dq=np.eye(7)[j]*eps;numeric[:,j]=(seed_pose(solver,base,measured+dq)[:3,3]-seed_pose(solver,base,measured-dq)[:3,3])/(2*eps)
    jacerror=float(np.max(abs(R@J[:3]-numeric)))
    compile(contact_policy_source(),'versioned_policy','exec')
    # Independent synthetic trajectory confirms component convention/recovery,
    # not sensor validity or physical identification performance.
    t=np.linspace(0,20,1200);q=.2+.14*np.sin(1.3*t)+.04*np.sin(3.1*t);v=.14*1.3*np.cos(1.3*t)+.04*3.1*np.cos(3.1*t);a=-.14*1.3**2*np.sin(1.3*t)-.04*3.1**2*np.sin(3.1*t)
    truth=np.array([.3,.2,.025,.15,.35]);samples=[dict(q=float(z),qd=float(w),qdd=float(x),direction=float(np.sign(w)),tau=0.) for z,w,x in zip(q,v,a)]
    y=features(samples)@truth
    for s,value in zip(samples,y):s['tau']=float(value)
    model=fit(samples);error=np.array(model['parameters'])-truth
    # This assert is in an OFFLINE verification only, never execution safety.
    np.testing.assert_allclose(oldv,35.1942511338,rtol=1e-5)
    assert max(x['command_fk_mm_s'] for x in new)<.01 and jacerror<1e-7
    np.testing.assert_allclose(error,0,atol=1e-4)
    return dict(physics_actors=0,archived_t=r['t'],old_first_command_fk_mm_s=float(oldv),new_first_step=new,
        joint_command_preserved=bool(np.array_equal(qtarget[arm],cmd)),grasp_reference_not_reset=True,
        FK_Jacobian_max_abs_error=jacerror,synthetic_only_component_error=error.tolist(),synthetic_regressor_rank=model['algebraic_rank'],
        controller_gains_or_protections_changed=False,not_physical_validation=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--pink',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    r=verify(a.episode,a.pink);a.output.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
