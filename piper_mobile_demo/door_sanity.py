"""ROS-independent copy of the existing no-robot door sanity diagnostic."""
import json,math
import numpy as np

def run_door_without_robot(output,world,stage,door,chain,asset_path,moving_path,door_path,rotation,xyz,handle,mass_audit,manifest,dt,fixture_audit=None):
    """Same environment/geometry, but genuinely no robot in this test scene."""
    from isaacsim.core.prims import RigidPrim,SingleXFormPrim
    from pxr import UsdPhysics
    from scipy.spatial.transform import Rotation
    paths=[str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()).startswith(asset_path)
        and not str(p.GetPath()).startswith(moving_path) and p.HasAPI(UsdPhysics.RigidBodyAPI)] + ['/World/table','/World/cabinet_fixture']
    contacts=world.scene.add(RigidPrim(prim_paths_expr=door_path,name='no_robot_door_contacts',
        contact_filter_prim_paths_expr=paths,track_contact_forces=True,prepare_contact_sensors=True,max_contact_count=256))
    link=SingleXFormPrim(moving_path);world.reset()
    root=np.eye(4);root[:3,:3]=rotation;root[:3,3]=xyz
    joint=chain.joints[manifest['moving_link']];hinge=root@chain.root_to_link(joint.parent,{})@joint.origin
    axis=hinge[:3,:3]@joint.axis;radius=float(np.linalg.norm(np.cross(axis,handle-hinge[:3,3])))
    result={'kind':'passive door torque sweep, robot not loaded','hinge_axis_world':axis.tolist(),
        'handle_radius_m':radius,'joint_limits_rad':manifest['source_joint_limits_rad'],
        'mass_audit':mass_audit,'asset_collision_geometry_unchanged':True,
        'fixture_audit':fixture_audit,'trials':[]}
    result['joint_usd_attributes']=[{str(at.GetName()):str(at.Get()) for at in p.GetAttributes()}
        for p in stage.Traverse() if str(p.GetPath()).startswith(asset_path) and p.IsA(UsdPhysics.RevoluteJoint)]
    output.parent.mkdir(parents=True,exist_ok=True)
    for torque in (0.,.001,.0025,.005,.01,.1,.25,.5,.75,1.,2.,-.1):
        door.set_joint_positions(np.array([0.]));door.set_joint_velocities(np.array([0.]))
        for _ in range(24):door.set_joint_efforts(np.array([0.]));world.step(render=False)
        samples=[];duration=3. if torque==2. else 1.
        for i in range(round(duration/dt)):
            door.set_joint_efforts(np.array([torque]));world.step(render=False)
            if i%8==0:
                q=float(door.get_joint_positions()[0]);p,quat=link.get_world_pose()
                expected=root@chain.root_to_link(manifest['moving_link'],{manifest['joint_name']:q})
                samples.append({'t_s':(i+1)*dt,'angle_deg':math.degrees(q),
                    'velocity_rad_s':float(door.get_joint_velocities()[0]),
                    'actuation_torque_nm':float(door._articulation_view._physics_view.get_dof_actuation_forces()[0,0]),
                    'urdf_isaac_position_error_m':float(np.linalg.norm(p-expected[:3,3])),
                    'urdf_isaac_rotation_error_rad':float((Rotation.from_matrix(expected[:3,:3]).inv()*Rotation.from_quat(np.roll(quat,-1))).magnitude()),
                    'contact_by_body_n':dict(zip(paths,np.linalg.norm(np.asarray(contacts.get_contact_force_matrix(dt=dt)).reshape(-1,3),axis=1).tolist()))})
        door.set_joint_efforts(np.array([0.]))
        coast=[]
        for i in range(120):
            world.step(render=False)
            if i%8==0:coast.append({'t_s':i*dt,'velocity_rad_s':float(door.get_joint_velocities()[0])})
        result['trials'].append({'torque_nm':torque,'equivalent_tangential_force_n':torque/radius,
            'max_angle_deg':max(s['angle_deg'] for s in samples),'end_angle_deg':samples[-1]['angle_deg'],
            'collision_peak_by_body_n':{name:max(s['contact_by_body_n'][name] for s in samples) for name in paths},
            'samples':samples,'zero_torque_coast':coast})
        output.write_text(json.dumps(result,indent=2))
    first=min((r['torque_nm'] for r in result['trials'] if r['torque_nm']>0 and r['max_angle_deg']>.5),default=None)
    result['opening_torque_upper_bound_nm']=first
    result['opening_tangential_force_upper_bound_n']=first/radius if first else None
    result['threshold']='more than 0.5 degree within test duration; passive hinge, measured contacts included'
    output.write_text(json.dumps(result,indent=2))
