"""Independent passive-hinge, ideal rigid-grasp and locked-handle load tests.

No object-id heuristics, geometry/friction/closing-force changes or base search.
"""
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .backend import matrix,Failure
from .preflight import probe_ik,trajectory_end,moved_door_boxes
from .redundant_path import max_margin_ik,values,with_q,margins
from r1a7_backend import JOINTS
import r1a7_plant as plant
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def checked_command(data):
    r=plant.command(data,timeout=90)
    if not r.get('ok'):raise RuntimeError(str(r))
    return r


def run_door_without_robot(output,world,stage,door,chain,asset_path,moving_path,door_path,rotation,xyz,handle,mass_audit,manifest,dt):
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
        'mass_audit':mass_audit,'collision_geometry_unchanged':True,'trials':[]}
    result['joint_usd_attributes']=[{str(at.GetName()):str(at.Get()) for at in p.GetAttributes()}
        for p in stage.Traverse() if str(p.GetPath()).startswith(asset_path) and p.IsA(UsdPhysics.RevoluteJoint)]
    output.parent.mkdir(parents=True,exist_ok=True)
    for torque in (0.,.01,.1,.25,.5,.75,1.,2.,-.1):
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
    result['threshold']='more than 0.5 degree within test duration; includes support collision friction'
    output.write_text(json.dumps(result,indent=2))


def run_tests(node,grasp,output,report,save):
    diagnostic={'kind':'three isolated tests, fixed base and unchanged contact parameters'}
    report['isolation']=diagnostic
    def checkpoint():
        (output/'isolation.json').write_text(json.dumps(diagnostic,indent=2,default=str));save()
    audit=checked_command({'op':'diagnostic_audit'});diagnostic['asset_audit']=audit
    checked_command({'op':'resume'})
    sanity={'trials':[]};diagnostic['door_sanity']=sanity
    for torque in (0.,.0001,.0005,.001,.005,.01,.05,.2,.5,1.,2.,-.01):
        checked_command({'op':'diagnostic_reset_door'});plant.settle(.1)
        begin=plant.state()['t'];checked_command({'op':'diagnostic_door_torque','torque_nm':torque})
        plant.settle(1.)
        checked_command({'op':'diagnostic_door_torque','torque_nm':0.})
        state=plant.state();samples=[h for h in state['history'] if h['t']>=begin]
        angles=np.array([h['joint_q'] for h in samples]);vel=np.array([h['joint_velocity_rad_s'] for h in samples]);times=np.array([h['t'] for h in samples])
        fit=slice(1,min(7,len(samples)))
        acceleration=float(np.polyfit(times[fit]-times[0],vel[fit],1)[0]) if len(samples)>7 else None
        row={'torque_nm':torque,'equivalent_handle_tangential_force_n':torque/audit['handle_radius_m'],
            'end_angle_deg':float(np.degrees(state['joint_q'])),'max_angle_deg':float(np.degrees(angles.max())),
            'min_angle_deg':float(np.degrees(angles.min())),
            'initial_acceleration_rad_s2':acceleration,
            'inferred_effective_inertia_kg_m2':torque/acceleration if torque>0 and acceleration and acceleration>0 else None,
            'max_collision_force_n':max(h.get('door_net_collision_force_n',0.) for h in samples),
            'max_robot_finger_contact_n':max(max(h['forces']) for h in samples),
            'actuation_torque_readback_nm':[min(h['actual_dof_actuation_torque_nm'] for h in samples),max(h['actual_dof_actuation_torque_nm'] for h in samples)],
            'collision_peak_by_body_n':{name:max(h['door_collision_by_body_n'][name] for h in samples) for name in samples[-1]['door_collision_by_body_n']},
            'samples':samples}
        sanity['trials'].append(row);checkpoint()
    opening=[r for r in sanity['trials'] if r['torque_nm']>0 and r['max_angle_deg']>.5]
    sanity['observed_opening_torque_upper_bound_nm']=min((r['torque_nm'] for r in opening),default=None)
    sanity['observed_opening_force_upper_bound_n']=sanity['observed_opening_torque_upper_bound_nm']/audit['handle_radius_m'] if opening else None
    sanity['threshold_definition']='opens >0.5 degrees within 1 s; upper bound, not calibrated Coulomb friction'
    checked_command({'op':'diagnostic_reset_door'});plant.settle(.2);checked_command({'op':'pause'})
    node.scene();home=node.measured()
    _,gs=probe_ik(node,grasp,home,random_seeds=8,timeout_s=.25)
    if gs is None:raise Failure('NO_IK','isolated grasp pose unavailable')
    pre=grasp.copy();pre[:3,3]-=.08*grasp[:3,2]
    _,ps=probe_ik(node,pre,gs,random_seeds=8,timeout_s=.25)
    if ps is None:raise Failure('NO_IK','isolated pregrasp unavailable')
    approach=node.cartesian(ps,grasp);preplan=node.plan(home,ps)
    start=trajectory_end(ps,approach)
    diagnostic['comfortable_grasp_margin']=margins(node,values(start))
    if diagnostic['comfortable_grasp_margin']['all_joint_min_rad']<=.1:
        raise Failure('LOW_JOINT_MARGIN','isolated load test requires >0.1 rad at starting pose')
    # Ideal rigid relation is mathematical, not a fake physical fixed joint.
    initial=plant.state();q0=float(initial['joint_q']);moving=initial['moving_link'];joint=initial['joint_name']
    T0=matrix(initial['moving_pose']['position'],initial['moving_pose']['quaternion_wxyz'])
    root0=node.chain.root_to_link(moving,{joint:q0});boxes=initial['collision_boxes']
    ideal={'rows':[],'collision_check':'all MoveIt self/table/support/static/door constraints; no planner ACM relaxation',
        'tracking':'continuous previous IK seed, 0.5 degree increments, dense joint interpolation',
        'max_feasible_door_angle_with_ideal_grasp':float(math.degrees(q0))}
    diagnostic['ideal_grasp']=ideal;current=start;previous=q0
    upper=min(90.,math.degrees(initial['joint_limits']['upper']))
    for degrees in np.arange(.5,upper+.001,.5):
        angle=math.radians(float(degrees));root=node.chain.root_to_link(moving,{joint:angle});pose=T0@np.linalg.inv(root0)@root
        target=node.chain.target_tcp(moving_link=moving,joint_name=joint,q_now=q0,q_target=angle,T_world_moving_now=T0,T_world_tcp_now=grasp)
        node.scene(collision_boxes=moved_door_boxes(boxes,T0,pose),moving_pose_override=pose)
        solutions,trials=max_margin_ik(node,target,current)
        row={'door_angle_deg':float(degrees),'ik_trials':trials};ideal['rows'].append(row)
        J=[];old=values(current);eps=1e-5;fk=node.robot_chain.forward(dict(zip(JOINTS,old)))
        for j in range(7):
            t=node.robot_chain.forward(dict(zip(JOINTS,old+np.eye(7)[j]*eps)))
            J.append(np.r_[(t[:3,3]-fk[:3,3])/eps,.1*(Rotation.from_matrix(fk[:3,:3]).inv()*Rotation.from_matrix(t[:3,:3])).as_rotvec()/eps])
        row['weighted_jacobian_singular_values']=np.linalg.svd(np.array(J).T,compute_uv=False).tolist()
        accepted=None
        for solution,detail in solutions:
            try:
                endpoint=values(solution);count=max(3,int(np.ceil(np.max(np.abs(endpoint-old))/.01)))
                for alpha in np.linspace(0,1,count+1):
                    qr=old+alpha*(endpoint-old);a=previous+alpha*(angle-previous)
                    m=margins(node,qr)
                    if m['all_joint_min_rad']<=.05:raise Failure('LOW_JOINT_MARGIN','one of seven joints below 0.05')
                    r=node.chain.root_to_link(moving,{joint:a});p=T0@np.linalg.inv(root0)@r
                    node.scene(collision_boxes=moved_door_boxes(boxes,T0,p),moving_pose_override=p)
                    node.validate(with_q(current,qr))
                    expected=node.chain.target_tcp(moving_link=moving,joint_name=joint,q_now=q0,q_target=a,T_world_moving_now=T0,T_world_tcp_now=grasp)
                    actual=node.robot_chain.forward(dict(zip(JOINTS,qr)))
                    if np.linalg.norm(expected[:3,3]-actual[:3,3])>.002:raise Failure('OFF_ARC','interpolated IK path left handle arc')
                accepted=solution;row.update(status='VALID',q=endpoint.tolist(),margin=detail);break
            except Failure as error:row.setdefault('edge_failures',[]).append({'status':error.category,'detail':str(error)})
        if accepted is None:
            row['status']='COLLISION' if any(t['status']=='COLLISION' for t in trials) else ('NO_SAFE_CONTINUOUS_IK' if not solutions else 'INTERPOLATION_FAILURE')
            ideal['failed_angle_deg']=float(degrees);ideal['failure_reason']=row['status'];checkpoint();break
        current=accepted;previous=angle;ideal['max_feasible_door_angle_with_ideal_grasp']=float(degrees);checkpoint()
    ideal['scope']='maximum verified on this continuous local IK branch; not a global proof over all grasps/branches'
    node.scene();checked_command({'op':'resume'})
    checked_command({'op':'diagnostic_lock_door','locked':True})
    load={'rows':[],'method':'locked door; displacement-controlled tangential pull; force from finger-door contact vectors',
          'max_stable_tangential_force':None,'friction_and_gripper_command_unchanged':True}
    diagnostic['isolated_grasp']=load;recording=False
    try:
        checked_command({'op':'video_start','path':str(output/'isolated_grasp.mp4')});recording=True
        node.stage='ISOLATED_PREGRASP';node.execute(preplan,'NO_PLAN')
        node.stage='ISOLATED_APPROACH';node.execute(approach,'NO_PLAN')
        result=node.gripper(0.);plant.settle(.5)
        closed=plant.state();load['close_forces_n']=closed['forces'];load['gripper_stalled']=bool(result.stalled)
        if not all(f>.2 for f in closed['forces']):
            load['status']='BAD_CONTACT_AT_ZERO_LOAD';checkpoint();return
        reference=np.array(closed['tcp']);hinge=np.array(audit['hinge_origin_world']);axis=np.array(audit['hinge_axis_world'])
        reference_pose=matrix(closed['tcp'],closed['tcp_quat'])
        tangent=np.cross(axis,reference-hinge);tangent/=np.linalg.norm(tangent)
        load['tangent_world']=tangent.tolist();load['locked_angle_deg']=math.degrees(closed['joint_q'])
        for displacement in np.arange(0,.01501,.00025):
            if displacement:
                goal=reference_pose.copy();goal[:3,3]+=displacement*tangent
                node.scene();measured=node.measured()
                solutions,trials=max_margin_ik(node,goal,measured)
                if not solutions:
                    load['load_ik_failure_trials']=trials
                    raise Failure('LOAD_NO_SAFE_IK','no collision-free IK for small isolated loading displacement')
                chosen=min(solutions,key=lambda s:np.linalg.norm(values(s[0])-values(measured)))[0]
                old=values(measured);new=values(chosen)
                trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=list(JOINTS)
                for alpha in np.linspace(0,1,11):
                    qr=old+alpha*(new-old);node.validate(with_q(measured,qr))
                    if margins(node,qr)['all_joint_min_rad']<=.05:raise Failure('LOW_JOINT_MARGIN','loading interpolation outside margin')
                    point=JointTrajectoryPoint();point.positions=qr.tolist();point.velocities=[0.]*7
                    point.time_from_start=Duration(sec=int(2*alpha),nanosec=int((2*alpha%1)*1e9))
                    trajectory.joint_trajectory.points.append(point)
                for pt in trajectory.joint_trajectory.points:
                    qdict=dict(zip(trajectory.joint_trajectory.joint_names,pt.positions))
                    if margins(node,np.array([qdict[n] for n in JOINTS]))['all_joint_min_rad']<=.05:
                        raise Failure('LOW_JOINT_MARGIN','load test left safe comfortable workspace')
                node.stage='ISOLATED_TANGENTIAL_PULL';node.execute(trajectory,'NO_PLAN')
            begin=plant.state()['t'];plant.settle(.5);state=plant.state()
            samples=[h for h in state['history'] if h['t']>=begin]
            contact=np.array([sum(np.dot(v,tangent) for v in h['force_vectors_world']) for h in samples])
            slipped=float(np.linalg.norm(np.array(state['tcp'])-reference))
            lost=sum(not all(f>.2 for f in h['forces']) for h in samples)/max(1,len(samples))
            row={'commanded_tangential_displacement_m':float(displacement),'relative_slip_m':slipped,
                'measured_tangential_force_n':float(abs(np.mean(contact))),
                'signed_tangential_force_n':float(np.mean(contact)),
                'force_peak_n':float(np.max(np.abs(contact))),
                'single_side_contact_loss_fraction':lost,'forces_n':state['forces'],
                'actual_door_angle_deg':math.degrees(state['joint_q']),
                'all_joint_margin_rad':margins(node,values(node.measured()))['all_joint_min_rad']}
            load['rows'].append(row)
            if abs(state['joint_q']-closed['joint_q'])>math.radians(.1):row['status']='DOOR_LOCK_FAILURE'
            elif lost>.2:row['status']='SINGLE_SIDE_CONTACT_LOSS'
            elif slipped>.005:row['status']='GRASP_SLIP_FAILURE'
            else:
                row['status']='STABLE'
                if displacement>0:
                    load['max_stable_tangential_force']=max(load['max_stable_tangential_force'] or 0.,row['measured_tangential_force_n'])
                else:load['zero_displacement_tangential_preload_n']=row['measured_tangential_force_n']
            checkpoint()
            if row['status']!='STABLE':load['status']=row['status'];break
        else:load['status']='NO_FAILURE_WITHIN_TEST_RANGE'
    except Failure as error:load['status']=error.category;load['detail']=str(error)
    finally:
        if recording:load['video_result']=checked_command({'op':'video_stop'})
        checked_command({'op':'diagnostic_lock_door','locked':False});checkpoint()
    report['status']='ISOLATION_DIAGNOSTICS_COMPLETE';checkpoint()
