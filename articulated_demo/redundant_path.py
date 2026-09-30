"""Exact URDF IK with continuous max-margin redundancy, checked by MoveIt."""
import math
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity,GetPositionFK
from r1a7_backend import JOINTS, FOCUS, GROUP, BASE, TCP, Failure


def values(state):
    q=dict(zip(state.joint_state.name,state.joint_state.position))
    return np.asarray([q[name] for name in JOINTS])


def with_q(seed,q):
    state=RobotState();state.joint_state.name=list(seed.joint_state.name)
    all_values=dict(zip(seed.joint_state.name,seed.joint_state.position))
    all_values.update(zip(JOINTS,q))
    state.joint_state.position=[float(all_values[n]) for n in state.joint_state.name]
    return state


def margins(node,q):
    bounds=np.asarray([node.limits[n] for n in JOINTS])
    margin=np.minimum(q-bounds[:,0],bounds[:,1]-q)
    return {'all_joint_min_rad':float(margin.min()),
            'focus_min_rad':float(margin[4:].min()),
            'joint_margins_rad':{name:float(margin[i]) for i,name in enumerate(JOINTS)},
            **{n:float(margin[JOINTS.index(n)]) for n in FOCUS}}


def max_margin_ik(node,target,seed,trust_radius=.25):
    """Optimize the seventh DoF while enforcing all six pose equations."""
    previous=values(seed)
    bounds=np.asarray([node.limits[n] for n in JOINTS])
    # Reserve 0.005 rad for MoveIt's joint-goal tolerance beyond the 0.05
    # physical-execution gate, including J1–J4 as well as the wrist joints.
    low=np.maximum(bounds[:,0]+.055,previous-trust_radius)
    high=np.minimum(bounds[:,1]-.055,previous+trust_radius)
    rot=Rotation.from_matrix(target[:3,:3])
    def error(q):
        T=node.robot_chain.forward(dict(zip(JOINTS,q)))
        return np.r_[T[:3,3]-target[:3,3],
            .1*(rot.inv()*Rotation.from_matrix(T[:3,:3])).as_rotvec()]
    eps=1e-5
    J=np.column_stack([(error(previous+np.eye(7)[i]*eps)-error(previous))/eps for i in range(7)])
    tangent=np.linalg.svd(J,full_matrices=True)[2][-1]
    candidates=[];diagnostics=[]
    redundant_index=int(np.argmax(np.abs(tangent)))
    # The maximum-margin solution can collide. Sampling the redundant coordinate
    # retains lower-margin alternatives instead of converging every seed to it.
    options=[(v,None) for v in (0.,-.12,.12)]
    for displacement in (-.24,-.16,-.08,.08,.16,.24):
        fixed=float(np.clip(previous[redundant_index]+displacement,
                            low[redundant_index]+1e-7,high[redundant_index]-1e-7))
        options.append((0.,fixed))
    for offset,fixed in options:
        q0=np.clip(previous+offset*tangent,low,high)
        x0=np.r_[q0,max(.0,margins(node,q0)['focus_min_rad'])]
        def objective(x):
            return -x[7]+.03*np.sum((x[:7]-previous)**2)
        def limit_margin(x):
            return np.r_[x[4:7]-bounds[4:,0]-x[7],bounds[4:,1]-x[4:7]-x[7]]
        equalities=lambda x:error(x[:7]) if fixed is None else np.r_[error(x[:7]),x[redundant_index]-fixed]
        fit=minimize(objective,x0,method='SLSQP',
            bounds=[*zip(low,high),(0.,1.5)],
            constraints=[{'type':'eq','fun':equalities},
                         {'type':'ineq','fun':limit_margin}],
            options={'maxiter':65,'ftol':1e-8})
        q=fit.x[:7];res=error(q)
        detail={'solver_success':bool(fit.success),'redundant_joint':JOINTS[redundant_index],
            'fixed_redundant_q_rad':fixed,'position_error_m':float(np.linalg.norm(res[:3])),
            'rotation_error_rad':float(np.linalg.norm(res[3:])/.1),**margins(node,q)}
        if detail['position_error_m']>1e-4 or detail['rotation_error_rad']>.003:
            detail['status']='NO_EXACT_IK';diagnostics.append(detail);continue
        state=with_q(seed,q)
        req=GetStateValidity.Request();req.robot_state=state;req.group_name=GROUP
        check=node.call('check_state_validity',req)
        if not check.valid:
            detail['status']='COLLISION'
            detail['contacts']=[[c.contact_body_1,c.contact_body_2] for c in check.contacts]
            diagnostics.append(detail);continue
        fk=GetPositionFK.Request();fk.header.frame_id=BASE;fk.fk_link_names=[TCP];fk.robot_state=state
        measured=node.call('compute_fk',fk)
        if measured.error_code.val!=1:raise Failure('FRAME_ERROR','MoveIt FK failed for optimized IK')
        pose=measured.pose_stamped[0].pose
        dp=np.linalg.norm(np.array([pose.position.x,pose.position.y,pose.position.z])-target[:3,3])
        dr=(rot.inv()*Rotation.from_quat([pose.orientation.x,pose.orientation.y,
            pose.orientation.z,pose.orientation.w])).magnitude()
        detail['moveit_fk_error_m']=float(dp);detail['moveit_fk_error_rad']=float(dr)
        if dp>.001 or dr>.01:raise Failure('FRAME_ERROR','URDF optimized IK disagrees with MoveIt FK')
        detail['status']='VALID';diagnostics.append(detail)
        if any(np.linalg.norm(q-values(existing[0]))<.005 for existing in candidates):continue
        candidates.append((state,detail))
    candidates.sort(key=lambda item:(item[1]['focus_min_rad'],
        -np.linalg.norm(values(item[0])-previous)),reverse=True)
    return candidates,diagnostics


def plan_redundant_arc(node,current,T_grasp,initial,boxes,T_moving0,root0,
                       q_goal,*,step_deg=1.):
    """MoveIt joint plans whose dense FK stays on the prescribed door arc."""
    from .backend import matrix
    from .preflight import moved_door_boxes,trajectory_end
    q0=float(initial['joint_q']);q_previous=q0
    rows=[];trajectories=[]
    angles=np.linspace(q0,q_goal,max(2,int(math.ceil(math.degrees(q_goal-q0)/step_deg))+1))[1:]
    for q in angles:
        rootq=node.chain.root_to_link(initial['moving_link'],{initial['joint_name']:float(q)})
        T_movingq=T_moving0@np.linalg.inv(root0)@rootq
        target=node.chain.target_tcp(moving_link=initial['moving_link'],
            joint_name=initial['joint_name'],q_now=q0,q_target=float(q),
            T_world_moving_now=T_moving0,T_world_tcp_now=T_grasp)
        node.scene(collision_boxes=moved_door_boxes(boxes,T_moving0,T_movingq),
                   moving_pose_override=T_movingq)
        solutions,trials=max_margin_ik(node,target,current)
        row={'door_angle_deg':float(math.degrees(q)),
             'delta_angle_deg':float(math.degrees(q-q0)),'ik_trials':trials}
        rows.append(row)
        safe=[(s,d) for s,d in solutions if d['focus_min_rad']>.05 and d['all_joint_min_rad']>.05]
        if not safe:
            row['status']='LOW_JOINT_MARGIN' if solutions else (
                'ARC_COLLISION' if any(t['status']=='COLLISION' for t in trials) else 'NO_ARC_KINEMATIC_IK')
            return {'status':row['status'],'arc':rows,'failed_door_angle_deg':row['door_angle_deg']},trajectories
        accepted=None
        for goal,detail in safe[:3]:
            try:
                validity=GetStateValidity.Request();validity.robot_state=current;validity.group_name=GROUP
                start_check=node.call('check_state_validity',validity)
                pairs=[[c.contact_body_1,c.contact_body_2] for c in start_check.contacts]
                row['start_contacts_in_frozen_endpoint_scene']=pairs
                frozen_door_only=not start_check.valid and bool(pairs) and all('cabinet_door' in p for p in pairs)
                if frozen_door_only:
                    # OMPL sees a static world, while this door moves with TCP.
                    # Verify the real start under the preceding door angle,
                    # then allow the door only during OMPL's proposal. Every
                    # dense sample below restores full paired collision checks.
                    previous_root=node.chain.root_to_link(initial['moving_link'],{initial['joint_name']:q_previous})
                    previous_pose=T_moving0@np.linalg.inv(root0)@previous_root
                    node.scene(collision_boxes=moved_door_boxes(boxes,T_moving0,previous_pose),moving_pose_override=previous_pose)
                    node.validate(current)
                    node.scene(collision_boxes=moved_door_boxes(boxes,T_moving0,T_movingq),
                        moving_pose_override=T_movingq,allow_moving_door_for_planner=True)
                    row['moving_door_collision_check']='paired dense robot/door samples; static door relaxed only for OMPL proposal'
                trajectory=node.plan(current,goal)
                q_start=values(current)
                pts=trajectory.joint_trajectory.points
                names=trajectory.joint_trajectory.joint_names
                index=[names.index(n) for n in JOINTS]
                q_end=values(goal);direction=q_end-q_start
                samples=[np.asarray(p.positions)[index] for p in pts]
                # Densify in joint space, independent of the planner's timing.
                dense=[q_start]
                for point in samples:
                    count=max(1,int(np.ceil(np.max(np.abs(point-dense[-1]))/.015)))
                    dense.extend(np.linspace(dense[-1],point,count+1)[1:])
                max_position=0.;max_rotation=0.;minimum=1e9;all_minimum=1e9
                for qr in dense:
                    alpha=float(np.clip(np.dot(qr-q_start,direction)/max(1e-12,np.dot(direction,direction)),0,1))
                    door_angle=q_previous+alpha*(q-q_previous)
                    expected=node.chain.target_tcp(moving_link=initial['moving_link'],
                        joint_name=initial['joint_name'],q_now=q0,q_target=float(door_angle),
                        T_world_moving_now=T_moving0,T_world_tcp_now=T_grasp)
                    actual=node.robot_chain.forward(dict(zip(JOINTS,qr)))
                    dp=float(np.linalg.norm(actual[:3,3]-expected[:3,3]))
                    dr=float((Rotation.from_matrix(expected[:3,:3]).inv()*Rotation.from_matrix(actual[:3,:3])).magnitude())
                    max_position=max(max_position,dp);max_rotation=max(max_rotation,dr)
                    if dp>.002 or dr>math.radians(2):raise Failure('OFF_ARC','MoveIt joint path departed from door arc')
                    moving=node.chain.root_to_link(initial['moving_link'],{initial['joint_name']:float(door_angle)})
                    pose=T_moving0@np.linalg.inv(root0)@moving
                    node.scene(collision_boxes=moved_door_boxes(boxes,T_moving0,pose),moving_pose_override=pose)
                    node.validate(with_q(current,qr))
                    minimum=min(minimum,margins(node,qr)['focus_min_rad'])
                    all_minimum=min(all_minimum,margins(node,qr)['all_joint_min_rad'])
                if minimum<=.05 or all_minimum<=.05:raise Failure('LOW_JOINT_MARGIN','dense arc path margin below threshold for one of seven joints')
                row.update(status='PLANNED',margin_rad=minimum,
                    all_joint_min_margin_rad=all_minimum,
                    focus_margin_rad={n:detail[n] for n in FOCUS},
                    joint_margins_rad=detail['joint_margins_rad'],
                    q=values(goal).tolist(),points=len(pts),
                    max_tcp_error_m=max_position,max_tcp_rotation_error_rad=max_rotation)
                accepted=trajectory;break
            except Failure as failure:
                row.setdefault('plan_failures',[]).append({'status':failure.category,'detail':str(failure)})
                node.scene(collision_boxes=moved_door_boxes(boxes,T_moving0,T_movingq),moving_pose_override=T_movingq)
        if accepted is None:
            row['status']='NO_ARC_PLAN'
            return {'status':'NO_ARC_PLAN','arc':rows,'failed_door_angle_deg':row['door_angle_deg']},trajectories
        trajectories.append(accepted);current=trajectory_end(current,accepted);q_previous=float(q)
    return {'status':'FULL_PATH_PLANNED','arc':rows,
        'full_arc_min_margin_rad':min(r['margin_rad'] for r in rows),
        'full_arc_min_all_joint_margin_rad':min(r['all_joint_min_margin_rad'] for r in rows)},trajectories
