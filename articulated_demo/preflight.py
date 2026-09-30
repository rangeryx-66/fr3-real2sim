"""Read-only MoveIt grasp and 0→22 degree door-path preflight."""
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity

from .backend import matrix
from r1a7_backend import BASE, FOCUS, GROUP, JOINTS, TCP, Failure, stamped


def trajectory_end(start, trajectory):
    result = RobotState()
    result.joint_state.name = list(start.joint_state.name)
    values = dict(zip(start.joint_state.name, start.joint_state.position))
    last = trajectory.joint_trajectory.points[-1]
    values.update(zip(trajectory.joint_trajectory.joint_names, last.positions))
    result.joint_state.position = [float(values[n]) for n in result.joint_state.name]
    return result


def moved_door_boxes(boxes, T_moving0, T_movingq):
    result = [dict(box) for box in boxes]
    for box in result:
        if box['id'] != 'cabinet_door': continue
        T_box0 = matrix(box['center'],box['quaternion_wxyz'])
        T_boxq = T_movingq @ np.linalg.inv(T_moving0) @ T_box0
        box['center'] = T_boxq[:3,3].tolist()
        box['quaternion_wxyz'] = np.roll(Rotation.from_matrix(T_boxq[:3,:3]).as_quat(),1).tolist()
    return result


def probe_ik(node, target, seed, *, random_seeds=8, timeout_s=.25,
             optimized_seeds=2):
    """Return every layer separately; collision never hides kinematic IK."""
    values = dict(zip(seed.joint_state.name,seed.joint_state.position))
    attempts = [values]
    for _ in range(random_seeds):
        trial = dict(values)
        for name in JOINTS:
            lo,hi = node.limits[name]
            trial[name] = float(node.rng.uniform(lo+.01,hi-.01))
        attempts.append(trial)
    fit_hits = 0
    if optimized_seeds:
        bounds=np.asarray([node.limits[name] for name in JOINTS])
        lower,upper=bounds[:,0]+.01,bounds[:,1]-.01
        target_rotation=Rotation.from_matrix(target[:3,:3])
        def residual(q):
            predicted=node.robot_chain.forward(dict(zip(JOINTS,q)))
            rotation=(target_rotation.inv()*Rotation.from_matrix(predicted[:3,:3])).as_rotvec()
            return np.r_[predicted[:3,3]-target[:3,3],.08*rotation]
        initial=np.asarray([values[name] for name in JOINTS])
        for q0 in [initial,*node.rng.uniform(lower,upper,size=(max(0,optimized_seeds-1),7))]:
            fit=least_squares(residual,np.clip(q0,lower,upper),bounds=(lower,upper),
                max_nfev=100,ftol=1e-4,xtol=1e-4,gtol=1e-4)
            if np.linalg.norm(fit.fun[:3])>.005 or np.linalg.norm(fit.fun[3:])/.08>math.radians(5):
                continue
            fit_hits += 1
            attempts.append({**values,**dict(zip(JOINTS,fit.x))})
    result = {'seeds':len(attempts),'timeout_s':timeout_s,
              'urdf_optimized_seed_hits':fit_hits,
              'kinematic_ik':0,'collision_free':0,'margin_over_005':0,
              'margin_over_008':0,'collisions':[],'collision_reason_counts':{},
              'solutions':[]}
    best = None
    for trial in attempts:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = GROUP; ik.ik_link_name = TCP
        ik.pose_stamped = stamped(target)
        ik.avoid_collisions = False
        ik.robot_state = RobotState()
        ik.robot_state.joint_state.name = list(seed.joint_state.name)
        ik.robot_state.joint_state.position = [float(trial[n]) for n in seed.joint_state.name]
        ik.timeout.sec = int(timeout_s)
        ik.timeout.nanosec = int((timeout_s % 1)*1e9)
        response = node.call('compute_ik',request)
        if response.error_code.val != 1: continue
        result['kinematic_ik'] += 1
        validity = GetStateValidity.Request()
        validity.robot_state = response.solution
        validity.group_name = GROUP
        checked = node.call('check_state_validity',validity)
        if not checked.valid:
            pairs=[[c.contact_body_1,c.contact_body_2] for c in checked.contacts]
            result['collisions'].append(pairs)
            flat={body for pair in pairs for body in pair}
            for object_id,reason in (('table','TABLE_COLLISION'),
                ('r1a7_pedestal','PEDESTAL_COLLISION'),
                ('cabinet_static','CABINET_STATIC_COLLISION'),
                ('cabinet_door','CABINET_DOOR_COLLISION'),
                ('cabinet_fixture','FIXTURE_COLLISION')):
                if object_id in flat:
                    counts=result['collision_reason_counts']
                    counts[reason]=counts.get(reason,0)+1
            world_ids={'table','r1a7_pedestal','cabinet_static',
                       'cabinet_door','cabinet_fixture'}
            if any(a not in world_ids and b not in world_ids for a,b in pairs):
                counts=result['collision_reason_counts']
                counts['ROBOT_SELF_COLLISION']=counts.get('ROBOT_SELF_COLLISION',0)+1
            continue
        result['collision_free'] += 1
        margin = float(node.margin(response.solution))
        q = dict(zip(response.solution.joint_state.name,response.solution.joint_state.position))
        result['solutions'].append({'margin_rad':margin,
            'focus_margin_rad':{n:float(min(q[n]-node.limits[n][0],node.limits[n][1]-q[n])) for n in FOCUS}})
        result['margin_over_005'] += margin > .05
        result['margin_over_008'] += margin > .08
        if best is None or margin > best[0]: best = margin,response.solution
    return result,best[1] if best else None


def candidate_preflight(node, T_grasp, *, random_seeds=8, timeout_s=.25,
                        arc_step_deg=2.,optimize_redundancy=False,plan_sink=None,arc_goal_deg=22.):
    """Plan without execution; update moving-door OBB at every arc step."""
    import r1a7_plant as plant
    row = {'status':'STARTED','arc_waypoints_planned':0}
    initial = plant.state()
    boxes = initial['collision_boxes']
    T_moving0 = matrix(initial['moving_pose']['position'],
                       initial['moving_pose']['quaternion_wxyz'])
    q0 = float(initial['joint_q'])
    if not 0.<arc_goal_deg<=22.:raise ValueError('arc goal must be within 0→22 degrees')
    q_goal = min(math.radians(arc_goal_deg),float(initial['joint_limits']['upper'])-.02)
    row['arc_start_deg'] = float(math.degrees(q0))
    row['arc_goal_deg'] = float(math.degrees(q_goal))
    row['initial_door_angle_ok'] = abs(q0) <= math.radians(2)
    if q0 >= q_goal:
        row['status']='INITIAL_DOOR_DRIFT'
        return row
    try:
        node.scene()
        home = node.measured()
        grasp,grasp_state = probe_ik(node,T_grasp,home,
            random_seeds=random_seeds,timeout_s=timeout_s)
        row['grasp'] = grasp
        if not grasp['kinematic_ik']:
            row['status']='NO_KINEMATIC_IK'; return row
        if not grasp['collision_free']:
            row['status']='COLLISION'; return row
        if not grasp['margin_over_005']:
            row['status']='LOW_JOINT_MARGIN'; return row
        pre = T_grasp.copy(); pre[:3,3] -= .08*T_grasp[:3,2]
        pregrasp,pre_state = probe_ik(node,pre,grasp_state,
            random_seeds=random_seeds,timeout_s=timeout_s)
        row['pregrasp'] = pregrasp
        if not pregrasp['kinematic_ik']:
            row['status']='NO_PREGRASP_IK'; return row
        if not pregrasp['collision_free']:
            row['status']='PREGRASP_COLLISION'; return row
        if not pregrasp['margin_over_005']:
            row['status']='LOW_JOINT_MARGIN'; return row
        try:
            approach = node.cartesian(pre_state,T_grasp)
            row['approach_points']=len(approach.joint_trajectory.points)
        except Failure as error:
            row['status']='NO_APPROACH'; row['detail']=str(error); return row
        try:
            preplan = node.plan(home,pre_state)
            row['pregrasp_plan_points']=len(preplan.joint_trajectory.points)
        except Failure as error:
            row['status']='NO_PREGRASP_PLAN'; row['detail']=str(error); return row
        current = trajectory_end(pre_state,approach)
        row['grasp_joint_q']=[dict(zip(current.joint_state.name,current.joint_state.position))[n] for n in JOINTS]
        root0 = node.chain.root_to_link(initial['moving_link'],
                                        {initial['joint_name']:q0})
        if optimize_redundancy:
            from .redundant_path import plan_redundant_arc
            outcome,segments=plan_redundant_arc(node,current,T_grasp,initial,
                boxes,T_moving0,root0,q_goal)
            row.update(outcome)
            row['arc_waypoints_planned']=sum(r['status']=='PLANNED' for r in row['arc'])
            if row['status']=='FULL_PATH_PLANNED' and not row['initial_door_angle_ok']:
                row['status']='INITIAL_DOOR_DRIFT'
            if row['status']=='FULL_PATH_PLANNED' and plan_sink is not None:
                plan_sink.update(pre_traj=preplan,approach=approach,arc_segments=segments,
                    initial_joint_q_rad=q0,grasp=T_grasp,arc=row['arc'],arc_goal_deg=arc_goal_deg)
            return row
        arc = []
        row['arc'] = arc
        for q in np.linspace(q0,q_goal,max(2,int(math.ceil(math.degrees(q_goal-q0)/arc_step_deg))+1))[1:]:
            rootq = node.chain.root_to_link(initial['moving_link'],
                                             {initial['joint_name']:float(q)})
            T_movingq = T_moving0 @ np.linalg.inv(root0) @ rootq
            target = node.chain.target_tcp(moving_link=initial['moving_link'],
                joint_name=initial['joint_name'],q_now=q0,q_target=float(q),
                T_world_moving_now=T_moving0,T_world_tcp_now=T_grasp)
            node.scene(collision_boxes=moved_door_boxes(boxes,T_moving0,T_movingq),
                       moving_pose_override=T_movingq)
            try:
                segment = node.cartesian(current,target)
            except Failure as error:
                diagnostic,_=probe_ik(node,target,current,
                    random_seeds=random_seeds,timeout_s=timeout_s)
                row['arc_failure_ik']=diagnostic
                row['status']=('NO_ARC_KINEMATIC_IK' if not diagnostic['kinematic_ik']
                    else 'ARC_COLLISION' if not diagnostic['collision_free']
                    else 'LOW_JOINT_MARGIN' if not diagnostic['margin_over_005']
                    else 'NO_ARC_PLAN')
                row['detail']=str(error)
                row['failed_door_angle_deg']=float(math.degrees(q))
                row['failed_arc_angle_deg']=float(math.degrees(q-q0));return row
            current = trajectory_end(current,segment)
            arc.append({'joint_angle_deg':float(math.degrees(q-q0)),
                'points':len(segment.joint_trajectory.points),
                'margin_rad':float(node.trajectory_margin(segment))})
            row['arc_waypoints_planned'] += 1
        row['arc']=arc
        row['status']='FULL_PATH_PLANNED' if row['initial_door_angle_ok'] else 'INITIAL_DOOR_DRIFT'
        return row
    finally:
        node.scene(collision_boxes=boxes)
