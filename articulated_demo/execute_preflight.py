"""Execute only an accepted complete arc; never command the cabinet joint."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .backend import Failure,matrix
import r1a7_plant as plant


def execute_preflight(node,plan,report,save,stop_requested=None):
    if not plan.get('arc_segments') or plan['arc'][-1]['door_angle_deg']<21.99:
        raise Failure('NO_PLAN','complete 0→22 degree preflight is required')
    if any(r['status']!='PLANNED' or r['margin_rad']<=.05 or r.get('all_joint_min_margin_rad',0.)<=.05 for r in plan['arc']):
        raise Failure('NO_PLAN','arc contains unsafe or unplanned segments')
    if not plant.command({'op':'resume'},timeout=10).get('ok'):
        raise RuntimeError('Isaac physics resume failed')
    plant.settle(.05)
    if abs(plant.state()['joint_q'])>math.radians(2):
        raise Failure('INITIAL_DOOR_DRIFT','door moved before grasp execution')
    node.stage='PREGRASP';node.execute(plan['pre_traj'],'NO_PLAN')
    node.stage='APPROACH';node.execute(plan['approach'],'NO_PLAN')
    node.stage='CLOSE';result=node.gripper(0.);plant.settle(.4)
    closed=plant.state();q0=float(closed['joint_q'])
    report['contact_at_close']={'finger_forces_n':closed['forces'],
        'gripper_stalled':bool(result.stalled),'gripper_reached_goal':bool(result.reached_goal),
        'joint_q_rad':q0}
    save()
    if not all(float(f)>.2 for f in closed['forces']):
        raise Failure('BAD_CONTACT','no bilateral measured handle contact')
    if abs(q0)>math.radians(2):
        raise Failure('BAD_CONTACT','door moved outside the preflight start tolerance')
    def relative(state):
        return np.linalg.inv(matrix(state['moving_pose']['position'],
            state['moving_pose']['quaternion_wxyz']))@matrix(state['tcp'],state['tcp_quat'])
    reference=relative(closed)
    def slip(state):
        current=relative(state)
        return (float(np.linalg.norm(current[:3,3]-reference[:3,3])),
            float((Rotation.from_matrix(reference[:3,:3]).inv()*Rotation.from_matrix(current[:3,:3])).magnitude()))
    report['joint_path']=[]
    for i,(trajectory,row) in enumerate(zip(plan['arc_segments'],plan['arc'])):
        if stop_requested and stop_requested():raise Failure('CUTOFF_05_00','experiment deadline reached')
        before=plant.state();dp,dr=slip(before)
        if dp>.005 or dr>math.radians(10):raise Failure('CONTACT_LOSS','slip before arc segment')
        node.scene();node.validate(node.measured())
        node.stage=f'OPEN_{i+1}';node.execute(trajectory,'NO_PLAN')
        after=plant.state();dp,dr=slip(after)
        sample={'requested_joint_angle_deg':row['door_angle_deg'],
            'actual_joint_angle_deg':math.degrees(after['joint_q']),
            'relative_tcp_slip_m':dp,'relative_tcp_slip_rad':dr,
            'forces_n':after['forces'],'planned_margin_rad':row['margin_rad']}
        report['joint_path'].append(sample);save()
        if dp>.005 or dr>math.radians(10):raise Failure('CONTACT_LOSS','Dex1 slipped during opening')
        if not all(float(f)>.2 for f in after['forces']):raise Failure('CONTACT_LOSS','bilateral contact lost during opening')
        if abs(after['joint_q']-math.radians(row['door_angle_deg']))>math.radians(3):
            raise Failure('JOINT_STUCK','door angle does not track the planned arc')
        node.scene();node.validate(node.measured())
    start=plant.state()['t'];plant.settle(2.2);final=plant.state()
    hold=[h for h in final['history'] if h['t']>=start]
    dp,dr=slip(final)
    report['hold']={'samples':len(hold),'duration_s':hold[-1]['t']-hold[0]['t'] if len(hold)>1 else 0.,
        'min_joint_delta_deg':min((math.degrees(h['joint_q']-q0) for h in hold),default=0.),
        'relative_tcp_slip_m':dp,'relative_tcp_slip_rad':dr,
        'min_bilateral_force_n':min((min(h['forces']) for h in hold),default=0.)}
    report['actual_joint_delta_deg']=math.degrees(final['joint_q']-q0);save()
    if report['hold']['duration_s']<2 or report['hold']['min_joint_delta_deg']<20:
        raise Failure('CONTACT_LOSS','actual door did not hold 20 degrees for two seconds')
    if dp>.005 or dr>math.radians(10) or report['hold']['min_bilateral_force_n']<=.2:
        raise Failure('CONTACT_LOSS','contact or relative transform failed during hold')
    report['status']='SUCCESS'
