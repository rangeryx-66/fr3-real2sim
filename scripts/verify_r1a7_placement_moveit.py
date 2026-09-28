#!/usr/bin/env python3
"""Validate grasp, segment, interaction and self collision for a base pose.

Only MoveIt compute_ik, check_state_validity and optional scene updates are
called. No trajectory planning, robot action, simulation, or grasp execution.
"""
import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from r1a7_workspace_diagnostics import NumericalIK, JOINTS, HOME, frozen_targets, transform
from verify_r1a7_workspace_moveit import Client
from r1a7_placement_robustness import (PREGRASP_M, LIFT_M, STEP_M,
                                       INTERACTION_M, MAX_JOINT_STEP_RAD, FOCUS)


class RobustClient(Client):
    def __init__(self):
        super().__init__()
        self.valid_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not self.valid_client.wait_for_service(timeout_sec=60):
            raise RuntimeError('MoveIt state validity unavailable')

    def wait_result(self, client, request, timeout=10):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception():
            raise RuntimeError('MoveIt service timeout')
        return future.result()

    def state(self, q):
        state = RobotState()
        state.joint_state = JointState()
        state.joint_state.name = list(JOINTS)+['dex1_Joint1_1','dex1_Joint2_1']
        state.joint_state.position = list(map(float,q))+[-.02,-.02]
        return state

    def valid(self, q):
        req = GetStateValidity.Request()
        req.group_name = 'r1a7_arm'
        req.robot_state = self.state(q)
        result = self.wait_result(self.valid_client, req)
        return bool(result.valid), [[c.contact_body_1,c.contact_body_2] for c in result.contacts]

    def apply_scene(self, scene_json, world_to_model):
        """Optional future table/mount collision hook, world-frame box schema."""
        spec = json.loads(Path(scene_json).read_text())
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        applied = []
        for item in spec.get('collision_objects', []):
            if not item.get('enabled', True):
                continue
            obj = CollisionObject()
            obj.id = item['id']
            obj.header.frame_id = 'r1a7_world'
            obj.operation = CollisionObject.ADD
            shape = SolidPrimitive()
            shape.type = SolidPrimitive.BOX
            shape.dimensions = list(map(float,item['size_m']))
            center = world_to_model @ np.r_[item['center_world_m'],1.]
            pose = Pose()
            pose.position.x,pose.position.y,pose.position.z = map(float,center[:3])
            from scipy.spatial.transform import Rotation
            world_rotation = Rotation.from_quat(item.get('orientation_world_xyzw',[0,0,0,1]))
            q = (Rotation.from_matrix(world_to_model[:3,:3])*world_rotation).as_quat()
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = map(float,q)
            obj.primitives = [shape]
            obj.primitive_poses = [pose]
            req.scene.world.collision_objects.append(obj)
            applied.append(obj.id)
        if not self.scene_client.wait_for_service(timeout_sec=60):
            raise RuntimeError('MoveIt planning scene service unavailable')
        result = self.wait_result(self.scene_client, req)
        if not result.success:
            raise RuntimeError('MoveIt scene update rejected')
        return applied


def margin(q, solver):
    m = np.minimum(q-solver.lo,solver.hi-q)
    return {j:float(v) for j,v in zip(JOINTS,m)}


def shift(T, delta):
    result = T.copy()
    result[:3,3] += delta
    return result


def pose_set(grasp):
    approach = grasp[:3,2]
    pre = [shift(grasp,-a*approach)
           for a in np.arange(STEP_M,PREGRASP_M+1e-8,STEP_M)]
    lift = [shift(grasp,[0,0,a])
            for a in np.arange(STEP_M,LIFT_M+1e-8,STEP_M)]
    interactions = {}
    for name,unit in [('xp',[1,0,0]),('xm',[-1,0,0]),
                      ('yp',[0,1,0]),('ym',[0,-1,0]),
                      ('zp',[0,0,1]),('zm',[0,0,-1])]:
        interactions[name] = [shift(grasp,a*np.array(unit))
                              for a in (INTERACTION_M/2,INTERACTION_M)]
    return pre,lift,interactions


def follow(node, solver, start_q, world_poses, world_to_model, timeout):
    q = np.array(start_q)
    steps = []
    for world in world_poses:
        target = world_to_model @ world
        next_q,code = node.ik(target,q,False,timeout)
        if next_q is None:
            return dict(ok=False,reason='NO_IK',code=code,steps=steps)
        valid,contacts = node.valid(next_q)
        if not valid:
            alternative,collision_code = node.ik(target,q,True,timeout)
            if alternative is None:
                return dict(ok=False,reason='SELF_OR_SCENE_COLLISION',
                            code=collision_code,contacts=contacts,steps=steps)
            next_q = alternative
        jump = float(np.max(np.abs(next_q-q)))
        if jump>MAX_JOINT_STEP_RAD:
            return dict(ok=False,reason='JOINT_JUMP',jump_rad=jump,steps=steps)
        q = next_q
        margins = margin(q,solver)
        steps.append(dict(q=q.tolist(),margins_rad=margins,jump_rad=jump))
    return dict(ok=True,steps=steps)


def branch_paths(node, solver, q, grasp, world_to_model, timeout):
    pre,lift,interactions = pose_set(grasp)
    approach = follow(node,solver,q,pre,world_to_model,timeout)
    lifting = follow(node,solver,q,lift,world_to_model,timeout)
    interaction = {name:follow(node,solver,q,poses,world_to_model,timeout)
                   for name,poses in interactions.items()}
    all_results = [approach,lifting,*interaction.values()]
    margins = [margin(q,solver)[j] for j in ('J5','J6','J7')]
    for result in all_results:
        for step in result['steps']:
            margins.extend(step['margins_rad'][j] for j in ('J5','J6','J7'))
    return dict(pregrasp=approach,lift=lifting,interactions=interaction,
                pregrasp_lift_ok=approach['ok'] and lifting['ok'],
                interaction_directions=sum(r['ok'] for r in interaction.values()),
                full_ok=all(r['ok'] for r in all_results),
                path_focus_margin_min_rad=min(margins))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--base',nargs=4,type=float,required=True)
    p.add_argument('--seeds',type=int,default=12)
    p.add_argument('--branches',type=int,default=3)
    p.add_argument('--timeout',type=float,default=.15)
    p.add_argument('--tcp-offset',nargs=3,type=float,default=(0.,0.,0.),
                   metavar=('DX','DY','DZ'),
                   help='hypothetical TCP translation in the frozen grasp local frame, metres')
    p.add_argument('--scene-json',type=Path)
    p.add_argument('--scene-only',action='store_true',
                   help='load optional scene objects and check home, then exit')
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args()
    solver = NumericalIK()
    rng = np.random.default_rng(20260928)
    world_to_model = transform(yaw=90.) @ np.linalg.inv(transform(*args.base))
    rclpy.init()
    node = RobustClient()
    try:
        added = node.apply_scene(args.scene_json,world_to_model) if args.scene_json else []
        if args.scene_only:
            valid,contacts = node.valid(HOME)
            result = dict(base=list(args.base),scene_objects=added,
                          home_valid=valid,home_contacts=contacts)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(result,indent=2)+'\n')
            print('SCENE_ONLY',result,flush=True)
            return
        rows = []
        tcp_shift=np.eye(4)
        tcp_shift[:3,3]=args.tcp_offset
        for rank,frozen_grasp in enumerate(frozen_targets()):
            grasp=frozen_grasp @ tcp_shift
            target = world_to_model @ grasp
            grasp_solutions = []
            codes = []
            for seed in [HOME,*rng.uniform(solver.lo+.01,solver.hi-.01,size=(args.seeds,7))]:
                q,code = node.ik(target,seed,False,args.timeout)
                codes.append(code)
                if q is None:
                    continue
                valid,contacts = node.valid(q)
                if not valid:
                    alternative,_ = node.ik(target,q,True,args.timeout)
                    if alternative is None:
                        continue
                    q = alternative
                if any(np.max(np.abs(q-other))<.05 for other in grasp_solutions):
                    continue
                grasp_solutions.append(q)
            row = dict(rank=rank,kinematic_ik=1 in codes,
                       self_collision_free_ik=bool(grasp_solutions),
                       grasp_codes=codes,solution_count=len(grasp_solutions))
            if grasp_solutions:
                grasp_solutions.sort(key=lambda q:min(margin(q,solver)[j]
                                                    for j in ('J5','J6','J7')),reverse=True)
                options = []
                for q in grasp_solutions[:args.branches]:
                    result = branch_paths(node,solver,q,grasp,world_to_model,args.timeout)
                    options.append(dict(grasp_q=q.tolist(),grasp_margins_rad=margin(q,solver),
                                        **result))
                chosen = max(options,key=lambda r:(r['full_ok'],r['pregrasp_lift_ok'],
                                                 r['interaction_directions'],
                                                 r['path_focus_margin_min_rad']))
                row.update(chosen)
            rows.append(row)
            print(f'{rank+1}/11 IK={row["kinematic_ik"]} self={row["self_collision_free_ik"]} '
                  f'segment={row.get("pregrasp_lift_ok",False)} '
                  f'interaction={row.get("interaction_directions",0)}/6 '
                  f'full={row.get("full_ok",False)}',flush=True)
        m = [min(row['grasp_margins_rad'][j] for j in ('J5','J6','J7'))
             for row in rows if row['self_collision_free_ik']]
        output = dict(base=list(args.base),tcp_offset_m=list(args.tcp_offset),scene_objects=added,
                      definitions=dict(pregrasp_m=PREGRASP_M,lift_m=LIFT_M,
                                       step_m=STEP_M,interaction_m=INTERACTION_M,
                                       max_joint_step_rad=MAX_JOINT_STEP_RAD),
                      summary=dict(grasp_ik=sum(r['kinematic_ik'] for r in rows),
                                   self_collision_free_grasps=sum(r['self_collision_free_ik'] for r in rows),
                                   pregrasp_lift=sum(r.get('pregrasp_lift_ok',False) for r in rows),
                                   all_interactions=sum(r.get('full_ok',False) for r in rows),
                                   interaction_directions=sum(r.get('interaction_directions',0) for r in rows),
                                   focus_grasp_margin_median_rad=float(np.median(m)) if m else None,
                                   focus_grasp_margin_min_rad=min(m,default=None),
                                   safe_grasp_005=sum(v>.05 for v in m),
                                   safe_grasp_008=sum(v>.08 for v in m),
                                   safe_complete_005=sum(r.get('full_ok',False) and r['path_focus_margin_min_rad']>.05 for r in rows),
                                   safe_complete_008=sum(r.get('full_ok',False) and r['path_focus_margin_min_rad']>.08 for r in rows)),
                      candidates=rows)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(output,indent=2)+'\n')
        print('DONE',args.output,output['summary'],flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
