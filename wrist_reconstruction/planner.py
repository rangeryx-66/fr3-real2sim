"""Hierarchical robot + existing SE(2) platform wrist-view planning.

No object joint queries; scene placement comes from the latest RGB-D estimate.
Every executed route reuses mobile.route and every arm edge retains PhysicalScene.
"""
import json,time,copy
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from piper_mobile_demo.model import Model
from piper_mobile_demo.owned_scene import Shape,intersects
from interactive_twin_recovery.mobile import candidate_bases,scene_at,home_valid,route
from wrist_reconstruction.geometry import optical_to_tcp,visibility

class PlanningExhausted(RuntimeError):
    pass

class MobileWristPlanner:
    def __init__(self,recovery):
        self.recovery=recovery;self.r=recovery.r;self.root=recovery.root;self.model=recovery.model
        self.policy=self.r.capture.config['mobile_scan'];self.rows=[]
    def save(self):
        p=self.r.capture.output/'mobile_wrist_planning.json';p.write_text(json.dumps(self.rows,indent=2))
    def scene(self,base):
        data=self.recovery.export_current(self.recovery.current_D)
        return scene_at(self.root,data,self.model,self.r.base,base)
    def check(self,scene,q,base,fingers):
        if self.model.margin(np.asarray(q))<=.05:return False,'LOW_JOINT_MARGIN',None
        P=self.model.poses(q,base,finger_q=fingers);ok,why=scene.check(P,scene.moving_reference,False)
        if not ok:return False,why,None
        C=P['tcp_link']@self.r.capture.cal['X'];camera=Shape(trimesh.creation.box(self.r.capture.cal['camera_body_size_m']),C,True,'/World/wrist_camera_housing','camera')
        for obstacle in scene.scene:
            if intersects(camera,obstacle):return False,'SCAN_CAMERA_ENVIRONMENT_COLLISION:'+obstacle.path,None
        for robot in scene.robot:
            # Camera is mounted to this rigid wrist cluster; all other links,
            # including the movable fingers, remain checked.
            if robot.body not in ('link6','flange_link','gripper_base') and intersects(camera,robot):return False,'SCAN_CAMERA_SELF_COLLISION:'+robot.body,None
        return True,'SAFE',None
    def arm_path(self,scene,start,goal,base,fingers):
        outer=self
        class View:
            def __getattr__(self,name):return getattr(outer.model,name)
            def check(self,q,b):return outer.check(scene,q,b,fingers)
        return Model.joint_plan(View(),np.asarray(start),np.asarray(goal),base,iterations=self.policy['arm_rrt_iterations'])
    def arm_only(self,T_camera,base,start,row,path=True):
        goal=self.model.ik(optical_to_tcp(T_camera,self.r.capture.cal['X']),base,seed=start,starts=5)
        if goal is None:row['status']='SCAN_NO_IK';return None
        scene=self.scene(base);ok,why,_=self.check(scene,goal,base,self.r.finger_q())
        if not ok:row['status']=why;return None
        row.update(minimum_joint_margin_rad=self.model.margin(goal),status='GOAL_VALID')
        result={'base':list(base),'q_goal':goal.tolist(),'T_camera':T_camera.tolist()}
        if path:
            edge=self.arm_path(scene,start,goal,base,self.r.finger_q())
            if edge is None:row['status']='SCAN_NO_COLLISION_FREE_ARM_PATH';return None
            result['arm_path']=edge;row['status']='ARM_PATH_VALID'
        return result
    def home(self):
        r=self.r;scene=self.scene(r.base);start=r.arm_q();row={'operation':'home','base':list(r.base),'alternatives':[]};self.rows.append(row)
        # Separate bounded seeds/planner attempts; a failed RRT draw is not a
        # proof that every home path is impossible.
        for attempt in range(self.policy['home_path_attempts']):
            edge=self.arm_path(scene,start,self.model.home,r.base,r.finger_q());row['alternatives'].append({'attempt':attempt,'valid':edge is not None});self.save()
            if edge is not None:r.execute_arm_path(edge,'SYSTEM_SCAN_HOME');return
        raise PlanningExhausted('NO_SAFE_HOME_PATH_RECOVERY_EXHAUSTED')
    def plan(self,T_camera,require_coverage=True):
        r=self.r;T_camera=np.asarray(T_camera);entry={'operation':'view','requested_T_camera':T_camera.tolist(),'initial_base':list(r.base),'candidates':[]};self.rows.append(entry)
        if require_coverage and visibility(r.capture.observed_cloud,T_camera,r.capture.cal['K'],r.capture.cal['resolution_wh'])<r.capture.config['capture']['minimum_initial_cloud_in_frame']:
            entry['status']='SCAN_VIEW_CROPPED';self.save();raise PlanningExhausted('SCAN_VIEW_CROPPED')
        fixed={'base':list(r.base),'kind':'arm-only'};entry['candidates'].append(fixed)
        choice=self.arm_only(T_camera,r.base,r.arm_q(),fixed)
        if choice is not None:choice['route']=None;entry['status']='ARM_ONLY';self.save();return choice
        # The camera target is a task-space goal, not an asset-specific offset.
        normal=-T_camera[:3,2];normal[2]=0
        if np.linalg.norm(normal)<1e-8:normal=np.asarray(r.initial_visual['outward_normal_world']).copy();normal[2]=0
        visual={'anchor_world_m':optical_to_tcp(T_camera,r.capture.cal['X'])[:3,3].tolist(),'outward_normal_world':normal.tolist()}
        bases=candidate_bases(r.base,visual,self.policy);coarse=[];started=time.monotonic()
        for base in bases[:self.policy['maximum_base_candidates_per_view']]:
            if time.time()>=r.deadline or time.monotonic()-started>self.policy['search_wall_s']:break
            row={'base':base,'kind':'mobile'};entry['candidates'].append(row)
            scene=self.scene(base);ok,why,gap=home_valid(scene,self.model,base)
            if not ok:row['status']=why;continue
            c=self.arm_only(T_camera,base,self.model.home,row,path=False)
            if c is None:continue
            row.update(clearance_m=gap,travel_m=float(np.linalg.norm(np.subtract(base[:2],r.base[:2]))));c['score']=[row['minimum_joint_margin_rad'],gap,-row['travel_m']];c['row']=row;coarse.append(c)
        coarse.sort(key=lambda c:c['score'],reverse=True);self.save()
        for c in coarse[:self.policy['full_path_base_budget']]:
            base=c['base'];row=c['row'];scene=self.scene(base)
            edge=self.arm_path(scene,self.model.home,c['q_goal'],base,r.finger_q())
            if edge is None:row['status']='SCAN_NO_COLLISION_FREE_ARM_PATH';self.save();continue
            path=route(self.root,self.recovery.export_current(self.recovery.current_D),self.model,list(r.base),base)
            row['route']=path;self.save()
            if not path['valid']:row['status']='NO_COLLISION_FREE_BASE_ROUTE';continue
            camera_route_ok=True
            for route_base in path['waypoints']:
                route_scene=self.scene(route_base);safe,why,_=self.check(route_scene,self.model.home,route_base,r.finger_q())
                if not safe:row['status']='BASE_ROUTE_CAMERA_'+why;camera_route_ok=False;break
            if not camera_route_ok:self.save();continue
            c.update(arm_path=edge,route=path);c.pop('row');row['status']='MOBILE_VIEW_PREFLIGHT_PASSED';entry['status']='MOBILE_VIEW_PREFLIGHT_PASSED';self.save();return c
        entry['status']='MOBILE_VIEW_RECOVERY_EXHAUSTED';self.save();raise PlanningExhausted('MOBILE_VIEW_RECOVERY_EXHAUSTED')
    def execute(self,T_camera,require_coverage=True):
        r=self.r
        if not self.recovery.released:raise RuntimeError('BASE_SCAN_REQUIRES_PHYSICAL_RELEASE')
        choice=self.plan(T_camera,require_coverage)
        if choice['route'] is not None:
            self.home();self.recovery.move_base(choice)
        r.execute_arm_path(choice['arm_path'],'SYSTEM_WRIST_SCAN');return choice
    def observe_handle(self,anchor):
        from wrist_reconstruction.geometry import look_at
        r=self.r;normal=np.asarray(self.recovery.visual_current(self.recovery.current_D)['outward_normal_world']);anchor=np.asarray(anchor);attempts=[]
        for angle in [0,-25,25,-50,50]:
            direction=Rotation.from_euler('z',angle,degrees=True).apply(normal)
            for distance in self.policy['handle_observe_standoff_m']:
                eye=anchor+direction*distance+np.array([0,0,.06]);T=look_at(eye,anchor)
                try:return self.execute(T,require_coverage=False)
                except PlanningExhausted as e:attempts.append({'azimuth_deg':angle,'distance_m':distance,'reason':str(e)})
        (r.capture.output/'handle_reobserve_exhausted.json').write_text(json.dumps(attempts,indent=2));raise PlanningExhausted('HANDLE_REOBSERVE_RECOVERY_EXHAUSTED')
