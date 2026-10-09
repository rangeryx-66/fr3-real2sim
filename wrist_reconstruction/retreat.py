"""Finite release/escape alternatives; original collision and load guards stay on."""
import time,json
import numpy as np
from piper_mobile_demo.model import Model
from wrist_reconstruction.planner import PlanningExhausted

def release_aperture(actual,successful,extra,certified=False):
    """A released pose uses actual width; failed closure retains proven width."""
    if certified:return actual
    return min(.1,max(actual,successful or 0.)+extra)

class RetreatPlanner:
    def __init__(self,recovery):self.recovery=recovery;self.r=recovery.r;self.model=recovery.model;self.policy=self.r.capture.config['retreat'];self.rows=[]
    def save(self): (self.r.capture.output/'retreat_alternatives.json').write_text(json.dumps(self.rows,indent=2))
    def plans(self,D):
        r=self.r;E=r.tcp();start=r.arm_q();normal=np.asarray(self.recovery.visual_current(D)['outward_normal_world']);back=-E[:3,2];up=np.array([0.,0,1]);lateral=E[:3,0]
        directions=[back,back+.5*up,normal,normal+.5*up,back+.25*lateral,back-.25*lateral]
        scene=self.recovery.mobile.scene(r.base);plans=[];started=time.monotonic()
        actual=float(r.finger_q()[0]-r.finger_q()[1])
        template=getattr(self.recovery,'template',None)
        successful=None if not template else float(template['actual_aperture_m'])
        # An empty failed closure can approach zero aperture. Releasing from
        # that width +20mm is not clearance for a previously held 24mm handle.
        openings=[release_aperture(actual,successful,x) for x in self.policy['extra_openings_m']]
        maximum=r.capture.config.get('maximum_range',{}).get('enabled')
        if maximum and self.recovery.released:
            # A prior release certificate authorizes the measured opening only.
            # Never preflight a wider aperture that execution will not command.
            openings=[float(r.finger_q()[0]-r.finger_q()[1])]
        if maximum and not self.recovery.released:
            # Both original release apertures must be considered. Choosing
            # +20mm unconditionally discarded the already configured +40mm
            # alternative when a rotated handle intersected the finger root.
            for opening in openings:
                ok,why,_=self.recovery.mobile.check(scene,start,r.base,np.array([opening/2,-opening/2]))
                self.rows.append({'index':len(self.rows),'kind':'release-aperture-preflight','base':list(r.base),'release_opening_m':opening,'status':'RELEASE_APERTURE_PREFLIGHT_PASSED' if ok else why});self.save()
                if ok:
                    openings=[opening];break
        alternatives=[(d,openings[0]) for d in directions] if maximum else [(d,o) for o in openings for d in directions]
        cartesian_wall=self.policy['planning_wall_s']*(.5 if maximum else 1.)
        for index,(direction,opening) in enumerate(alternatives):
            if index>=self.policy['maximum_alternatives'] or time.monotonic()-started>cartesian_wall:break
            direction=direction/np.linalg.norm(direction);row={'index':len(self.rows),'direction_world':direction.tolist(),'release_opening_m':opening,'base':list(r.base)};self.rows.append(row)
            def check(q,b):
                if self.model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN',None
                return self.recovery.mobile.check(scene,q,b,np.array([opening/2,-opening/2]))
            edge=[];seed=start;why='SAFE'
            for amount in np.linspace(0,self.policy['escape_distance_m'],self.policy['cartesian_waypoints']):
                target=E.copy();target[:3,3]+=direction*amount;q=self.model.ik(target,r.base,seed=seed,starts=1)
                if q is None:why='RETREAT_NO_IK';row['failure_distance_m']=float(amount);break
                ok,why,_=check(q,r.base)
                if not ok:break
                edge.append(q.tolist());seed=q
            if len(edge)!=self.policy['cartesian_waypoints']:row['status']=why;self.save();continue
            # Open fully only once the fingers are physically away from handle.
            ok=True
            for width in ([opening] if maximum else np.linspace(opening,.1,9)):
                ok,why,_=self.recovery.mobile.check(scene,seed,r.base,np.array([width/2,-width/2]))
                if not ok:break
            if not ok:row['status']='ESCAPE_OPENING_'+why;self.save();continue
            def home_check(q,b):
                if self.model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN',None
                return self.recovery.mobile.check(scene,q,b,np.array([.05,-.05]))
            if r.capture.config.get('maximum_range',{}).get('enabled'):
                row['status']='CLEARANCE_RETREAT_PREFLIGHT_PASSED';plans.append({'row':row,'opening':opening,'escape':edge,'home':None});self.save();continue
            outer=self
            class View:
                def __getattr__(self,name):return getattr(outer.model,name)
                def check(self,q,b):return home_check(q,b)
            path=Model.joint_plan(View(),seed,self.model.home,r.base,iterations=1500)
            if path is None:row['status']='NO_SAFE_HOME_PATH';self.save();continue
            row['status']='RETREAT_PREFLIGHT_PASSED';plans.append({'row':row,'opening':opening,'escape':edge,'home':path});self.save()
        if maximum:
            # Same twelve alternatives: six Cartesian, three checked base
            # backoffs and three joint-space escapes. Home is optional.
            opening=openings[0];plans.extend(self.base_clearance_plans(D,opening));home=np.asarray(self.model.home,float);start=np.asarray(start,float)
            goals=[start+.2*(home-start)]
            for sign in [-1.,1.]:
                q=start.copy();q[3]+=sign*.25;q[5]-=sign*.25;goals.append(q)
            deadline=min(r.deadline,time.time()+max(0,self.policy['planning_wall_s']-(time.monotonic()-started)))
            for index,goal in enumerate(goals):
                if len(self.rows) and time.time()>=deadline:break
                row={'index':len(self.rows),'kind':'joint-space-clearance','goal_q':goal.tolist(),'release_opening_m':opening,'base':list(r.base)};self.rows.append(row)
                if self.model.margin(goal)<=.05:row['status']='JOINT_ESCAPE_LOW_MARGIN';self.save();continue
                anchor=np.asarray(self.recovery.visual_current(D)['anchor_world_m'])
                if np.linalg.norm(self.model.poses(goal,r.base,finger_q=[opening/2,-opening/2])['tcp_link'][:3,3]-anchor)<self.policy['escape_distance_m']:
                    row['status']='JOINT_ESCAPE_NO_CLEARANCE';self.save();continue
                def joint_check(q,b):
                    if time.time()>=deadline:raise PlanningExhausted('JOINT_ESCAPE_WALL_BUDGET')
                    return self.recovery.mobile.check(scene,q,b,np.array([opening/2,-opening/2]))
                outer=self
                class JointView:
                    def __getattr__(self,name):return getattr(outer.model,name)
                    def check(self,q,b):return joint_check(q,b)
                try:path=Model.joint_plan(JointView(),start,goal,r.base,iterations=1500)
                except PlanningExhausted as error:row['status']=str(error);self.save();continue
                if path is None:row['status']='JOINT_ESCAPE_NO_PATH';self.save();continue
                for width in ([opening] if maximum else np.linspace(opening,.1,9)):
                    ok,why,_=self.recovery.mobile.check(scene,goal,r.base,[width/2,-width/2])
                    if not ok:break
                if not ok:row['status']='JOINT_ESCAPE_OPEN_'+why;self.save();continue
                row['status']='JOINT_CLEARANCE_PREFLIGHT_PASSED';plans.append({'row':row,'opening':opening,'escape':path,'home':None});self.save()
        return plans
    def base_clearance_plans(self,D,opening):
        r=self.r;normal=np.asarray(self.recovery.visual_current(D)['outward_normal_world'],float);normal[2]=0.
        if np.linalg.norm(normal)<1e-8:return []
        normal/=np.linalg.norm(normal);plans=[];q=r.arm_q();fingers=np.array([opening/2,-opening/2])
        from scipy.spatial.transform import Rotation
        for angle in (0.,-25.,25.):
            direction=Rotation.from_euler('z',angle,degrees=True).apply(normal)
            target=list(r.base);target[:2]=(np.asarray(target[:2])+self.policy['escape_distance_m']*direction[:2]).tolist()
            route=self.recovery.mobile.locked_route(list(r.base),target,q,fingers)
            row={'index':len(self.rows),'kind':'base-assisted-clearance','base':list(r.base),'target_base':target,'release_opening_m':opening,'route':route,'status':'BASE_CLEARANCE_PREFLIGHT_PASSED' if route['valid'] else 'BASE_CLEARANCE_NO_ROUTE'};self.rows.append(row);self.save()
            if route['valid']:plans.append({'row':row,'opening':opening,'escape':[],'home':None,'base_clearance':{'base':target,'route':route}})
        return plans
    def execute(self,plan):
        r=self.r;row=plan['row'];row['status']='EXECUTING';self.save()
        try:
            if plan.get('base_clearance'):
                self.recovery.move_base(plan['base_clearance'])
                row['status']='RETREAT_EXECUTED';self.save();return True
            r.execute_arm_path(plan['escape'],'SYSTEM_RETREAT_OUTWARD',minimum_duration=.15)
            if not r.capture.config.get('maximum_range',{}).get('enabled'):r.open_clear_gripper()
            # Maximum-range preserves the already unloaded release aperture.
            # Scan/base/regrasp planners check actual fingers, not a fictitious
            # required 100mm opening and another full slow closure.
            if plan['home'] is not None:r.execute_arm_path(plan['home'],'SYSTEM_RETREAT_HOME')
            row['status']='RETREAT_EXECUTED';self.save();return True
        except RuntimeError as error:
            self.recovery.released=False
            self.recovery.needs_safety_release=True
            events=getattr(r,'constraints',None)
            if events:events.dispatch(str(error),'retreat.execute','physical',candidate=row['index'])
            row['status']='PHYSICAL_PATH_STOP';row['reason']=str(error);self.save()
            # Halt and attempt only a checked return to the most recent safe arm
            # state. No state teleport and no disabling native contact reporting.
            r.halt_at_measured_state()
            if not r.return_last_safe():row['restore']='UNSAFE_TO_RECOVER';self.save();raise
            row['restore']='SAFE_RETURN_COMPLETED';self.save()
            raise PlanningExhausted('RETREAT_REPLAN_REQUIRED:'+str(error)) from error
