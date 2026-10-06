"""Finite release/escape alternatives; original collision and load guards stay on."""
import time,json
import numpy as np
from piper_mobile_demo.model import Model
from wrist_reconstruction.planner import PlanningExhausted

class RetreatPlanner:
    def __init__(self,recovery):self.recovery=recovery;self.r=recovery.r;self.model=recovery.model;self.policy=self.r.capture.config['retreat'];self.rows=[]
    def save(self): (self.r.capture.output/'retreat_alternatives.json').write_text(json.dumps(self.rows,indent=2))
    def plans(self,D):
        r=self.r;E=r.tcp();start=r.arm_q();normal=np.asarray(self.recovery.visual_current(D)['outward_normal_world']);back=-E[:3,2];up=np.array([0.,0,1]);lateral=E[:3,0]
        directions=[back,back+.5*up,normal,normal+.5*up,back+.25*lateral,back-.25*lateral]
        scene=self.recovery.mobile.scene(r.base);plans=[];started=time.monotonic()
        openings=[min(.1,float(np.diff(r.finger_q()[::-1])[0])+x) for x in self.policy['extra_openings_m']]
        for index,(direction,opening) in enumerate((d,o) for o in openings for d in directions):
            if index>=self.policy['maximum_alternatives'] or time.monotonic()-started>self.policy['planning_wall_s']:break
            direction=direction/np.linalg.norm(direction);row={'index':len(self.rows),'direction_world':direction.tolist(),'release_opening_m':opening,'base':list(r.base)};self.rows.append(row)
            def check(q,b):
                if self.model.margin(q)<=.05:return False,'LOW_JOINT_MARGIN',None
                return self.recovery.mobile.check(scene,q,b,np.array([opening/2,-opening/2]))
            edge=[];seed=start;why='SAFE'
            for amount in np.linspace(0,self.policy['escape_distance_m'],self.policy['cartesian_waypoints']):
                target=E.copy();target[:3,3]+=direction*amount;q=self.model.ik(target,r.base,seed=seed,starts=1)
                if q is None:why='RETREAT_NO_IK';break
                ok,why,_=check(q,r.base)
                if not ok:break
                edge.append(q.tolist());seed=q
            if len(edge)!=self.policy['cartesian_waypoints']:row['status']=why;self.save();continue
            # Open fully only once the fingers are physically away from handle.
            ok=True
            for width in np.linspace(opening,.1,9):
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
        return plans
    def execute(self,plan):
        r=self.r;row=plan['row'];row['status']='EXECUTING';self.save()
        try:
            r.execute_arm_path(plan['escape'],'SYSTEM_RETREAT_OUTWARD',minimum_duration=.15)
            r.open_clear_gripper()
            if plan['home'] is not None:r.execute_arm_path(plan['home'],'SYSTEM_RETREAT_HOME')
            row['status']='RETREAT_EXECUTED';self.save();return True
        except RuntimeError as error:
            row['status']='PHYSICAL_PATH_STOP';row['reason']=str(error);self.save()
            # Halt and attempt only a checked return to the most recent safe arm
            # state. No state teleport and no disabling native contact reporting.
            r.halt_at_measured_state()
            if not r.return_last_safe():row['restore']='UNSAFE_TO_RECOVER';self.save();raise
            row['restore']='SAFE_RETURN_COMPLETED';self.save();return False
