"""Small explicit operations; existing perception/planner and frozen G1 closure reused."""
import json, math, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .policy import SafetyStop
from .motion import progress_reference

class Operations:
    def __init__(self,runtime):
        self.r=runtime;self.g=runtime.g;self.cache={};self.acquisitions=0;self.panel_plan=None
        from native_g1_wrist_input import install_camera
        install_camera(self.g)
    def observe(self):
        from native_g1_agent_incremental_activation import observe
        with self.r.profile.measure('RGBD_SAM3_and_incremental_geometry'):return observe(self.g)
    def evaluate_view(self,target):
        from native_g1_hybrid_contact import evaluate_current_view
        with self.r.profile.measure('IK_and_view_planning'):return evaluate_current_view(self.g,target)
    def move_and_observe(self,evaluated):
        if not evaluated.get('feasible'):return dict(executed=False,reason='NO_VALIDATED_VIEW_ROUTE')
        if self.g['agent_moves']>=4:return dict(executed=False,reason='Observation budget used')
        self.r.move_path(evaluated['route']['q_path'][1:],'PREGRASP');self.r.hold(self.g['wrist_config']['settle_s'])
        self.g['agent_moves']+=1;self.g['wrist_routes'].append(evaluated['route']);return self.observe()
    def build_geometry(self):
        from native_g1_contact_recovery import build_geometry
        with self.r.profile.measure('grasp_ranking_and_IK'):result=build_geometry(self.g)
        data=result.pop('geometry_data',None)
        if data is not None:result['geometry_data_reference']=str(self.r.output/'agent_geometry/observed_geometry.json')
        return result
    def validate_execution(self):
        from native_g1_agent_execution import validate_execution
        return validate_execution(self.g)
    def reobserve(self):
        from native_g1_hybrid_contact import initial_observation
        self.g['observation_generations'].append(self.g['agent_observations']);self.g['agent_observations']=[];self.g['wrist_routes']=[]
        return initial_observation(self.g)
    def attempt(self,command):
        if self.acquisitions>=int(self.r.job.get('runtime_acquisition_budget',2)):
            return dict(executed=False,reason='Registered acquisition budget used')
        plan=json.loads((self.r.output/'visual_plan.json').read_text());family=[float(plan[k]) for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']]
        start=self.acquisitions;first=self.r.record.count;result=dict(family=family,segments=[],first_record=first,physical_grasp_started=False,short_success=False)
        self.g['recovery_state']='FINITE_CONTACT_OPERATION'
        try:
            result['acquisition']=self.acquire()
            for action in command['segments']:
                segment=self.interact(action['target_deg']);result['segments'].append(segment)
                if not segment['ok']:break
        except BaseException as error:
            result['error']=repr(error)
        finally:
            result['physical_grasp_started']=self.acquisitions>start
            result['last_record']=self.r.record.count
            # Keep the last commanded velocity for release_park's checked
            # deceleration. A recoverable result must not erase it first.
        # A true hard latch is authoritative. There is no loaded reset or
        # automatic retry-through. Ordinary acquisition/transmission failure
        # can still use the original checked jaw-only release and parking.
        if self.r.supervisor.latched is not None:
            result['safe_exit']=dict(safe=False,reason='HARD_STOP_NO_AUTOMATIC_CONTINUATION',stop=self.r.supervisor.latched)
        else:
            try:result['safe_exit']=self.release_park()
            except BaseException as error:result['safe_exit']=dict(safe=False,reason=repr(error))
        result['short_success']=bool(result.get('acquisition',{}).get('acquired') and any(s.get('ok') and s.get('gain_deg',0)>=2 for s in result['segments']))
        self.g['contact_attempts'].append(result);self.g['physical_grasps_started']=self.acquisitions
        return result
    def acquire(self):
        """Same closure update, preload mapping, verification and one-second hold."""
        from native_g1_agent_execution import execute
        from native_g1_wrist_input import refine
        execute(self.g,dict(operation='attempt_grasp_and_probe',source='explicit Agent approval'))
        # execute assigned current fresh visual candidate; no archived plan input.
        chosen=self.g['chosen'];T=np.asarray(chosen['T']);pre=T.copy();pre[:3,3]-=.04*T[:3,2]
        q0=np.asarray(self.r.robot.get_joint_positions())[self.r.arm];seed=self.r.solver.solve(pre,self.r.base,np.asarray(chosen['approach'][0]))
        if seed is None:raise RuntimeError('NO_CURRENT_PREGRASP_IK')
        self.r.move_path(chosen['preplan'][1:]+[seed],'PREGRASP');refine(self.g)
        T=np.asarray(self.g['T']);pre=T.copy();pre[:3,3]-=.04*T[:3,2];path=[]
        for f in np.linspace(0,1,21):
            X=pre.copy();X[:3,3]=(1-f)*pre[:3,3]+f*T[:3,3];seed=self.r.solver.solve(X,self.r.base,seed)
            if seed is None:raise RuntimeError('APPROACH_IK')
            path.append(seed.copy())
        self.r.move_path(path,'APPROACH')
        closure=self.r.Closure(self.r.tcp(),self.r.closure_policy);bias=self.r.qtarget[self.r.arm]-np.asarray(self.r.robot.get_joint_positions())[self.r.arm]
        self.r.reference_drive=None;self.r.begin_reference();closure_origin=self.r.tcp().copy();virtual_origin=self.r.reference_drive.pose.copy()
        preload_origin=None;self.acquisitions+=1;self.r.event('PHYSICAL_GRASP_START',index=self.acquisitions)
        for _ in range(int(100/self.r.dt)):
            if not self.r.rows:self.r.step()
            row=self.r.rows[-1];q=np.asarray(row['q']);P=self.r.model.poses(q[self.r.arm],self.r.base,finger_q=q[self.r.fingers])
            centers=[(P[n]@np.r_[self.r.model.pads[n].mean(0),1])[:3] for n in self.g['FINGERS']]
            cmd=closure.update(self.r.tcp(),row['aperture_m'],self.r.filtered(),centers,self.r.dt);self.g['phase']=cmd['state']
            if cmd['mode']=='position':self.r.jaw_width=float(cmd['opening_m'])
            else:
                if preload_origin is None:preload_origin=float(q[self.r.controller.master])
                angle=preload_origin+cmd['closing_effort_n']*abs(self.r.controller.geometry.width_derivative(preload_origin))/self.r.c['gripper_kp_nm_rad']
                self.r.jaw_width=self.r.controller.geometry.width(float(np.clip(angle,0,.85)))
            mapped=cmd['T']@np.linalg.inv(closure_origin)@virtual_origin
            self.r.cartesian_step(mapped,cmd['state'])
            w=list(self.r.rows)[-int(.5/self.r.dt):];loads=np.array([list(s['forces_n'].values()) for s in w]);ap=np.array([s['aperture_m'] for s in w])
            good=len(w)>=int(.45/self.r.dt) and np.all(np.mean(loads>.05,axis=0)>=.6) and np.all(np.mean(loads,axis=0)>=.1) and np.ptp(ap)<.0005
            if closure.state=='FORCE_HOLD' and good:break
        else:raise RuntimeError('BILATERAL_HOLD_NOT_ESTABLISHED')
        self.r.grasp_reference=np.linalg.inv(self.r.moving())@self.r.tcp();self.g['reference']=self.r.grasp_reference
        self.r.event('GRASP_VERIFIED',angle_deg=math.degrees(self.r.actual()),pad_loads_n=self.r.filtered().tolist())
        self.g['phase']='FORCE_HOLD';self.r.hold(1.);self.r.reference_drive=None
        return dict(acquired=True,hold_s=1.,angle_deg=math.degrees(self.r.actual()))
    def interact(self,target_deg,max_sim_s=45):
        from native_g1_ft_identification import kinematic_prior
        prior=kinematic_prior(self.g);axis=np.asarray(prior['axis_world']);origin=np.asarray(prior['origin_world_m']);start=self.r.actual();target=math.radians(target_deg);direction=1 if target>=start else -1
        T=self.r.tcp();angle0=self.r.actual();reference=angle0;self.r.reference_drive=None;began=self.r.tick*self.r.dt;checkpoint=began;last_angle=angle0;last_relative=0.;stalls=0
        while direction*(target-self.r.actual())>0:
            theta=self.r.actual();radius=max(.001,float(np.linalg.norm(self.r.tcp()[:3,3]-origin)));motion=self.r.motion_limits('LOADED')
            reference=progress_reference(theta,reference,target,direction,motion['speed_m_s'],radius,self.r.dt,math.radians(.5))
            R=Rotation.from_rotvec(axis*(reference-angle0)).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin
            self.g['current_articulation_reference']=reference
            self.r.cartesian_step(D@T,'OPEN_CONTINUE');now=self.r.tick*self.r.dt
            if now-checkpoint>=.5:
                drift=self.r.rows[-1]['relative_translation_slip_m'];progress=direction*(self.r.actual()-last_angle)*radius
                stalls=stalls+1 if drift-last_relative>max(0.,progress) else 0
                checkpoint=now;last_angle=self.r.actual();last_relative=drift
                if stalls>=3:return dict(ok=False,reason='CONTACT_TRANSMISSION_STALL',angle_deg=math.degrees(self.r.actual()))
            if now-began>=max_sim_s:return dict(ok=False,reason='BOUNDED_SEGMENT_COMPLETE_BEFORE_TARGET',angle_deg=math.degrees(self.r.actual()))
        self.r.stop_command();self.g['qvelocity']=np.zeros(7);self.r.reference_drive=None;return dict(ok=True,angle_deg=math.degrees(self.r.actual()),gain_deg=math.degrees(self.r.actual()-start))
    def release_park(self):
        if self.g.get('at_safe_park') and self.g.get('last_exit_plan'):
            from native_g1_safe_park import verify_park
            current=verify_park(self.g,self.g['last_exit_plan'])
            if current['reached']:return dict(safe=True,already_verified_parked=True,park=current)
        from native_g1_contact_recovery import release_geometry
        from native_g1_safe_park import plan_exit,verify_park
        audit=release_geometry(self.g,self.r.rows[-1]['aperture_m'])
        if not audit['valid']:return dict(safe=False,reason='RELEASE_SWEEP:'+str(audit))
        self.g['phase']='RELEASE';self.r.stop_command();self.r.grasp_reference=None;self.g['reference']=None;self.r.reference_drive=None;self.g['qvelocity']=np.zeros(7)
        width=self.r.rows[-1]['aperture_m'];steps=max(2,int(math.ceil((.084-width)/(self.r.policy.data['jaw_release_m_s']*self.r.dt))))
        for w in np.linspace(width,.084,steps+1):self.r.jaw_width=float(w);self.r.step()
        self.r.hold(.3)
        if max(self.r.filtered())>.05:self.r.hold(1.)
        if max(self.r.filtered())>.05 or not self.r.supervisor.force.settled():return dict(safe=False,reason='UNLOAD_NOT_CONFIRMED')
        self.r.event('RELEASED');plan=plan_exit(self.g)
        if not plan['valid']:return dict(safe=False,reason='SAFE_EXIT_UNAVAILABLE',plan=plan)
        self.r.move_path(plan['clearance_path'][1:],'CLEARANCE_RETREAT');self.r.event('CLEARANCE_REACHED')
        # Current measured door/state recheck before returning, with original checker.
        from native_g1_safe_park import route_to_joint
        measured=np.asarray(self.r.robot.get_joint_positions())[self.r.arm];path,reason=route_to_joint(self.g,measured,plan['parking_q'])
        if path is None:return dict(safe=False,reason=reason)
        self.r.move_path(path[1:],'SAFE_PARK_MOTION');self.r.hold(.3);verified=verify_park(self.g,plan)
        if verified['reached']:self.r.event('SAFE_PARK_REACHED');self.g['recovery_state']='UNLOADED_OBSERVATION';self.g['at_safe_park']=True;self.g['last_exit_plan']=plan
        return dict(safe=verified['reached'],park=verified,plan=plan)
    def plan_push(self,command):
        from native_g1_hybrid_contact import panel_plan,PanelScene
        if not isinstance(self.r.collision,PanelScene):
            raw={}
            from xarm7_native_planning import PAD_FRAMES,FINGER_BODIES,LOGICAL
            for n,l in zip(FINGER_BODIES,LOGICAL):
                m=self.r.model.native.meshes[n].copy();m.apply_transform(np.linalg.inv(PAD_FRAMES[n]));raw[l]=m
            self.r.collision=PanelScene(self.r.collision,raw);self.g['collision']=self.r.collision
        self.panel_plan=panel_plan(self.g,command);return self.panel_plan
    def push(self):
        p=self.panel_plan
        if not p or not p.get('valid'):return dict(executed=False,reason='NO_VALIDATED_OBSERVED_PANEL_PLAN')
        self.r.grasp_reference=None;self.g['reference']=None;self.g['phase']='PREGRASP';width=self.r.jaw_width;compact=p['compact_aperture_m']
        steps=max(2,int(math.ceil(abs(width-compact)/(self.r.policy.data['jaw_free_m_s']*self.r.dt))))
        for w in np.linspace(width,compact,steps+1):self.r.jaw_width=float(w);self.r.step()
        self.r.move_path(p['route'][1:],'PREGRASP');self.r.collision.paths=set(p['target_paths']);self.r.collision.active=True
        start=self.r.actual();T=np.asarray(p['T_contact']);N=np.asarray(p['normal_world']);prior=p['prior'];axis=np.asarray(prior['axis_world']);origin=np.asarray(prior['origin_world_m']);self.r.reference_drive=None
        contact_count=0;began=self.r.tick*self.r.dt;reference=start;radius=np.linalg.norm(np.cross(T[:3,3]-origin,axis))
        try:
            # The complete observed entry IK path was checked by panel_plan;
            # execute it with one continuous bounded joint-space time law.
            self.r.move_path(p['entry_path'],'PANEL_APPROACH')
            self.r.reference_drive=None
            while self.r.actual()>math.radians(p['target_deg']):
                theta=self.r.actual();reference=progress_reference(theta,reference,math.radians(p['target_deg']),-1,self.r.motion_limits('LOADED')['speed_m_s'],radius,self.r.dt);R=Rotation.from_rotvec(axis*(reference-start)).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin
                self.g['current_articulation_reference']=reference
                self.r.cartesian_step(D@T,'PANEL_PUSH');contact_count+=sum(c['force_n']>0 and c['target'] in self.r.collision.paths for c in self.r.rows[-1]['contacts'])
                if self.r.tick*self.r.dt-began>30:break
            # Normal completion must decelerate under the still-authorized
            # panel contact mode before the separation trajectory starts.
            # Exceptions bypass this continuation and remain authoritative.
            self.r.stop_command()
        finally:
            self.g['qvelocity']=np.zeros(7);self.r.reference_drive=None
        result=dict(panel_contacts=contact_count,closing_deg=math.degrees(start-self.r.actual()),angle_deg=math.degrees(self.r.actual()))
        # Full current-state separation is validated, never replayed blindly.
        retreat=self.r.tcp().copy();retreat[:3,3]+=.04*N;q=np.asarray(self.r.robot.get_joint_positions())[self.r.arm];path=[]
        for f in np.linspace(0,1,21):
            X=self.r.tcp().copy();X[:3,3]+=f*.04*N;q=self.r.solver.solve(X,self.r.base,q)
            if q is None:raise RuntimeError('PANEL_SEPARATION_IK')
            path.append(q.copy())
        self.r.move_path(path,'PANEL_SEPARATION');self.r.collision.active=False
        result['safe_exit']=self.release_park();return result
