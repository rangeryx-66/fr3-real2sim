"""Small explicit operations; existing perception/planner and frozen G1 closure reused."""
import json, math, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .policy import SafetyStop
from .segments import ExecutionBudgetExhausted

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
    def action_target(self,action):return action['target_deg']
    def measured_coordinate_fields(self):return dict(angle_deg=math.degrees(self.r.actual()))
    def attempt(self,command):
        if self.acquisitions>=int(self.r.job.get('runtime_acquisition_budget',2)):
            return dict(executed=False,reason='Registered acquisition budget used')
        plan=json.loads((self.r.output/'visual_plan.json').read_text());family=[float(plan[k]) for k in ['section_fraction','insertion_m','roll_deg','pitch_deg']]
        start=self.acquisitions;first=self.r.record.count;result=dict(family=family,segments=[],first_record=first,physical_grasp_started=False,short_success=False)
        self.g['recovery_state']='FINITE_CONTACT_OPERATION'
        try:
            result['acquisition']=self.acquire()
            for ix,action in enumerate(command['segments']):
                if ix:
                    from .segments import continuation_allowed
                    allowed,why=continuation_allowed(result['segments'][-1],action,self.r.supervisor.latched,self.r.episode_budget.remaining(self.r.tick*self.r.dt))
                    route=self.check_successor(action) if allowed else dict(valid=False,reason=why)
                    result.setdefault('continuations',[]).append(route)
                    if not route['valid']:break
                segment=self.interact(self.action_target(action),action=action);result['segments'].append(segment)
                if segment['status'] not in ('TARGET_REACHED','SAFE_PARTIAL_PROGRESS'):break
        except BaseException as error:
            result['error']=repr(error)
            if isinstance(error,ExecutionBudgetExhausted):result['operation_status']='BUDGET_EXHAUSTED'
            if isinstance(error,SafetyStop):
                result['operation_status']='SAFETY_STOP';result['segments'].append(dict(ok=False,status='SAFETY_STOP',reason=error.reason,evidence=error.evidence))
        finally:
            result['physical_grasp_started']=self.acquisitions>start
            result['last_record']=self.r.record.count
            # Keep the last commanded velocity for release_park's checked
            # deceleration. A recoverable result must not erase it first.
        # A true hard latch is authoritative. There is no loaded reset or
        # automatic retry-through. Ordinary acquisition/transmission failure
        # can still use the original checked jaw-only release and parking.
        if self.r.supervisor.latched is not None or result.get('operation_status')=='SAFETY_STOP':
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
        self.r.event('GRASP_VERIFIED',**self.measured_coordinate_fields(),pad_loads_n=self.r.filtered().tolist())
        self.g['phase']='FORCE_HOLD';self.r.hold(1.);self.r.reference_drive=None
        return dict(acquired=True,hold_s=1.,**self.measured_coordinate_fields())
    def check_successor(self,action):
        """Validate the next complete arc from measured state, never an old path."""
        from native_g1_ft_identification import kinematic_prior
        p=kinematic_prior(self.g);axis=np.asarray(p['axis_world']);axis=axis/np.linalg.norm(axis);origin=np.asarray(p['origin_world_m'])
        start=self.r.actual();T=self.r.tcp().copy();q=np.asarray(self.r.robot.get_joint_positions())[self.r.arm]
        for angle in np.linspace(0,math.radians(action['target_deg'])-start,21):
            R=Rotation.from_rotvec(axis*angle).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin
            seed=self.r.solver.solve(D@T,self.r.base,q)
            if seed is None:return dict(valid=False,reason='SUCCESSOR_CURRENT_STATE_IK',actual_start_deg=math.degrees(start))
            # Predict current moving-body configuration using the permitted hinge prior.
            if self.r.model.margin(seed)<=self.r.policy.data['joint_margin_rad']:return dict(valid=False,reason='JOINT_MARGIN')
            moving=D@self.r.moving();ok,why=self.r.collision.check(self.r.model.poses(seed,self.r.base,finger_q=np.asarray(self.r.robot.get_joint_positions())[self.r.fingers]),moving,True)
            if not ok:return dict(valid=False,reason=why,actual_start_deg=math.degrees(start))
            q=seed
        return dict(valid=True,actual_start_deg=math.degrees(start),target_deg=action['target_deg'],measured_state_route=True)
    def interact(self,target_deg,max_sim_s=None,action=None):
        try:return self.rotate_segment(target_deg,max_sim_s,action,phase='OPEN_CONTINUE')
        except ExecutionBudgetExhausted as error:
            self.r.stop_command()
            return dict(ok=False,status='BUDGET_EXHAUSTED',reason=str(error),safely_stopped=True,physical_transmission_valid=False)
    def rotate_segment(self,target_deg,max_sim_s=None,action=None,phase='OPEN_CONTINUE',contact_pose=None,prior=None,measure=None,articulation=None,target_q=None):
        from native_g1_ft_identification import kinematic_prior
        from .progress import ProgressClock,hinge_radius
        from .segments import segment_plan,TransmissionMonitor,slip_stall_confirmed,classify_outcome,TRANSMISSION_REASONS
        action=dict(action or {})
        if max_sim_s is not None:action.setdefault('max_sim_s',max_sim_s)
        prior=prior or kinematic_prior(self.g);measure=measure or self.r.actual
        axis=np.asarray(prior['axis_world']);axis/=np.linalg.norm(axis);origin=np.asarray(prior['origin_world_m'])
        start=measure();requested=math.radians(target_deg) if target_q is None else float(target_q);T=self.r.tcp().copy() if contact_pose is None else np.asarray(contact_pose).copy()
        linear=articulation is not None and articulation.joint_type=='prismatic'
        radius=1. if linear else hinge_radius(T[:3,3],origin,axis)
        motion_plan=dict(self.r.policy.motion('LOADED'))
        # The finite rail lead is one nominal loaded-speed stopping distance
        # plus one physics step. This is a reference allowance, not a new
        # collision/force/contact acceptance threshold.
        lookahead=(motion_plan['speed_m_s']**2/(2*motion_plan['acceleration_m_s2'])+motion_plan['speed_m_s']*self.r.dt) if linear else math.radians(.5)
        if linear:motion_plan.update(angular_speed_rad_s=motion_plan['speed_m_s'],angular_acceleration_rad_s2=motion_plan['acceleration_m_s2'])
        planning=segment_plan(start,requested,action,motion_plan,radius,self.r.dt,self.r.episode_budget.remaining(self.r.tick*self.r.dt),lookahead=lookahead)
        if not planning['valid']:return dict(ok=False,status='BUDGET_EXHAUSTED',reason=planning['reason'],planning=planning,**self.measured_coordinate_fields())
        target=requested;clock=ProgressClock(start,target,lookahead=lookahead);monitor=TransmissionMonitor(start,clock.direction,action,lookahead=lookahead);began=self.r.tick*self.r.dt;checkpoint=began;wall_began=time.monotonic()
        startup=self.prismatic_startup(action,lookahead) if linear else None
        if startup is not None:
            planning['bounded_preload_max_s']=startup.max_loading_s
            planning['preload_is_FIT_data']=False
        self.r.reference_drive=None;last_angle=start;last_relative=self.r.rows[-1]['relative_translation_slip_m'];stalls=0;reason=None;panel_contacts=0;eta=None;eta_exceeds=False
        rate=min(self.r.policy.motion('LOADED')['speed_m_s'],action.get('reference_speed_m_s',self.r.policy.motion('LOADED')['speed_m_s']))/max(radius,1e-9)
        horizon=min(planning['max_sim_s'],planning['nominal_duration_s']+clock.lookahead/rate+2*planning['profile_cycle_s']+planning['stop_reserve_s']);extensions=0
        if startup is not None:horizon=min(planning['max_sim_s'],horizon+startup.max_loading_s)
        while True:
            if clock.direction*(target-measure())<=0:
                self.r.stop_command();self.r.reference_drive=None;clock.velocity=0.
                if clock.direction*(target-measure())<=0:break
            now=self.r.tick*self.r.dt;elapsed=now-began;resources=self.r.episode_budget.remaining(now)
            if elapsed>=planning['max_sim_s']-planning['stop_reserve_s'] or resources['wall_s']<=0 or resources['sim_s']<=planning['stop_reserve_s']:
                reason='FINITE_EXECUTION_BUDGET';break
            # Forecast only from a completed measured window; no mass/friction truth.
            if monitor.stable() and monitor.last['progress_rad']>0:
                eta=clock.direction*(target-measure())/(monitor.last['progress_rad']/monitor.last['window_s'])+planning['stop_reserve_s']
                wall_per_sim=(time.monotonic()-wall_began)/max(elapsed,self.r.dt)
                eta_exceeds=eta>resources['sim_s'] or eta*wall_per_sim>resources['wall_s']
                # ETA is uncertain during initial compliance/stiction. It is
                # advisory, never an early stop based on one low-speed window.
            if elapsed>=horizon-planning['stop_reserve_s']:
                if monitor.stable() and monitor.last['progress_rad']>0:
                    horizon=min(planning['max_sim_s'],elapsed+max(planning['profile_cycle_s'],eta or planning['profile_cycle_s'])+planning['stop_reserve_s']);extensions+=1
                else:reason='ADAPTIVE_HORIZON_WITHOUT_HEALTHY_PROGRESS';break
            motion=self.r.motion_limits('LOADED')
            feedback=measure();rail_feedback=None
            if startup is not None:
                feedback,rail_feedback=startup.clock_feedback(self.startup_sample(self.r.rows[-1],elapsed))
                # Feedback and endpoint share the same bounded servo-offset
                # coordinate. Physical completion still tests target-measure().
                clock.target=target+clock.direction*rail_feedback['tracking_offset_m']
            reference,state=clock.advance(feedback,motion['speed_m_s'],1. if linear else hinge_radius(self.r.tcp()[:3,3],origin,axis),self.r.dt,motion['speed_m_s'] if linear else motion['angular_speed_rad_s'],motion['acceleration_m_s2'],motion['acceleration_m_s2'] if linear else motion['angular_acceleration_rad_s2'])
            if rail_feedback is not None:state.update(rail_feedback,actual_object_lead_m=clock.direction*(reference-measure()))
            if articulation is not None:D=articulation.delta(reference-start)
            else:
                R=Rotation.from_rotvec(axis*(reference-start)).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin
            step_phase=startup.phase if startup is not None else phase
            self.g['current_articulation_reference']=reference;self.r.cartesian_step(D@T,step_phase);now=self.r.tick*self.r.dt
            if phase=='PANEL_PUSH':panel_contacts+=sum(c['force_n']>0 and c['target'] in self.r.collision.paths for c in self.r.rows[-1]['contacts'])
            if startup is not None:
                was_sliding=startup.sliding;observed=startup.observe(self.startup_sample(self.r.rows[-1],now-began))
                if observed is not None and observed['classification']!=getattr(startup,'reported_classification',None):
                    self.r.event('PRISMATIC_CONTACT_STATE',classification=observed['classification'],phase_next=observed['next_phase'],actual_q_m=measure())
                    startup.reported_classification=observed['classification']
                if startup.stop_reason:reason=startup.stop_reason;break
                if not was_sliding and startup.sliding:
                    self.r.prismatic_preloading=False;self.r.profile_started=self.r.tick
                    monitor=TransmissionMonitor(measure(),clock.direction,action,lookahead=lookahead)
                    checkpoint=now;last_angle=measure();last_relative=self.r.rows[-1]['relative_translation_slip_m']
                    self.r.event('PRISMATIC_BREAKAWAY_CONFIRMED',actual_q_m=measure(),grasp_reference_unchanged=True)
                if not startup.sliding:continue
            if monitor.update(now-began,measure(),state,self.r.dt,clock.direction*(target-measure())):
                reason='SUSTAINED_LOW_TRANSMISSION';break
            # Original increasing-slip detector is retained as an independent signal.
            if not linear and phase=='OPEN_CONTINUE' and now-checkpoint>=.5:
                drift=self.r.rows[-1]['relative_translation_slip_m'];progress=clock.direction*(measure()-last_angle)*radius
                stalls=stalls+1 if drift-last_relative>max(0.,progress) else 0
                checkpoint=now;last_angle=measure();last_relative=drift
                if slip_stall_confirmed(stalls,state['lag_rad'],clock.lookahead):
                    reason='CONTACT_TRANSMISSION_STALL';break
        valid_until=self.r.tick*self.r.dt-began;self.r.stop_command();self.r.reference_drive=None
        # Reuse the existing terminal stationary cell/duration. This is a
        # completion observation, not a new hard protection or contact veto.
        from native_g1_safe_park import PARK_DEFINITION
        stopped_begin=np.asarray(self.r.robot.get_joint_positions())[self.r.arm].copy()
        self.r.hold(PARK_DEFINITION['stationary_hold_s'])
        stopped_q=np.asarray(self.r.robot.get_joint_positions())[self.r.arm]
        measured_stop=bool(np.max(abs(stopped_q-stopped_begin))<=PARK_DEFINITION['measured_joint_cell_rad'] and np.max(abs(stopped_q-self.r.qtarget[self.r.arm]))<=PARK_DEFINITION['measured_joint_cell_rad'])
        reached=clock.direction*(target-measure())<=0;failed=reason in TRANSMISSION_REASONS or bool(reason and reason.startswith('PRISMATIC_'));ok=reached and not failed and measured_stop
        stable=measured_stop and not failed and (ok or monitor.stable());gain=clock.direction*(measure()-start)
        status=classify_outcome(reached,measured_stop,reason,stable,gain)
        if failed:status='TRANSMISSION_FAILURE'
        # Interrupted FIT ends BEFORE the first suspect full window, not at stop.
        fit_end=min(valid_until,max(0.,monitor.failure['first_suspect_s'])) if monitor.failure else (monitor.valid_until if failed else valid_until)
        if startup is not None and failed:fit_end=min(fit_end,startup.first_suspect_s if startup.first_suspect_s is not None else 0.)
        coordinate_result=dict(angle_deg=math.degrees(measure()),gain_deg=math.degrees(measure()-start),reference_deg=math.degrees(clock.reference),final_tracking_lag_deg=math.degrees(clock.direction*(clock.reference-measure()))) if articulation is None else dict(joint_coordinate=float(measure()),gain_native=float(measure()-start),units=articulation.units)
        return dict(ok=bool(ok),status=status,reason=None if ok else reason,**coordinate_result,requested_target_reached=bool(reached),planning=planning,
            panel_contacts=panel_contacts,safely_stopped=measured_stop,physical_transmission_valid=stable,fit_prefix_transmission_valid=fit_end>0 and monitor.last is not None,fit_valid_until_s=fit_end,
            transmission=monitor.last,transmission_failure=monitor.failure,remaining_ETA_s=eta,ETA_exceeds_resources_advisory=eta_exceeds,adaptive_horizon_s=horizon,adaptive_extensions=extensions,elapsed_sim_s=self.r.tick*self.r.dt-began,
            slip_exceeds_progress_windows=stalls,stopped_state=dict(command_velocity_max_rad_s=float(np.max(abs(self.g['qvelocity']))),measured_joint_velocity_max_rad_s=float(np.max(abs(np.asarray(self.r.rows[-1]['joint_velocities'])[self.r.arm]))),measured_TCP_speed_m_s=self.r.rows[-1]['tcp_speed_m_s'],measured_coordinate_velocity_native=self.r.rows[-1]['joint_coordinate_velocity']),
            reference_native=float(clock.reference),coordinate_unit='m' if linear else 'rad',tracking_limited_fraction=clock.limited_steps/max(1,round(valid_until/self.r.dt)),final_tracking_lag_native=float(clock.direction*(clock.reference-measure())))
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
            segment=self.rotate_segment(p['target_deg'],None,getattr(self.r,'probe_profile',None),phase='PANEL_PUSH',contact_pose=self.r.tcp().copy())
            if hasattr(self.r.record,'end'):self.r.record.end(segment)
            contact_count=segment.get('panel_contacts',0)
        except ExecutionBudgetExhausted as error:
            self.r.stop_command();segment=dict(ok=False,status='BUDGET_EXHAUSTED',reason=str(error),safely_stopped=True,physical_transmission_valid=False)
            if hasattr(self.r.record,'end'):self.r.record.end(segment)
        finally:
            self.r.reference_drive=None
        result=dict(segment,panel_contacts=contact_count,closing_deg=math.degrees(start-self.r.actual()),angle_deg=math.degrees(self.r.actual()))
        # Full current-state separation is validated, never replayed blindly.
        retreat=self.r.tcp().copy();retreat[:3,3]+=.04*N;q=np.asarray(self.r.robot.get_joint_positions())[self.r.arm];path=[]
        for f in np.linspace(0,1,21):
            X=self.r.tcp().copy();X[:3,3]+=f*.04*N;q=self.r.solver.solve(X,self.r.base,q)
            if q is None:raise RuntimeError('PANEL_SEPARATION_IK')
            path.append(q.copy())
        self.r.move_path(path,'PANEL_SEPARATION');self.r.collision.active=False
        result['safe_exit']=self.release_park();return result
