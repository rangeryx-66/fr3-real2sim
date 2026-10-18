"""Explicit single-owner physics loop. No legacy step, force guard or owner wrapper executes."""
from collections import deque
import importlib.util
import json, math, os, signal, time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from .policy import Policy, SafetySupervisor, SafetyStop
from .evidence import Profile, Recorder, RenderSchedule
from .motion import timed_path, CartesianReference

class Runtime:
    def __init__(self, job, mode='FAST_DEV', policy_path=None, record_factory=Recorder):
        from .segments import EpisodeBudget
        self.episode_budget=EpisodeBudget(job);self.budget_exit_only=False;self._decelerating=False
        self.job=job;self.output=Path(job['output']);self.output.mkdir(parents=True,exist_ok=True)
        self.root=Path(__file__).resolve().parents[1];self.profile=Profile();self.mode=mode;self.stopped=False
        self.policy=Policy.load(policy_path or self.root/'configs/interaction_runtime_v1.json')
        self.job.get('wrist_experiment',{}).pop('force_policy',None)
        self.events=[];self.issues=[];self.tick=0;self.phase='SETTLE';self.previous=None;self.pad_presence={};self.filtered_loads=deque(maxlen=int(.08*240));self.rows=deque(maxlen=2400);self.last_command=None
        self.record=record_factory(self.output/'telemetry',self.policy.data['modes'][mode]['chunk_records'])
        self.render=RenderSchedule(mode,self.policy.data['modes'][mode]);self.video=None;self.video_index=[]
        from xarm_wrist_ft import prepare
        c=prepare(self.output/'sensor_assembly')
        with self.profile.measure('initialization'):
            from .native_scene import build
            self.scene=build(job,c)
            self.app=self.scene['app'];self.world=self.scene['world'];self.robot=self.scene['robot'];self.controller=self.scene['controller'];self.dt=self.scene['DT']
            from xarm7_native_planning import PAD_FRAMES
            from .kinematics import FastNativeModel
            from cross_robot_adapter import RobotPinkIK
            from xarm7_native_execution import collision_scene, native_reports
            from piper_mobile_demo.cooked_geometry import export_cooked
            from revolute10_fixed import authored_handle_colliders
            from xarm_wrist_ft import WristFT,tool_properties
            self.model=FastNativeModel(None,Path(job['asset_root']),json.loads((Path(job['source'])/'report.json').read_text()));self.solver=RobotPinkIK(c);self.model.ik=self.solver.solve
            self.base=np.asarray(json.loads((Path(job['source'])/'report.json').read_text())['robot_base_pose'])
            export_cooked(self.scene['stage'],self.output/'cooked_initial.json');export=json.loads((self.output/'cooked_initial.json').read_text())
            allowed=authored_handle_colliders(export,self.model.manifest['interaction_geometry']['selection'])
            self.collision=collision_scene(export,self.model,allowed);self.export=export;self.allowed=allowed
            self.native=native_reports(self.scene['stage'],export,allowed,self.world,self.dt,None)
            self.arm=np.asarray(self.scene['arm']);self.fingers=np.asarray(self.scene['fingers']);self.names=self.robot.dof_names
            self.actual_views=dict(self.scene['scene_monitor_views']);self.pad_frames=PAD_FRAMES
            chain=self.model.chain;self.tcp_offset=np.linalg.inv(chain.root_to_link('xarm_gripper_base_link',{}))@chain.root_to_link('tcp_link',{})
            self.sensor=WristFT(self.robot);self.tool_properties=tool_properties(c['sim_urdf']);self.c=c
            from isaacsim.core.prims import RigidPrim
            paths=[v.prim_paths[0] for v in self.actual_views.values()]
            self.pose_view=RigidPrim(prim_paths_expr=paths,name='runtime_measured_pose_batch',reset_xform_properties=False)
            self.pose_view.initialize();self.pose_names=[p.rsplit('/',1)[-1] for p in self.pose_view.prim_paths];self.pose_cache={};self.engineering_before_physics=None;self.engineering_load=None
            self.qtarget=np.asarray(self.robot.get_joint_positions()).copy();self.qvelocity=np.zeros(7);self.jaw_width=.084
            self.supervisor=SafetySupervisor(self.policy,self.model.limits,np.asarray(c['arm_speed_limits']),c['arm_effort_limits'])
            self.g=self.context();self.grasp_reference=None;self.loss_s=0.;self.reference_drive=None;self.reference_bias=None;self.seed=None
            active=Path(job['active_closure_source']);expected=job['expected_active_closure_sha256']
            import hashlib
            if hashlib.sha256(active.read_bytes()).hexdigest()!=expected:raise ValueError('Closure source mismatch before motion')
            spec=importlib.util.spec_from_file_location('frozen_G1_closure_runtime_v1',active);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            self.Closure=module.JawCenteredClosure
            from functools import lru_cache
            self.controller.geometry.width=lru_cache(maxsize=4096)(self.controller.geometry.width)
            self.controller.geometry.width_derivative=lru_cache(maxsize=4096)(self.controller.geometry.width_derivative)
            self.closure_policy=json.loads((self.root/'config/semantic_interaction.json').read_text());self.closure_policy.pop('max_pad_load_n',None)
            self.closure_policy['slow_closure_m_s']=self.policy.data['jaw_closure_m_s']
            self.g.update(policy=self.closure_policy,FixedClosure=self.Closure,JawCenteredClosure=self.Closure,_fixed_update=self.Closure.update)
        self.output.joinpath('EFFECTIVE_RUNTIME.json').write_text(json.dumps(dict(policy=self.policy.data,policy_sha256=self.policy.sha256,mode=mode,
            native_joint_order=self.names,arm_indices=self.arm.tolist(),finger_indices=self.fingers.tolist(),physics_dt=self.dt,
            no_legacy_owners=True,gripper_config=c,closure_sha256=expected,FT_primary='native fixed-interface transmitted wrench',
            optional_motor_force='unavailable; projected reactions are not drive torque',raw_private=True),indent=2))
        signal.signal(signal.SIGTERM,self.request_stop);signal.signal(signal.SIGINT,self.request_stop)
    def ensure_execution_budget(self,phase):
        from .segments import ExecutionBudgetExhausted
        if self.supervisor.latched is not None:raise SafetyStop('LATCHED_'+self.supervisor.latched['reason'],self.supervisor.latched)
        # Resource exhaustion forbids new work, never the existing checked exit.
        if self._decelerating or phase in ('RELEASE','CLEARANCE_RETREAT','PANEL_SEPARATION','SAFE_PARK_MOTION','PARK'):return
        remaining=self.episode_budget.remaining(self.tick*self.dt)
        if self.budget_exit_only or remaining['wall_s']<=0 or remaining['sim_s']<=0:
            if not self.budget_exit_only:self.event('BUDGET_EXIT_REQUESTED',remaining=remaining)
            self.budget_exit_only=True;raise ExecutionBudgetExhausted('EXIT_RESERVE_REACHED')
    def request_stop(self,*unused):self.stopped=True
    @staticmethod
    def matrix(p,q):
        T=np.eye(4);T[:3,3]=p;T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();return T
    def pose(self,name):
        if getattr(self,'pose_cache',None) and name in self.pose_cache:return self.pose_cache[name]
        view=self.actual_views[name];p,q=view.get_world_poses();return self.matrix(p[0],q[0])
    def tcp(self):
        return self.pose('xarm_gripper_base_link')@self.tcp_offset
    def moving(self):
        p,q=self.scene['door_contacts'].get_world_poses();return self.matrix(p[0],q[0])
    def actual(self):return float(self.scene['articulation'].get_joint_positions()[self.scene.get('selected_asset_dof',0)])
    def context(self):
        from articulated_interaction.physical_baseline import FINGERS
        from pathlib import Path
        g=dict(ROOT=self.root,a=SimpleNamespace(output=self.output,source=Path(self.job['source']),asset_root=Path(self.job['asset_root'])),job=self.job,spec=self.job,
            world=self.world,app=self.app,robot=self.robot,stage=self.scene['stage'],scene=self.scene,model=self.model,solver=self.solver,base=self.base,
            arm=self.arm,fingers=self.fingers,names=self.names,controller=self.controller,xarm_config=self.c,actual_views=self.actual_views,
            collision=self.collision,allowed=self.allowed,moving_initial=self.collision.moving_reference,dt=self.dt,tick=self.tick,qtarget=self.qtarget,qvelocity=self.qvelocity,rows=self.rows,
            matrix=self.matrix,tcp=self.tcp,moving=self.moving,actual=self.actual,step=self.step,move=self.compatible_move,hold=self.hold,
            ik=lambda T,q:self.solver.solve(T,self.base,q),poses=lambda q:self.model.poses(np.asarray(q)[self.arm],self.base,finger_q=np.asarray(q)[self.fingers]),
            filtered=self.filtered,FINGERS=FINGERS,export=self.export,vel=np.asarray(self.c['arm_speed_limits']),normal=np.array([0.,-1.,0.]),
            telemetry_events=self.events,diagnostic_issue=lambda k,e:self.issues.append((k,repr(e))),diagnostic_write=lambda p,v:Path(p).write_text(v),
            profile_mark=lambda p:setattr(self.profile,'phase',p),force_guard=self.supervisor.force,
            reference=None,phase=self.phase,wrist_routes=[],agent_observations=[],agent_moves=0,observation_generations=[],contact_attempts=[],
            physical_grasps_started=0,attempt_index=0,base_moves=0,regrasp_count=0,recovery_state='UNLOADED_OBSERVATION',near=self.scene['overview'][1])
        return g
    def filtered(self):return np.mean(self.filtered_loads,axis=0) if self.filtered_loads else np.zeros(2)
    def event(self,name,**data):
        self.events.append(dict(event=name,t=self.tick*self.dt,phase=self.phase,**data));self.render.event_clip(self.tick*self.dt)
    def optional(self,name,fn):
        try:return np.asarray(fn())
        except Exception as error:
            if not any(i[0]==name for i in self.issues):self.issues.append((name,repr(error)))
            return None
    def _read_ft(self,reactions):
        from xarm_wrist_ft import tool_gravity_world,wrench_at
        T=self.pose('xarm_gripper_base_link');bodies=[]
        for name,prop in self.tool_properties.items():
            B=self.pose(name);bodies.append(dict(name=name,mass_kg=prop['mass_kg'],T_world_body=B,com_world_m=B[:3,:3]@prop['com_body_m']+B[:3,3],inertia_world_kg_m2=B[:3,:3]@prop['inertia_body_kg_m2']@B[:3,:3].T))
        raw=np.asarray(reactions)[self.sensor.index].copy();gravity=tool_gravity_world(bodies,T[:3,3]);bias=np.asarray(self.job['ft_calibration']['bias_sensor'])
        # Guard retains measured dynamic tool loading conservatively. Segment-local
        # estimator separately compensates tool inertia using these 240 Hz poses.
        comp=wrench_at(raw-bias,T,T[:3,3])+gravity
        return dict(raw_wrench_sensor=raw,T_world_sensor=T,valid=bool(np.isfinite(raw).all()),timestamp_s=self.tick*self.dt,world_simulation_timestamp_s=float(self.world.current_time),sample_period_s=self.dt,body_states=bodies,tool_gravity_wrench_world=gravity,gravity_compensated_wrench_world=comp,tool_inertia_compensation='segment-local estimator; raw poses retained'),comp
    def step(self):
        from isaacsim.core.utils.types import ArticulationAction
        if self.stopped or (self.output/'STOP_AND_EXPORT').exists():raise KeyboardInterrupt('OWNER_REQUESTED_EXPORT')
        self.phase=self.g.get('phase',self.phase);self.profile.phase=self.phase;self.qvelocity=np.asarray(self.g.get('qvelocity',self.qvelocity));self.policy.phase(self.phase)
        # Validate commanded authority before submitting it to PhysX. Native
        # drive max-effort enforcement remains independent of this software check.
        if self.supervisor.latched is not None:raise SafetyStop('LATCHED_'+self.supervisor.latched['reason'],self.supervisor.latched)
        self.ensure_execution_budget(self.phase)
        command=self.qtarget[self.arm]
        if self.model.margin(command)<=self.policy.data['joint_margin_rad']:raise SafetyStop('COMMAND_JOINT_MARGIN')
        if not np.isfinite(command).all() or not np.isfinite(self.qvelocity).all():raise SafetyStop('INVALID_COMMAND')
        if np.any(abs(self.qvelocity)>np.asarray(self.c['arm_speed_limits'])/3+1e-12):raise SafetyStop('COMMAND_JOINT_VELOCITY')
        # PhysX native report points are emitted at contact detection, before
        # this step's integration/constraint projection. Preserve the measured
        # finger frame at that instant; do not compare a native point against
        # the already advanced end-of-step tip surface.
        pre_contact_poses={logical:self.pose(native)@self.pad_frames[native] for native,logical in [('left_finger','gripper_link1'),('right_finger','gripper_link2')]}
        with self.profile.measure('actuation'):
            self.native.clear();self.controller.apply_action(ArticulationAction(joint_positions=self.qtarget[self.arm],joint_velocities=self.qvelocity,joint_indices=self.arm));self.controller.command_width(self.jaw_width)
        if self.engineering_before_physics is not None:self.engineering_before_physics()
        with self.profile.measure('physics_and_contact_callback'):self.world.step(render=False,update_fabric=True)
        with self.profile.measure('state_and_FT'):
            positions,quaternions=self.pose_view.get_world_poses();matrices=np.repeat(np.eye(4)[None],len(positions),axis=0);matrices[:,:3,3]=positions;matrices[:,:3,:3]=Rotation.from_quat(np.roll(quaternions,-1,axis=1)).as_matrix();self.pose_cache=dict(zip(self.pose_names,matrices))
            q=np.asarray(self.robot.get_joint_positions());dq=np.asarray(self.robot.get_joint_velocities());reactions=np.asarray(self.robot.get_measured_joint_forces());contacts=self.native.state()
            T=self.tcp();D=self.moving();ft,wrench=self._read_ft(reactions);loads=contacts['finger_handle_forces_n'];self.filtered_loads.append(list(loads.values()))
            speed=0. if self.previous is None else float(np.linalg.norm(T[:3,3]-self.previous[:3,3])/self.dt);self.previous=T
            relative=np.linalg.inv(D)@T
            row=dict(t=self.tick*self.dt,phase=self.phase,q=q,aperture_m=self.controller.geometry.width(float(q[self.controller.master])),
                margin_rad=self.model.margin(q[self.arm]),T_tcp=T,T_moving_link=D,door_angle_deg=math.degrees(self.actual()),forces_n=loads,contacts=contacts['contacts'],
                command_q_arm=self.qtarget[self.arm].copy(),command_q_velocity_arm=self.qvelocity.copy(),command_aperture_m=self.jaw_width,wrist_ft=ft,
                joint_velocities=dq,joint_reactions=reactions,projected_joint_efforts=self.optional('projected_joint_efforts',self.robot.get_measured_joint_efforts),explicit_applied_efforts=self.optional('explicit_applied_efforts',self.robot.get_applied_joint_efforts),tcp_speed_m_s=speed,base=self.base.copy(),physics_dt_s=contacts['physics_window_dt_s'],
                relative_translation_slip_m=0. if self.grasp_reference is None else float(np.linalg.norm(relative[:3,3]-self.grasp_reference[:3,3])),
                relative_rotation_slip_deg=0. if self.grasp_reference is None else float(np.degrees(Rotation.from_matrix(relative[:3,:3]@self.grasp_reference[:3,:3].T).magnitude())))
            if self.engineering_load is not None:row['independent_engineering_tool_load_world_n']=np.asarray(self.engineering_load).copy()
            row['actuator_position_target_arm']=self.controller.last.get('implicit_position_target');row['actuator_gravity_bias_nm']=self.controller.last.get('gravity_bias_nm');row['native_G1_master_effort_cap_nm']=float(self.controller.caps[self.controller.master])
            row['telemetry']=dict(pad_load_raw_n=loads,pad_load_filtered_n=self.filtered(),command_aperture_m=self.jaw_width,joint_velocities=dq,measured_joint_forces=reactions,
                hinge_velocity_rad_s=float(self.scene['articulation'].get_joint_velocities()[0]),command_q_velocity_arm=self.qvelocity.copy())
        self.rows.append(row);self.tick+=1;self.g['tick']=self.tick
        for side,load in loads.items():
            present=load>.05
            if self.pad_presence.get(side)!=present:
                if present or side in self.pad_presence:self.event('PAD_CONTACT_ACQUIRED' if present else 'PAD_CONTACT_LOST',finger=side,raw_load_n=load)
                self.pad_presence[side]=present
        try:
            with self.profile.measure('collision_and_safety'):
                if abs(contacts['physics_window_dt_s']-self.dt)>1e-6:raise SafetyStop('PHYSICS_CLOCK_MISMATCH')
                # Measured native finger transforms for contact ownership; no
                # repeated all-link FK unless the original 30 Hz scene check is due.
                P={logical:self.pose(name)@self.pad_frames[name] for name,logical in [('left_finger','gripper_link1'),('right_finger','gripper_link2')]}
                contact_poses=pre_contact_poses if getattr(self.collision,'active',False) else P
                ok,why=self.collision.contact_guard(row['contacts'],contact_poses)
                if row['contacts'] and getattr(self.collision,'active',False):row['panel_contact_detection_poses']=pre_contact_poses
                if ok and self.tick%8==0:
                    P=self.model.poses(q[self.arm],self.base,finger_q=q[self.fingers]);ok,why=self.collision.check(P,D,self.intentional_contact())
                event=self.supervisor.check(phase=self.phase,q=q[self.arm],dq=dq[self.arm],tcp_speed=speed,pads=loads,dt=self.dt,t=row['t'],wrench=wrench,collision_ok=ok,collision_reason=why)
                row['force_event']=event
                if event['status']!='NORMAL' and (not self.events or self.events[-1].get('event')!=event['status']):self.event(event['status'],force=event)
                if self.grasp_reference is not None:
                    self.loss_s=self.loss_s+self.dt if max(self.filtered())<.05 else 0.
                    if self.loss_s>.15:raise SafetyStop('CONTACT_LOST_PAUSE_IN_SAME_SCENE')
        except BaseException as error:
            row['stop']=repr(error);self.event('SAFETY_STOP',reason=repr(error));raise
        finally:
            with self.profile.measure('buffered_evidence'):self.record.append(row)
        if self.render.due(row['t']):self.record_frame()
        return row
    def intentional_contact(self):return self.policy.phase(self.phase) in ('ESTABLISH','LOADED','RELEASE')
    def record_frame(self):
        with self.profile.measure('render_and_video'):
            try:
                camera=self.scene['overview'][1]
                if not getattr(self,'diagnostic_camera_initialized',False):
                    camera.initialize();self.diagnostic_camera_initialized=True
                self.world.render();image=np.asarray(camera.get_rgba())[...,:3]
                if not image.size:return
                import cv2,subprocess,shutil
                w,h=self.policy.data['modes'][self.mode]['video_wh'];image=cv2.resize(image,(w,h))
                if self.video is None:
                    self.video=subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r',str(self.policy.data['modes'][self.mode]['video_hz']),'-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','25',str(self.output/'events.mp4')],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=(self.output/'ffmpeg.log').open('w'))
                self.video.stdin.write(np.ascontiguousarray(image).tobytes());self.video_index.append(self.tick*self.dt)
            except Exception as e:self.issues.append(('record_frame',repr(e)))
    def check_q(self,q,allow=None):
        if self.model.margin(q)<=self.policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
        fingers=np.asarray(self.robot.get_joint_positions())[self.fingers]
        return self.collision.check(self.model.poses(q,self.base,finger_q=fingers),self.moving(),self.intentional_contact() if allow is None else allow)
    def move_path(self,path,phase='PREGRASP',minimum_duration=0.):
        self.ensure_execution_budget(phase)
        self.world.play();self.g['at_safe_park']=False;self.g['phase']=phase;self.phase=phase;start=self.qtarget[self.arm].copy();points=[start]+list(path)
        with self.profile.measure('trajectory_planning'):
            # No physics advances during this deterministic planning call, so
            # snapshot current released fingers and door once, not per sample.
            fingers=np.asarray(self.robot.get_joint_positions())[self.fingers].copy();moving=self.moving().copy();allow=self.intentional_contact()
            def checked(q):
                if self.model.margin(q)<=self.policy.data['joint_margin_rad']:return False,'JOINT_MARGIN'
                return self.collision.check(self.model.poses(q,self.base,finger_q=fingers),moving,allow)
            samples=timed_path(self.solver,self.base,points,self.dt,self.policy.motion(phase),np.asarray(self.c['arm_speed_limits'])/3,self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)],checked,minimum_duration)
        for q,v in samples:
            self.qtarget[self.arm]=q;self.g['qvelocity']=v;self.step()
        self.g['qvelocity']=np.zeros(7)
    def compatible_move(self,q,duration):self.move_path([q],self.g['phase'])
    def stop_command(self):
        self._decelerating=True
        try:
            velocity=np.asarray(self.g['qvelocity']).copy()
            acceleration=self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(self.g['phase'])]
            while np.max(abs(velocity))>1e-12:
                next_velocity=velocity-np.clip(velocity,-acceleration*self.dt,acceleration*self.dt)
                command=self.qtarget[self.arm]+.5*(velocity+next_velocity)*self.dt
                ok,why=self.check_q(command)
                if not ok:raise SafetyStop('STOP_ROUTE_COLLISION:'+why)
                self.qtarget[self.arm]=command;self.g['qvelocity']=next_velocity;self.step();velocity=next_velocity
            self.g['qvelocity']=np.zeros(7)
        finally:self._decelerating=False
    def hold(self,seconds):
        self.world.play();self.stop_command();self.g['qvelocity']=np.zeros(7)
        for _ in range(int(round(seconds/self.dt))):self.step()
    def begin_reference(self):
        from interaction_reference_handover import seed_pose
        self.seed=np.asarray(self.robot.get_joint_positions())[self.arm].copy();self.reference_bias=self.qtarget[self.arm]-self.seed
        self.reference_drive=CartesianReference(seed_pose(self.solver,self.base,self.seed));self.event('CONTINUOUS_REFERENCE_HANDOVER',unchanged_q_command=self.qtarget[self.arm].tolist())
    def motion_limits(self,phase):return self.policy.motion(phase)
    def cartesian_step(self,goal,phase):
        self.ensure_execution_budget(phase)
        self.g['phase']=phase;self.phase=phase
        if self.reference_drive is None:self.begin_reference()
        # An exactly stationary virtual reference needs no repeated QP solve.
        # Native position feedback, force sensing and collision guards still run
        # every original physics interval. This is numerical zero, not a new
        # physical tracking tolerance or grasp acceptance condition.
        if np.max(abs(np.asarray(goal)-self.reference_drive.pose))<1e-12 and np.max(abs(self.g['qvelocity']))<1e-10:
            return self.step()
        target=self.reference_drive.advance(goal,self.dt,self.motion_limits(phase))
        with self.profile.measure('Pink_IK'):q=self.solver.velocity_step(target,self.base,self.seed,self.dt)
        if q is None:raise RuntimeError('LOCAL_IK_UNAVAILABLE')
        requested=q+self.reference_bias;delta=requested-self.qtarget[self.arm]
        # One scalar preserves IK branch direction while imposing real joint
        # speed bounds; command acceleration is limited before position advances.
        wanted=delta/self.dt;vel=np.asarray(self.g['qvelocity']);acc=self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)];speeds=np.asarray(self.c['arm_speed_limits'])/3
        wanted=np.clip(wanted,-speeds,speeds);v=vel+np.clip(wanted-vel,-acc*self.dt,acc*self.dt)
        command=self.qtarget[self.arm]+v*self.dt
        # Preserve the original hand/closure 30 Hz geometric guard and the
        # panel primitive's per-step command guard; native contacts stay 240 Hz.
        if self.tick%8==0 or phase in ('PANEL_APPROACH','PANEL_PUSH','PANEL_SEPARATION'):
            with self.profile.measure('command_collision_queries'):ok,why=self.check_q(command)
            if not ok:raise SafetyStop('COMMAND_COLLISION:'+why)
        self.qtarget[self.arm]=command;self.g['qvelocity']=v;self.seed=command-self.reference_bias
        return self.step()
    def close(self):
        with self.profile.measure('export'):
            index=self.record.close()
            if self.video is not None:
                try:self.video.stdin.close();self.video.wait(timeout=15)
                except Exception as e:self.issues.append(('video_close',repr(e)))
            (self.output/'events.json').write_text(json.dumps(self.events,indent=2));(self.output/'video_timestamps.json').write_text(json.dumps(self.video_index))
        (self.output/'RUNTIME_RESULT.json').write_text(json.dumps(dict(profile=self.profile.report(self.tick*self.dt),records=index,issues=self.issues,policy_sha256=self.policy.sha256,owned_actor_cleanup_requested=True),indent=2))
        self.world.pause();self.app.close()
