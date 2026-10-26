"""Scoped mobile-mount assembly around byte-frozen V8 physical owners."""
import json
from pathlib import Path
from unittest.mock import patch

from .mobile_mount import author,drive_class


def runtime_class(original):
    class MobileRuntime(original):
        def __init__(self,job):
            import xarm_ft_v7_scene as plant
            import xarm7_native_gripper as gripper
            original_author=plant.author
            def assemble(stage,g,case,output):
                audit=original_author(stage,g,case,output)
                author(stage,output)
                # This object was wrapped before the mount existed and cached
                # /Geometry/world as its tensor root. Reinitialize the SAME
                # scene-registered wrapper before the first world.reset(),
                # without position/orientation or xform reset. All caller and
                # Scene references retain the one explicit robot object.
                from isaacsim.core.prims import SingleArticulation
                SingleArticulation.__init__(g['robot'],prim_path='/World/Piper',
                    name='piper',reset_xform_properties=False)
                return audit
            # Explicit constructor-only bindings: never hot-swap an active actor.
            # NativeDrive still owns every arm/G1 command and its exact caps.
            with patch.object(plant,'author',assemble),patch.object(gripper,'NativeDrive',drive_class(gripper.NativeDrive)):
                super().__init__(job)
            from .load_telemetry import instrument as instrument_load
            instrument_load(self)
            from .object_state_telemetry import instrument as instrument_object
            instrument_object(self)
            if job.get('v84_abc') and job['articulation_v7']['joint_type']=='revolute':
                from .preload_extension import instrument as instrument_preload
                instrument_preload(self)
            from .video_evidence import configure
            configure(self)
            from .orchestration_extension import install
            install(self)  # new opt-in actor constructor only; never hot-load
            import numpy as np
            from isaacsim.core.prims import RigidPrim
            self.mount=json.loads((self.output/'V81_MOBILE_MOUNT.json').read_text())
            self.base_indices=np.array([self.names.index(n) for n in self.mount['joint_names']])
            self.base_view=RigidPrim(prim_paths_expr=self.mount['base_body'],name='v81_measured_base',reset_xform_properties=False)
            self.base_view.initialize()
            self.initial_base=self.base.copy()
            self.base_goal=np.asarray(self.robot.get_joint_positions())[self.base_indices].copy()
            self.base_velocity=np.zeros(2)
            self.base_active=False
            self.base_trace=[]
            self.base_generation=0
            self.v81_contact_following_mode='legacy_window'
            from .motion_dwell import MotionDwell
            self.station_motion=MotionDwell(self.dt)
            if job['articulation_v7']['joint_type']=='revolute':
                from .contact_measurement import body_pose_field
                previous_fields=self.record.coordinate_fields
                self.record.coordinate_fields=lambda row:body_pose_field(previous_fields,row)
                self.contact_evidence.record_points=True
                self.contact_evidence.failures['legacy_revolute_point_velocity_field']='Per-point legacy rail velocity field is invalid for rotation; recompute from recorded body/finger poses. V81 scalar diagnostic uses full rigid motion.'
                self.record.metadata['v81_contact_diagnostics']=dict(
                    scalar_tangent_velocity='full measured rigid-point relative velocity; pre-integration alignment',
                    legacy_per_point_velocity='invalid for revolute; do not use',
                    normal_and_friction_measurements_unchanged=True,
                    feeds_protection_controller_or_estimator=False,
                    moving_link_pose='known-state supervision, must be evaluation-only in the future unknown-geometry stage')
            self.read_base()
            from .collision import MobileScene
            self.mobile_collision=MobileScene(self.export,self.model,self.allowed,self.initial_base)
            self.collision=self.mobile_collision;self.g['collision']=self.collision
            # Passive object law remains the existing owner. Base actuation is
            # separate and cannot submit object or arm/G1 joint commands.
            passive=self.engineering_before_physics
            def actuate():
                if passive is not None:passive()
                from isaacsim.core.utils.types import ArticulationAction
                self.controller.raw.apply_action(ArticulationAction(joint_positions=self.base_goal,
                    joint_velocities=self.base_velocity,joint_indices=self.base_indices))
            self.engineering_before_physics=actuate
            self.record.metadata['v81_base']=dict(joint_indices=self.base_indices.tolist(),joint_names=self.mount['joint_names'],
                moving_only_when_verified_unloaded=True,kind='simulated driven XY carriage; not wheels')
            if job.get('v81_integrated_repair') and job['articulation_v7']['joint_type']=='prismatic':
                from .preload_extension import instrument
                instrument(self)
            if job.get('instance_uuid'):
                from .instance_admission import native_unloaded_check
                native_unloaded_check(self)

        def move_path(self,path,phase='PREGRASP'):
            from .path_admission import preflight
            path=list(path)
            preflight(self,path,phase,0.)
            # Original timing/actuation and measured-state supervisor remain.
            return super().move_path(path,phase)

        def record_frame(self):
            if self.job.get('v83_continuous_video_hz'):
                from .video_evidence import configure
                from interaction_runtime.runtime import Runtime
                configure(self)
                return Runtime.record_frame(self)
            return super().record_frame()

        def step(self):
            from interaction_runtime.policy import SafetyStop
            try:row=super().step()
            except SafetyStop as error:
                from agentic_interaction_v87.constraint_feedback import action_fault
                normalized = action_fault(self, error)
                if normalized is None:
                    return self.rows[-1]  # pad-report absence must not abort braking
                if normalized is not error:
                    raise normalized from error
                if self.job.get('v84_abc'):
                    from .terminal_safety import persist
                    try:persist(self, dict(safe_exit=dict(safe=False,
                        reason='HARD_STOP_NO_AUTOMATIC_CONTINUATION')), pause=True)
                    except Exception as export_error:
                        error.add_note('Terminal safety export failed: '+repr(export_error))
                raise
            from .integration import progress
            progress(self,row)
            return row

        def station_boundary(self,record=True):
            import numpy as np
            from .station import Boundary
            from agentic_interaction_v8.resources import digest
            q=np.asarray(self.robot.get_joint_positions());dq=np.asarray(self.robot.get_joint_velocities())
            parked=bool(self.g.get('at_safe_park'))
            # Also usable at the normal initialized native rest for the short
            # interface check; it is not an arbitrary released observation pose.
            home=np.asarray(self.c['home_q']) if 'home_q' in self.c else np.array([0,-.5,0,1.,0,.5,0])
            if parked and self.g.get('last_exit_plan'):
                home=np.asarray(self.g['last_exit_plan']['parking_q'])
            stationary=(np.max(abs(q[self.arm]-home))<=.025 and
                self.station_motion.ready(self.rows[-1]['t'],q[self.arm]))
            unloaded=self.grasp_reference is None and max(self.filtered())<=self.policy.data['force']['unloaded_pad_n'] and self.supervisor.force.settled()
            released=float(self.rows[-1]['aperture_m'])>=.083
            velocity=float(self.rows[-1].get('joint_coordinate_velocity',0.))
            boundary=Boundary(self.job['episode_id'],self.base_generation,len(self.g['observation_generations']),
                float(self.world.current_time),tuple(self.base),tuple(q[self.arm]),tuple(q[self.fingers]),
                self.actual(),'m' if self.job['articulation_v7']['joint_type']=='prismatic' else 'rad',
                bool(stationary),abs(velocity)<1e-4,bool(released),bool(unloaded),
                bool(parked or stationary),None if self.supervisor.latched is None else str(self.supervisor.latched),
                digest(self.output/'cooked_initial.json'))
            from dataclasses import asdict
            if record:
                (self.output/'V81_STATION_BOUNDARY.json').write_text(json.dumps(dict(
                    boundary=asdict(boundary),arm_velocity_rad_s=dq[self.arm].tolist(),
                    native_velocity_instantaneous_below_01=bool(np.max(abs(dq[self.arm]))<.01),
                    measured_motion_dwell=self.station_motion.latest,
                    measured_aperture_m=float(self.rows[-1]['aperture_m']),
                    object_velocity_native_s=velocity,pad_loads_n=self.filtered().tolist(),
                    grasp_reference_present=self.grasp_reference is not None,
                    force_history_settled=bool(self.supervisor.force.settled()))))
            return boundary

        def cartesian_step(self,goal,phase):
            if (getattr(self,'v81_contact_following_mode','legacy_window')=='rate_consistent'
                    and self.job['articulation_v7']['joint_type']=='prismatic'
                    and self.transverse_unloading_enabled and self.grasp_reference is not None
                    and self.policy.phase(phase)=='LOADED' and not self._decelerating
                    and self.unloading is None):
                from .contact_following import RateConsistentFollowing
                if self.unloading_closure is None:
                    raise RuntimeError('UNLOADING_CLOSURE_CONTEXT_UNAVAILABLE')
                c=self.unloading_closure
                self.unloading=RateConsistentFollowing(self.closure_policy,
                    self.contact_evidence.axis,self.filtered_loads.maxlen,
                    c.command[:3,3]-c.origin[:3,3])
                self.event('V81_RATE_CONSISTENT_FOLLOWING',
                    filtered_window_steps=self.filtered_loads.maxlen,
                    integration_steps=1,original_centering_caps_preserved=True,
                    jaw_command_feedback=False,actuator_authority_changed=False)
            return super().cartesian_step(goal,phase)

        def evaluate_station(self,target):
            import numpy as np
            from .station import evaluate_station
            from .policy_checks import settle_station
            try:b=settle_station(self)
            finally:self.station_boundary()  # preserve the actual refusal, too
            b.check();motion=self.policy.motion('SAFE_PARK_MOTION')
            if len(target)==2:target=(*target,*b.base_xyzyaw[2:])
            goal=np.asarray(self.robot.get_joint_positions())[self.base_indices]+np.asarray(target[:2])-np.asarray(b.base_xyzyaw[:2])
            stop_margin=motion['speed_m_s']**2/(2*motion['acceleration_m_s2'])+motion['speed_m_s']*self.dt
            lo,hi=self.mount['native_limits_m']
            if np.any(goal<lo+stop_margin) or np.any(goal>hi-stop_margin):
                return dict(valid=False,reason='NATIVE_CARRIAGE_LIMIT_WITH_STOP_RESERVE',target=list(target),
                    goal_joints=goal.tolist(),stop_margin_m=stop_margin)
            M=self.moving().copy()
            def sweep(state,path):
                for p in path:
                    P=self.model.poses(np.asarray(state.arm_q),p['base'],finger_q=np.asarray(state.finger_q))
                    ok,why=self.collision.check(P,M,False)
                    if not ok:return dict(valid=False,reason=why)
                return dict(valid=True,samples=len(path),entire_resting_robot_and_native_support=True,
                            timed_sample_s=self.dt,planned_rest_to_rest=True)
            def destination(state,goal):
                return sweep(state,[dict(base=goal)])
            return evaluate_station(b,target,speed=motion['speed_m_s'],acceleration=motion['acceleration_m_s2'],
                collision_interval_s=self.dt,validate_swept_route=sweep,validate_destination=destination)

        def move_station(self,target):
            import numpy as np
            from .policy_checks import require_station_actuation
            require_station_actuation(self)
            self.ensure_execution_budget('PREGRASP')
            plan=self.evaluate_station(target)  # always revalidate actual measured state
            if not plan['valid']:return plan
            before=self.base.copy();initial_joints=np.asarray(self.robot.get_joint_positions())[self.base_indices].copy()
            path=plan['selected']['path'];self.world.play();self.g['phase']='SAFE_PARK_MOTION'
            self.base_active=True
            try:
                for sample in path[1:]:
                    self.ensure_execution_budget('PREGRASP')
                    require_station_actuation(self)
                    self.base_goal=initial_joints+np.asarray(sample['base'][:2])-before[:2]
                    self.base_velocity=np.asarray(sample['velocity_xy'])
                    self.step()
                self.base_velocity=np.zeros(2);self.hold(.3)
                self.read_base()
                error=float(np.linalg.norm(self.base[:2]-np.asarray(target)[:2]))
                reached=error<=.001
                result=dict(executed=True,reached=reached,actual_base=self.base.tolist(),
                    initial_base=before.tolist(),position_error_m=error,plan=plan,
                    object_coordinate=self.actual(),object_joint_command=False)
                if reached:self.base_generation+=1
                return result
            finally:
                self.base_velocity=np.zeros(2)
                self.base_goal=np.asarray(self.robot.get_joint_positions())[self.base_indices].copy()
                self.base_active=False
                (self.output/'V81_BASE_TRACE.json').write_text(json.dumps(self.base_trace))

        def read_base(self):
            import numpy as np
            from scipy.spatial.transform import Rotation
            p,q=self.base_view.get_world_poses();T=self.matrix(p[0],q[0])
            B=T@np.linalg.inv(self.model.chain.root_to_link('link_base',{}))
            self.base[:3]=B[:3,3]
            self.base[3]=Rotation.from_matrix(B[:3,:3]).as_euler('xyz',degrees=True)[2]
            self.g['base']=self.base
            return B

        def enrich_measurement(self,row,pre_contact_poses):
            previous_fingers=dict(self.contact_evidence.previous) if hasattr(self,'contact_evidence') else {}
            if hasattr(self,'base_view'):
                B=self.read_base()
                row['measured_base_xyzyaw']=self.base.copy()
                if self.base_active:
                    self.base_trace.append(dict(t=row['t'],base=self.base.tolist(),
                        commanded_joint_positions_m=self.base_goal.tolist(),
                        measured_joint_positions_m=row['q'][self.base_indices].tolist()))
                if hasattr(self,'station_motion'):
                    self.station_motion.update(row['t'],row['q'][self.arm],row['T_tcp'],self.actual())
            result=super().enrich_measurement(row,pre_contact_poses)
            if (self.job['articulation_v7']['joint_type']=='revolute'
                    and 'contact_diagnostic' in row):
                from .contact_measurement import relative_contact_speed
                speed=None
                if len(self.rows)>=2:
                    speed=relative_contact_speed(previous_fingers,pre_contact_poses,
                        self.rows[-2]['T_moving_link'],self.rows[-1]['T_moving_link'],
                        row['contacts'],self.rows[-1]['t']-self.rows[-2]['t'])
                row['contact_diagnostic'].update(relative_tangent_speed_m_s=float('nan') if speed is None else speed,
                    slip_velocity_available=speed is not None,
                    tangent_velocity_source='V81 full measured rigid-point velocity; diagnostic only')
            return result

        def close(self):
            (self.output/'V81_BASE_TRACE.json').write_text(json.dumps(self.base_trace))
            return super().close()
    MobileRuntime.__name__='Mobile'+original.__name__
    return MobileRuntime
