"""Unvalidated prismatic engineering feedback; frozen revolute path is untouched.

The frozen runtime already preserves the loaded command bias and bounds joint /
Cartesian speed and acceleration. Its nominal-seed QP does not observe subsequent
servo deflection. Re-seed that same QP from measured arm state during rail motion,
retaining the handover bias and all existing command/safety constraints. The
optional integral prototype introduces an explicitly declared outer tracking
time constant; native arm/G1 actuator gains, friction and authority are unchanged.
Native microtracking remains insufficient; do not promote to the benchmark.
"""
import numpy as np
from .probing_v7 import RuntimeV7, OperationsV7
from .probing_v2 import serve
from .segments import validate_action


class PrismaticFeedbackRuntime(RuntimeV7):
    def __init__(self,job):
        if job['articulation_v7']['joint_type']!='prismatic':
            raise ValueError('Prismatic development entry cannot run revolute cases')
        if job.get('experiment')=='V7_BENCHMARK' or job.get('benchmark_registration'):
            raise ValueError('Unvalidated engineering entry cannot launch registered benchmark')
        super().__init__(job)
        self.measured_rail_feedback=bool(job.get('prismatic_measured_ik_feedback',True))
        self.integrated_measured_feedback=bool(job.get('prismatic_integrated_ik_feedback',False))
        if self.integrated_measured_feedback and not self.measured_rail_feedback:raise ValueError('INTEGRAL_FEEDBACK_REQUIRES_MEASURED_IK')
        self.integral_window_s=float(job['shared_execution']['progress_window_s']) if self.integrated_measured_feedback else None
        self.integral_initial_error=None;self.integral_limited=False
        if self.integrated_measured_feedback:
            if not np.isfinite(self.integral_window_s) or self.integral_window_s<=0:raise ValueError('POSITIVE_MEASURED_PROGRESS_WINDOW_REQUIRED')
            # Explicit passive recorder seam. Keep original FK command channels;
            # add the distinct physical pose reference and actual drive target.
            self._coordinate_fields=self.record.coordinate_fields
            self.record.coordinate_fields=self.reference_fields
            self.record.metadata['prismatic_integral_feedback']=dict(
                window_s=self.integral_window_s,robot_gains_or_friction_changed=False,
                command_TCP_pose='FK of nominal arm command, including accumulated tracking compensation',
                physical_reference_TCP_pose='bounded Cartesian reference supplied to measured-state IK',
                implicit_position_target_arm='actual implicit position target including original robot gravity compensation; not motor effort')
        from agent_command_protocol_v12 import atomic_json
        materials={}
        for label,view in [('left_finger',self.scene['finger_views'][0]),('right_finger',self.scene['finger_views'][1])]:
            try:
                getter=getattr(view._physics_view,'get_material_properties')
                materials[label]=dict(available=True,raw_native_material_properties=np.asarray(getter()).tolist(),API_documentation=getter.__doc__,component_convention='retain installed API raw fields; do not infer measured friction from force ratios')
            except Exception as error:materials[label]=dict(available=False,reason=repr(error))
        atomic_json(self.output/'PRISMATIC_CONTACT_MATERIAL_READBACK.json',materials)
        self.contact_retention=bool(job.get('prismatic_contact_retention',False))
        self.retention=None;self.retention_epoch=np.zeros(3);self.retention_closure=None
        if self.contact_retention:
            original=self.Closure
            def closure_context(T,policy):
                value=original(T,policy)
                self.retention_closure=value;self.retention=None;self.retention_epoch=np.zeros(3)
                return value  # original update/closure/effort logic, unmodified
            self.Closure=closure_context
            atomic_json(self.output/'PRISMATIC_RETENTION_POLICY.json',dict(
                development_only=True, original_closure_unchanged=True,
                jaw_target_feedback=False, clamp_authority_changed=False,
                step_m=self.closure_policy['max_centering_step_m'],
                total_bound_m=self.closure_policy['max_centering_displacement_m'],
                cadence_physics_steps=self.filtered_loads.maxlen,
                stable_band_source='existing .8 * preload_n FORCE_HOLD condition; command feature, not a new gate',
                direction='measured pad-centre axis projected perpendicular to known rail',
                finite_loading_budget_and_all_protections_unchanged=True))

    def begin_reference(self):
        super().begin_reference()
        if getattr(self,'integrated_measured_feedback',False):
            from interaction_reference_handover import seed_pose
            self.integral_initial_error=(seed_pose(self.solver,self.base,self.qtarget[self.arm])[:3,3]
                                         -seed_pose(self.solver,self.base,self.seed)[:3,3])
        if getattr(self,'contact_retention',False):
            self.retention_epoch=self.retention.offset.copy() if self.retention is not None else np.zeros(3)

    def reference_fields(self,row):
        from .compact_v2 import pose7
        values=self._coordinate_fields(row)
        def vector(name):
            value=row.get(name)
            return np.full(7,np.nan,dtype=np.float32) if value is None else np.asarray(value,np.float32)
        values.update(physical_reference_TCP_pose=pose7(self.reference_drive.pose) if self.reference_drive is not None else np.full(7,np.nan),
            implicit_position_target_arm=vector('actuator_position_target_arm'),
            actuator_gravity_bias_nm=vector('actuator_gravity_bias_nm'),
            tracking_integral_limited=np.uint8(self.integral_limited))
        return values

    def retention_phase(self,phase):
        return (getattr(self,'contact_retention',False) and self.grasp_reference is not None
                and (phase=='FORCE_HOLD' or self.policy.phase(phase)=='LOADED'))

    def retention_goal(self,goal,phase):
        if not self.retention_phase(phase):return goal
        if self.reference_drive is None:self.begin_reference()
        if self.retention is None:
            from .prismatic_retention_v7 import PrismaticContactBalance
            if self.retention_closure is None:raise RuntimeError('RETENTION_CLOSURE_CONTEXT_UNAVAILABLE')
            closure=self.retention_closure
            self.retention=PrismaticContactBalance(self.closure_policy,self.contact_evidence.axis,
                self.filtered_loads.maxlen,closure.command[:3,3]-closure.origin[:3,3],self.dt)
        centres=[]
        for native,logical in [('left_finger','gripper_link1'),('right_finger','gripper_link2')]:
            T=self.pose(native)@self.pad_frames[native]
            centres.append((T@np.r_[self.model.pads[logical].mean(0),1])[:3])
        update=self.retention.update(self.tick,self.filtered(),centres)
        if update is not None:self.event('PRISMATIC_RETENTION_CORRECTION',**update)
        target=np.array(goal,copy=True);target[:3,3]+=self.retention.offset-self.retention_epoch
        return target

    def hold(self,seconds):
        phase=self.g.get('phase',self.phase)
        if not self.retention_phase(phase):return super().hold(seconds)
        # Keep the original duration and live physics. A continuous measured-
        # state handover preserves the active arm command before micro-motion.
        self.world.play();self.stop_command();self.reference_drive=None;self.begin_reference()
        goal=self.reference_drive.pose.copy()
        for _ in range(int(round(seconds/self.dt))):self.cartesian_step(goal,phase)
        self.stop_command()

    def cartesian_step(self,goal,phase):
        goal=self.retention_goal(goal,phase)
        active=(self.policy.phase(phase)=='LOADED' or
                (getattr(self,'integrated_measured_feedback',False) and phase=='FORCE_HOLD' and self.grasp_reference is not None))
        if not self.measured_rail_feedback or not active:
            self.integral_limited=False
            return super().cartesian_step(goal,phase)
        from .policy import SafetyStop
        self.ensure_execution_budget(phase);self.g['phase']=phase;self.phase=phase
        if self.reference_drive is None:self.begin_reference()
        target=self.reference_drive.advance(goal,self.dt,self.motion_limits(phase))
        measured=np.asarray(self.robot.get_joint_positions())[self.arm].copy()
        # A stationary nominal reference must not skip measured-state feedback.
        # Use the same Pink task/cost/gain, not a new force or arm actuator servo.
        integrated=getattr(self,'integrated_measured_feedback',False)
        solver_target=target
        if integrated:
            from interaction_reference_handover import seed_pose
            from scipy.spatial.transform import Rotation
            current=seed_pose(self.solver,self.base,measured)
            # Pink's task gain is per solve. Convert to the existing physical
            # progress-window time constant before integrating measured motion.
            fraction=min(1.,self.dt/(self.integral_window_s*float(self.solver.task.gain)))
            solver_target=current.copy();solver_target[:3,3]+=fraction*(target[:3,3]-current[:3,3])
            rotation=Rotation.from_matrix(target[:3,:3]@current[:3,:3].T).as_rotvec()
            solver_target[:3,:3]=Rotation.from_rotvec(fraction*rotation).as_matrix()@current[:3,:3]
        with self.profile.measure('Pink_IK'):
            q=self.solver.velocity_step(solver_target,self.base,measured,self.dt)
        if q is None:raise RuntimeError('LOCAL_IK_UNAVAILABLE')
        requested=self.qtarget[self.arm]+q-measured if integrated else q+self.reference_bias
        self.integral_limited=False
        if integrated:
            command_pose=seed_pose(self.solver,self.base,self.qtarget[self.arm])
            next_pose=seed_pose(self.solver,self.base,requested)
            deficit=command_pose[:3,3]-current[:3,3]-self.integral_initial_error
            limits=self.policy.motion('LOADED')
            allowance=limits['speed_m_s']**2/(2*limits['acceleration_m_s2'])+limits['speed_m_s']*self.dt
            # Conditional integration, not a new abort or expanded authority.
            # Stop growing a saturated tracking bias; existing acceleration
            # constraints below still decelerate the command continuously.
            if np.linalg.norm(deficit)>=allowance and deficit@(next_pose[:3,3]-command_pose[:3,3])>0:
                requested=self.qtarget[self.arm].copy();self.integral_limited=True
        delta=requested-self.qtarget[self.arm]
        wanted=delta/self.dt;velocity=np.asarray(self.g['qvelocity'])
        acc=self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)]
        speeds=np.asarray(self.c['arm_speed_limits'])/3
        wanted=np.clip(wanted,-speeds,speeds)
        v=velocity+np.clip(wanted-velocity,-acc*self.dt,acc*self.dt)
        command=self.qtarget[self.arm]+v*self.dt
        if self.tick%8==0:
            with self.profile.measure('command_collision_queries'):ok,why=self.check_q(command)
            if not ok:raise SafetyStop('COMMAND_COLLISION:'+why)
        self.qtarget[self.arm]=command;self.g['qvelocity']=v
        self.seed=command-self.reference_bias
        return self.step()


class PrismaticFeedbackOperations(OperationsV7):
    def release_park(self):
        # A resource boundary may interrupt unloaded view travel. Stop under
        # its actual motion phase first; RELEASE's lower speed limit applies
        # only once that travel has stopped. Original per-step guards stay on.
        phase=self.g.get('phase',self.r.phase)
        if (self.r.supervisor.latched is None and self.r.grasp_reference is None
                and max(self.r.filtered())<=self.r.policy.data['force']['unloaded_pad_n']
                and self.r.policy.phase(phase) in ('FREE','APPROACH','SEPARATE','PARK')):
            self.r.event('UNLOADED_EXIT_DECELERATE_BEFORE_RELEASE',previous_phase=phase)
            self.r.stop_command()
            self.g['phase']='SAFE_PARK_MOTION'
            self.r.hold(.3)  # unchanged stopped-state observation duration
        return super().release_park()

    def attempt(self,command):
        result=super().attempt(command)
        # Preserve the original contact stop. Only a nonlatched absence-of-load
        # stop may enter the existing validated jaw release / parking primitive.
        # Never clear a hard force, collision, speed, joint or actuator latch.
        reasons=[s.get('reason') for s in result.get('segments',[])]
        if ('CONTACT_LOST_PAUSE_IN_SAME_SCENE' in reasons
                and self.r.supervisor.latched is None
                and not result.get('safe_exit',{}).get('safe')):
            self.r.event('CONTACT_OPERATION_STOPPED_REQUEST_GUARDED_EXIT',
                         original_status=result.get('operation_status'))
            # No more object-driving motion. The ordinary release resets this
            # grasp-only presence monitor too, but does so after deceleration;
            # a contact-loss stop would otherwise recursively stop that exit.
            self.r.grasp_reference=None;self.g['reference']=None
            self.g['phase']='RELEASE'
            try:result['safe_exit']=self.release_park()
            except BaseException as error:
                result['safe_exit']=dict(safe=False,reason=repr(error))
            result['contact_stop_retained']=True
            result['Agent_recovery_available']=bool(result['safe_exit'].get('safe'))
        return result

    def build_geometry(self):
        if not self.r.job.get('prismatic_bidirectional_ranking',True):
            return super().build_geometry()
        # Scope the observed-only ranking adapter to this single planning call.
        from unittest.mock import patch
        import native_g1_multiview_input as visual
        from native_g1_prismatic_ranking import rescore
        with patch.object(visual,'rescore',rescore):
            return super().build_geometry()


def serve_prismatic(r):
    if r.job.get('engineering_microtracking',False):
        from .prismatic_microtracking_v7 import run
        return run(r)
    def check(a):
        validate_action(a)
        from .articulation_v7 import Articulation
        Articulation('prismatic',(1,0,0),(0,0,0),tuple(r.job['articulation_v7']['limits_native'])).validate_target(a)
        if not a['role'].startswith(('FIT','VALIDATION')):raise ValueError('Predeclare FIT/VALIDATION')
    return serve(r,operations_factory=PrismaticFeedbackOperations,action_validator=check)
