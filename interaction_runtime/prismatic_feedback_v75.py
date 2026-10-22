"""Prismatic-only live evidence adapter; frozen execution/safety are inherited.

This repairs evidence identity and conditional whole-chain safety/admission.
No task load limit, wrench compensation or actuator authority is increased.
Instantiate explicitly in a V7.5 engineering entry only.
"""
import json, threading
import numpy as np
from .prismatic_v73 import PrismaticRuntimeV73,PrismaticOperationsV73
from .prismatic_lifecycle_v75 import evidence_identity


class PrismaticEvidenceRuntimeV75(PrismaticRuntimeV73):
    def __init__(self,job):
        if job.get('prismatic_version')!='V7.5':raise ValueError('Explicit V7.5 engineering configuration required')
        super().__init__(job)
        self.v75_config=json.loads((self.root/'configs/prismatic_execution_v75.json').read_text())
        self.g.update(v75_active_attempt=0,v75_attempt_generations={},v75_attempt_native_point_ids={},
            v75_periodic_arm_indices=[i for i,n in enumerate(self.model.names)
                if any(j.name==n and j.kind in ('revolute','continuous')for j in self.model.chain.joints.values())],
            v75_arm_speed_limits=self.c['arm_speed_limits'])
        from xarm_ft_known_joint_prior import native_joint_prior
        self.g['v75_motion_prior']=native_joint_prior(job['asset_root'],self.moving())
        from .prismatic_load_v75 import LoadTrend
        self.v75_load_trend=LoadTrend(self.policy);self.v75_load_stop=None
        previous=self.record.coordinate_fields
        def identity_fields(row):
            fields=previous(row)
            def arm_vector(name):
                v=row.get(name)
                return np.full(7,np.nan,np.float32)if v is None else np.asarray(v,np.float32)
            fields.update(attempt_id_v75=np.uint16(self.g['v75_active_attempt']),
                observation_generation_v75=np.uint16(len(self.g['observation_generations'])),
                implicit_arm_drive_target_rad_v75=arm_vector('actuator_position_target_arm'),
                arm_gravity_bias_Nm_v75=arm_vector('actuator_gravity_bias_nm'))
            return fields
        self.record.coordinate_fields=identity_fields
        self.record.metadata['prismatic_v75_command_semantics']=dict(
            command_q='nominal controller arm reference',implicit_arm_drive_target_rad_v75='actual finite-position drive target, including gravity-bias mapping',
            arm_gravity_bias_Nm_v75='authored compensation command, not measured motor torque',
            independent_motor_force_available=False,explicit_zero_effort_is_not_motor_output=True)

    def step(self):
        row=super().step()
        if self.phase in ('OPEN_CONTINUE','TRANSMISSION_PROBE') and self.probe_profile is not None and not self._decelerating:
            from xarm_wrist_ft import wrench_at
            ft=row['wrist_ft'];T=ft['T_world_sensor']
            w=wrench_at(np.asarray(ft['raw_wrench_sensor'])-self.job['ft_calibration']['bias_sensor'],T,T[:3,3])+ft['tool_gravity_wrench_world']
            stop=max(row['tcp_speed_m_s']/self.motion_limits(self.phase)['acceleration_m_s2'],
                float(np.max(abs(self.g['qvelocity'])))/self.policy.data['arm_command_acceleration_rad_s2']['LOADED'])+self.dt
            risk=self.v75_load_trend.update(row['t'],self.dt,list(row['forces_n'].values()),w,stop)
            if risk is not None and self.v75_load_stop is None:
                self.v75_load_stop=risk;self.event('PRISMATIC_PREDICTED_LOAD_MARGIN',load_margin=risk)
        return row

    def cartesian_step(self,goal,phase):
        if self.v75_load_stop is not None and phase in ('OPEN_CONTINUE','TRANSMISSION_PROBE'):
            from .prismatic_load_v75 import PrismaticLoadMarginStop
            raise PrismaticLoadMarginStop(self.v75_load_stop)
        return super().cartesian_step(goal,phase)


class PrismaticEvidenceOperationsV75(PrismaticOperationsV73):
    def request_plan(self,command):
        from .segments import validate_action
        actions=command.get('planned_segments')
        self.current_plan_ready=False
        if not isinstance(actions,list)or not actions:
            return dict(plan=False,reason='DECLARE_MINIMUM_FIT_VALIDATION_CHAIN_BEFORE_RANKING',recoverable=True)
        from agent_command_protocol_v12 import validate
        validate(dict(protocol_version=1,sequence=0,operation='attempt_grasp_and_probe',segments=actions),0)
        for action in actions:validate_action(action);self.articulation().validate_target(action)
        self.g['v75_declared_chain']=actions
        requested=command.get('native_g1_preload_request_n',self.r.closure_policy['preload_n'])
        if isinstance(requested,bool)or requested not in (.5,.75,1.,1.5):return dict(plan=False,reason='TESTED_PRELOAD_MENU_ONLY',recoverable=True)
        if self.g['recovery_state']!='UNLOADED_OBSERVATION' or self.r.supervisor.latched is not None:
            return dict(plan=False,reason='VERIFIED_UNLOADED_BOUNDARY_REQUIRED',recoverable=False)
        from .prismatic_preload_v71 import preload_policy
        self.r.closure_policy=preload_policy(self.r.closure_policy,requested);self.g['policy']=self.r.closure_policy
        self.g['v75_planned_preload']=requested
        return super().request_plan(command)

    def build_geometry(self):
        from unittest.mock import patch
        import native_g1_execution_candidates as executable
        import xarm7_native_planning as native
        from .prismatic_planning_v73 import native_precision_ik_class
        from .prismatic_candidate_v75 import PrismaticCandidateCheckV75
        # Bypass only V7.3's factory selection; its observed pipeline and plan
        # inspection are reused, frozen scores and native predicates unchanged.
        from .prismatic_preload_v71 import PreloadOperations
        with patch.object(executable,'CandidateCheck',PrismaticCandidateCheckV75),patch.object(native,'RobotPinkIK',native_precision_ik_class()):
            result=PreloadOperations.build_geometry(self)
        if result.get('plan'):
            from prismatic_inspection_v73 import plan_board
            result['plan_board']=plan_board(self.g)
        return result

    def release_park(self):
        # Original planner/executor/verification, only the existing rest cells'
        # legal joint representations are extended. Never clear a hard latch.
        from unittest.mock import patch
        import native_g1_safe_park as frozen
        from .prismatic_park_v75 import parking_targets
        original=frozen.parking_targets
        with patch.object(frozen,'parking_targets',lambda g:parking_targets(g,original)):
            return super().release_park()

    def attempt(self,command):
        if command.get('segments')!=self.g.get('v75_declared_chain'):
            return dict(executed=False,recoverable=True,reason='DECLARE_AND_REVALIDATE_CHANGED_CHAIN_BEFORE_ACQUISITION')
        if command.get('native_g1_preload_request_n',self.r.closure_policy['preload_n'])!=self.g.get('v75_planned_preload'):
            return dict(executed=False,recoverable=True,reason='RECHECK_CLOSING_CHAIN_AFTER_PRELOAD_CHANGE')
        if not self.current_plan_ready:return dict(executed=False,reason='NO_CURRENT_AUTOMATIC_PLAN',recoverable=True)
        from agent_command_protocol_v12 import atomic_json
        from .prismatic_candidate_v75 import PrismaticCandidateCheckV75
        from .prismatic_cost_v75 import admit
        candidate=json.loads((self.r.output/'visual_plan.json').read_text())
        check=PrismaticCandidateCheckV75(self.g,self.r.policy,self.r.c,self.r.solver)(candidate)
        if not check['valid']:
            self.current_plan_ready=False
            return dict(executed=False,recoverable=True,reason='CURRENT_WHOLE_CHAIN_REVALIDATION_REJECTED',planning=check)
        admission=admit(self.r,candidate,command['segments'],check)
        atomic_json(self.r.output/('CHAIN_ADMISSION_%02d.json'%len(self.g['contact_attempts'])),admission)
        if not admission['admit']:return dict(executed=False,recoverable=True,reason=admission['status'],admission=admission)
        number=len(self.g['contact_attempts'])+1
        self.g['v75_active_attempt']=number
        self.g['v75_attempt_generations'][number]=len(self.g['observation_generations'])
        self.g['v75_attempt_native_point_ids'][number]=self.acquisitions+1
        self.r.v75_load_stop=None
        try:
            result=super().attempt(command);result['chain_admission']=admission
            if len(self.g['contact_attempts'])<number:
                missing=dict(available=False,reason='NO_NEW_OPERATION_STARTED',previous_attempt_fallback=False)
                result['contact_inspection']=missing;self.g['v75_current_contact_feedback']=missing
            if any(s.get('reason')=='PREDICTED_LOAD_MARGIN_REQUIRES_UNLOAD'for s in result.get('segments',[])):
                result.update(failed_contact_candidate=True,short_success=False)
            return result
        finally:
            self.g['v75_active_attempt']=0

    def rotate_segment(self,*args,**kwargs):
        from .prismatic_load_v75 import PrismaticLoadMarginStop
        # Recheck the existing finite robot/drawer relative reference envelope
        # with ACTUAL coupled finger state at every segment boundary. A nominal
        # co-moving robot/object prediction misses inward command lead on reverse.
        from .prismatic_chain_v75 import relative_lead_check
        motion=kwargs.get('articulation');target=kwargs.get('target_q')
        if motion is not None and motion.joint_type=='prismatic':
            j=np.asarray(self.r.robot.get_joint_positions());seed=j[self.r.arm];fingers=j[self.r.fingers]
            start=self.r.actual();T=self.r.tcp().copy();M=self.r.moving().copy();bias=self.r.qtarget[self.r.arm]-seed
            boundary=dict(valid=True)
            for value in np.linspace(start,target,max(2,int(np.ceil(abs(target-start)/.001))+1)):
                D=motion.delta(float(value)-start);goal=D@T;q=self.r.solver.solve(goal,self.r.base,seed,starts=1)
                if q is None:boundary=dict(valid=False,reason='MEASURED_CHAIN_IK');break
                boundary=relative_lead_check(self.g,goal,q,D@M,[dict(finger_q=fingers)],self.r.policy,self.r.solver,bias)
                if not boundary['valid']:break
                seed=q
            from agent_command_protocol_v12 import atomic_json
            atomic_json(self.r.output/('RELATIVE_LEAD_PREFLIGHT_%02d.json'%len(self.r.record.segments)),boundary)
            if not boundary['valid']:
                self.r.stop_command()
                return dict(ok=False,status='SAFE_PARTIAL_PROGRESS',reason='CURRENT_RELATIVE_LEAD_ROUTE_UNAVAILABLE',
                    requested_target_reached=False,safely_stopped=True,physical_transmission_valid=False,
                    planning=dict(coordinate_unit='m',relative_lead=boundary),gain_native=0.,joint_coordinate=self.r.actual())
        try:return super().rotate_segment(*args,**kwargs)
        except PrismaticLoadMarginStop as error:
            # Original bounded deceleration/settling and original hard guards.
            # If any guard trips here it propagates to the original hard latch.
            self.r.stop_command();self.r.hold(.3)
            result=dict(ok=False,status='SAFE_PARTIAL_PROGRESS',reason=error.args[0],
                requested_target_reached=False,safely_stopped=True,
                physical_transmission_valid=False,fit_prefix_transmission_valid=False,
                interrupted_FIT_not_automatically_qualified=True,load_margin=error.evidence,
                planning=dict(coordinate_unit='m',original_policy_unchanged=True),
                joint_coordinate=self.r.actual(),next='guarded release, fresh Agent decision only when verified unloaded')
            self.r.record.segments[-1]['result']=result
            return result

    def feedback(self):
        attempts=self.g['contact_attempts']
        contact=dict(available=False,reason='NO_ACTUAL_ATTEMPT',previous_attempt_fallback=False)
        if attempts:
            last=attempts[-1];number=len(attempts)
            try:
                # A bounded disk barrier, not a PhysX hold or loaded Agent wait.
                terminal=(last.get('operation_status')=='SAFETY_STOP' and
                    last.get('safe_exit',{}).get('reason')=='HARD_STOP_NO_AUTOMATIC_CONTINUATION')
                if self.g['recovery_state']!='UNLOADED_OBSERVATION' and self.r.supervisor.latched is None and not terminal:
                    raise RuntimeError('POST_OPERATION_DIAGNOSTIC_REQUIRES_SAFE_BOUNDARY')
                generation=self.g['v75_attempt_generations'][number]
                identity=evidence_identity(self.r.job.get('episode_id',self.r.output.name),number,generation,
                    int(last['first_record']),int(last['last_record']))
                identity['native_point_attempt_id']=self.g['v75_attempt_native_point_ids'][number]
                self.r.record.flush();event=threading.Event();self.r.record.queue.put(event)
                if not event.wait(timeout=5):raise RuntimeError('EVIDENCE_WRITER_BARRIER_TIMEOUT')
                if self.r.record.issues:raise RuntimeError('EVIDENCE_WRITER_ERROR:'+str(self.r.record.issues))
                if self.r.contact_evidence is not None:self.r.contact_evidence.flush()
                from render_prismatic_feedback_v75 import render
                contact=render(self.r.output,identity)
                contact['terminal_hard_stop_diagnostic_only']=terminal
            except Exception as error:
                contact=dict(available=False,reason=repr(error),attempt_id=number,previous_attempt_fallback=False)
        self.g['v75_current_contact_feedback']=contact
        p=self.r.output/'model/estimate.json'
        return dict(attempts=attempts,contact_feedback=contact,
            model=json.loads(p.read_text())if p.exists()else dict(status='worker pending'),
            events=self.r.events[-30:],**self.measured_coordinate_fields())

    def decision_packet(self,seq,result=None):
        packet=super().decision_packet(seq,result)
        contact=self.g.get('v75_current_contact_feedback',dict(available=False,reason='NO_CURRENT_ATTEMPT_FEEDBACK'))
        packet.update(contact_board=contact.get('path'),contact_geometry=contact.get('contact_geometry',{}).get('path'),
            contact_evidence_identity={k:contact.get(k)for k in ('available','reason','episode_id','attempt_id','observation_generation','first_step','last_step_exclusive','simulation_interval_s','source_files')},
            current_observation_generation=len(self.g['observation_generations']),
            prior_segment_is_not_current_measurement=True)
        return packet
