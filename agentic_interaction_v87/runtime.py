"""Final-command rate governance; original native drive and safety stay owners."""
import numpy as np
from .command_governor import CommandGovernor


def runtime_class(parent):
    class GovernedRuntime(parent):
        def __init__(self,job):
            super().__init__(job)
            from interaction_reference_handover import seed_pose
            self.command_governor=CommandGovernor(lambda q:seed_pose(self.solver,self.base,q),self.dt)
            self._v87_cartesian_active=False;self._v87_fixture_done=False
            self._v87_last_command_phase=None
            self._v87_last_command_motion=None
            from .constraint_feedback import enabled, ActionForceMonitor
            if enabled(self):
                self.supervisor.force = ActionForceMonitor(self.policy, self, previous=self.supervisor.force)
                self.record.metadata['v87_constraint_policy'] = dict(
                    policy=job['v87_constraint_policy'], emergency_limits_unchanged=True,
                    sustained_band_authority='bounded stop/unload, never continued pulling',
                    frozen_policy_bytes_unchanged=True)
            from .recovery import annotate_interrupted_end
            self.record.end=annotate_interrupted_end(self.record.end)
            previous=self.record.coordinate_fields
            def fields(row):
                result=previous(row);m=self.command_governor.latest
                feedback=getattr(self.supervisor.force,'feedback',{})
                result.update(v87_constraint_speed_scale=float(feedback.get('scale',1.)),
                    v87_constraint_unload_required=bool(feedback.get('unload_required',False)))
                result.update(v87_command_linear_speed_m_s=float(m.get('linear_speed_m_s',np.nan)),
                    v87_command_linear_acceleration_m_s2=float(m.get('linear_acceleration_m_s2',np.nan)),
                    v87_command_governor_limited=bool(m.get('limited',False)),
                    v87_command_governor_valid=bool(m))
                return result
            self.record.coordinate_fields=fields
            self.record.metadata['v87_command_governor']=dict(
                scope='nonlinear FK of final arm command, joint/TCP velocity and acceleration',
                unchanged_native_drive_and_hard_limits=True,feeds_Physics_ID=False)

        def step(self):
            row = super().step()
            from .constraint_feedback import enabled, require_recovery
            if enabled(self):
                row['constraint_feedback'] = dict(getattr(self.supervisor.force, 'feedback', {}))
                require_recovery(self)
            return row

        def begin_reference(self):
            super().begin_reference()
            if hasattr(self,'command_governor') and np.max(abs(self.g['qvelocity']))<1e-10:
                self.command_governor.reset()

        def _govern(self,wanted,phase,motion=None):
            acc=self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)]
            return self.command_governor.velocity(self.qtarget[self.arm],wanted,
                np.asarray(self.g['qvelocity']),np.asarray(self.c['arm_speed_limits'])/3,
                acc,self.motion_limits(phase)if motion is None else motion)

        def cartesian_step(self,goal,phase):
            self.ensure_execution_budget(phase);self.g['phase']=phase;self.phase=phase
            if self.reference_drive is None:self.begin_reference()
            self._v87_cartesian_active=True
            if self.policy.phase(phase)=='LOADED' and not self._decelerating:
                hard=self.policy.motion(phase)['measured_speed_stop_m_s']
                measured=float(self.rows[-1].get('tcp_speed_m_s',0.))
                from .constraint_feedback import enabled
                if not enabled(self) and measured>=.9*hard:
                    self.event('V87_CONTROLLED_INTERRUPT',reason='MEASURED_SPEED_APPROACHING_HARD_LIMIT',
                        measured_tcp_speed_m_s=measured,hard_limit_unchanged_m_s=hard)
                    raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:MEASURED_SPEED_APPROACHING_HARD_LIMIT')
                fixture=self.job.get('v87_controlled_interruption_fixture')
                if fixture and not self._v87_fixture_done and self.record.segments:
                    segment=self.record.segments[-1]['id']
                    if getattr(self,'_v87_fixture_segment',None)!=segment:
                        self._v87_fixture_segment=segment;self._v87_fixture_q=float(self.actual())
                    if abs(self.actual()-self._v87_fixture_q)>=fixture['progress_native']:
                        self._v87_fixture_done=True
                        self.event('V87_ENGINEERING_CONTROLLED_INTERRUPT',engineering_injected=True,
                            measured_coordinate_native=float(self.actual()))
                        raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:REGISTERED_ENGINEERING_FIXTURE')
            if np.max(abs(np.asarray(goal)-self.reference_drive.pose))<1e-12 and np.max(abs(self.g['qvelocity']))<1e-10:
                return self.step()
            motion=self.motion_limits(phase)
            # Feedback slows an ordinary tracking deviation before a controlled
            # interruption is needed; it does not replace measured hard guards.
            measured=float(self.rows[-1].get('tcp_speed_m_s',0.))
            from .constraint_feedback import enabled
            if not enabled(self) and measured>1.5*motion['speed_m_s']:
                motion=dict(motion,speed_m_s=motion['speed_m_s']**2/measured)
            from .constraint_feedback import motion as feedback_motion, require_recovery
            if enabled(self):
                require_recovery(self)
                motion=feedback_motion(self,motion)
            target=self.reference_drive.advance(goal,self.dt,motion)
            with self.profile.measure('Pink_IK'):
                q=self.solver.velocity_step(target,self.base,self.seed,self.dt)
            if q is None:raise RuntimeError('LOCAL_IK_UNAVAILABLE')
            wanted=(q+self.reference_bias-self.qtarget[self.arm])/self.dt
            previous_twist=self.command_governor.previous_twist.copy()
            governed_motion=dict(self.motion_limits(phase))
            with self.profile.measure('v87_final_command_governor'):
                velocity=self._govern(wanted,phase,governed_motion)
            command=self.qtarget[self.arm]+velocity*self.dt
            if self.model.margin(command)<=self.policy.data['joint_margin_rad']:
                self.command_governor.previous_twist=previous_twist
                raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:COMMAND_JOINT_MARGIN_REJECTED')
            if self.tick%8==0 or phase in ('PANEL_APPROACH','PANEL_PUSH','PANEL_SEPARATION'):
                with self.profile.measure('command_collision_queries'):ok,why=self.check_q(command)
                if not ok:
                    self.command_governor.previous_twist=previous_twist
                    raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:COMMAND_PATH_REJECTED:'+why)
            self.qtarget[self.arm]=command;self.g['qvelocity']=velocity
            self._v87_last_command_phase=phase
            self._v87_last_command_motion=governed_motion
            self.seed=command-self.reference_bias
            return self.step()

        def stop_command(self):
            if not getattr(self,'_v87_cartesian_active',False):return super().stop_command()
            # The release owner requests RELEASE before calling stop(). Its
            # lower speed ceiling cannot be imposed instantaneously on motion
            # accepted in LOADED. Brake in the last accepted phase first; only
            # a stopped command may enter the lower-speed release phase.
            requested_phase=self.g['phase']
            braking_phase=self._v87_last_command_phase or requested_phase
            braking_motion=self._v87_last_command_motion
            if requested_phase!=braking_phase:
                self.event('V87_PHASE_TRANSITION_BRAKE',braking_phase=braking_phase,
                    requested_next_phase=requested_phase,limits=braking_motion,
                    policy_thresholds_unchanged=True)
            self.g['phase']=braking_phase
            self._decelerating=True
            try:
                for _ in range(int(5/self.dt)):
                    if np.max(abs(self.g['qvelocity']))<=1e-12:return
                    v=self._govern(np.zeros(7),braking_phase,braking_motion)
                    q=self.qtarget[self.arm]+v*self.dt
                    ok,why=self.check_q(q)
                    if not ok:
                        from interaction_runtime.policy import SafetyStop
                        raise SafetyStop('STOP_ROUTE_COLLISION:'+why)
                    self.qtarget[self.arm]=q;self.g['qvelocity']=v;self.step()
                from interaction_runtime.policy import SafetyStop
                raise SafetyStop('CONTROLLED_STOP_NOT_VERIFIED')
            finally:
                self._decelerating=False
                self.g['phase']=requested_phase

        def move_path(self,*args,**kwargs):
            self._v87_cartesian_active=False
            result=super().move_path(*args,**kwargs)
            if hasattr(self,'command_governor'):self.command_governor.reset()
            return result

    return GovernedRuntime
