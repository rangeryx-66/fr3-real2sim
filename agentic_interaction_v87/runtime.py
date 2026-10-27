"""Final-command rate governance; original native drive and safety stay owners."""
import numpy as np
from unittest.mock import patch
from .command_governor import CommandGovernor


def runtime_class(parent):
    class GovernedRuntime(parent):
        def __init__(self,job):
            self._v87_diagnostics_installed=False
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
                    sustained_band_authority='bounded continuous reference braking; measured stall/resource returns to Agent',
                    frozen_policy_bytes_unchanged=True)
            from .recovery import annotate_interrupted_end
            self.record.end=annotate_interrupted_end(self.record.end)
            self._install_diagnostics()
            if enabled(self):
                from .motion_policy import PhysicalMotionDwell, articulation_prior
                self.station_motion=PhysicalMotionDwell(self.dt,lambda:articulation_prior(self))
                if hasattr(self,'v75_load_trend'):
                    from .constraint_feedback import PredictiveLoadRegulator
                    self.v75_load_trend=PredictiveLoadRegulator(self,self.v75_load_trend)
            self.record.metadata['v87_command_governor']=dict(
                scope='nonlinear FK of final arm command, joint/TCP velocity and acceleration',
                unchanged_native_drive_and_hard_limits=True,feeds_Physics_ID=False)

        def _install_diagnostics(self):
            if getattr(self,'_v87_diagnostics_installed',False) or not hasattr(self,'record'):
                return
            from .motion_policy import diagnostic_fields
            previous=self.record.coordinate_fields
            def fields(row):
                return dict(previous(row),**diagnostic_fields(self))
            self.record.coordinate_fields=fields
            self._v87_diagnostics_installed=True

        def step(self):
            # Mobile constructor performs native admission steps before this
            # outer constructor returns. Keys must exist on its FIRST row.
            self._install_diagnostics()
            context=getattr(self,'_v87_submission',None)
            if context is not None and not context['submitted']:
                command,velocity=self._project_proposal(self.qtarget[self.arm],context)
                phase=context['phase']
                if self.tick%8==0 or phase in ('PANEL_APPROACH','PANEL_PUSH','PANEL_SEPARATION'):
                    okay,why=context['original_check'](command)
                    if not okay:
                        raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:COMMAND_PATH_REJECTED:'+why)
                self.qtarget[self.arm]=command;self.g['qvelocity']=velocity
                self.seed=command-self.reference_bias
                self._v87_last_command_phase=phase;self._v87_last_command_motion=context['motion']
                context['submitted']=True
            row=super().step()
            row['constraint_feedback']=dict(getattr(self.supervisor.force,'feedback',{}))
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

        def motion_limits(self,phase):
            limits=super().motion_limits(phase)
            from .constraint_feedback import enabled,motion
            if (getattr(self,'_v87_submission',None) is not None and enabled(self)
                    and self.policy.phase(phase)=='LOADED' and not self._decelerating):
                return motion(self,limits)
            return limits

        def _project_proposal(self,proposal,context):
            proposal=np.asarray(proposal,float)
            if context.get('proposal') is not None and np.array_equal(proposal,context['proposal']):
                return context['projected'],context['velocity']
            governor=self.command_governor
            governor.previous_twist=context['twist'].copy()
            governor.calls=context['calls'];governor.limited=context['limited']
            wanted=(proposal-context['q'])/self.dt
            acceleration=self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(context['phase'])]
            velocity=governor.velocity(context['q'],wanted,context['previous_velocity'],
                np.asarray(self.c['arm_speed_limits'])/3,acceleration,context['motion'])
            command=context['q']+velocity*self.dt
            if self.model.margin(command)<=self.policy.data['joint_margin_rad']:
                raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:COMMAND_JOINT_MARGIN_REJECTED')
            context.update(proposal=proposal.copy(),projected=command,velocity=velocity)
            return command,velocity

        def _restore_unsubmitted(self,context):
            if context.get('submitted'):return
            self.qtarget[self.arm]=context['q']
            self.g['qvelocity']=context['previous_velocity'].copy()
            self.seed=context['seed']
            self.command_governor.previous_twist=context['twist'].copy()
            self.command_governor.calls=context['calls'];self.command_governor.limited=context['limited']
            self.command_governor.latest=context['latest'].copy()

        def cartesian_step(self,goal,phase):
            self.ensure_execution_budget(phase);self._v87_cartesian_active=True
            from .constraint_feedback import enabled,action_fault
            if self.policy.phase(phase)=='LOADED' and not self._decelerating:
                if (not enabled(self) and float(self.rows[-1].get('tcp_speed_m_s',0.))
                        >=.9*self.policy.motion(phase)['measured_speed_stop_m_s']):
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
            # Keep the actual inherited rail/revolute reference, retention,
            # bias, anti-windup and IK owners. Govern only their final proposal.
            context=dict(q=self.qtarget[self.arm].copy(),previous_velocity=np.asarray(self.g['qvelocity']).copy(),
                twist=self.command_governor.previous_twist.copy(),calls=self.command_governor.calls,
                limited=self.command_governor.limited,phase=phase,motion=super().motion_limits(phase),
                seed=None if self.seed is None else np.asarray(self.seed).copy(),
                latest=dict(self.command_governor.latest),submitted=False,original_check=self.check_q)
            self._v87_submission=context
            def checked(proposal,*args,**kwargs):
                command,_=self._project_proposal(proposal,context)
                return context['original_check'](command,*args,**kwargs)
            try:
                with patch.object(self,'check_q',checked):
                    return super().cartesian_step(goal,phase)
            except Exception as error:
                self._restore_unsubmitted(context)
                from interaction_runtime.policy import SafetyStop
                if isinstance(error,SafetyStop):
                    normalized=action_fault(self,error)
                    if normalized is not error and normalized is not None:raise normalized from error
                raise
            finally:
                self._restore_unsubmitted(context)
                self._v87_submission=None

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
