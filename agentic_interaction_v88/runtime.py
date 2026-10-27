"""Opt-in simultaneous XY/arm proposal and independently governed submission."""
import numpy as np
from unittest.mock import patch
from agentic_interaction_v87.command_governor import CommandGovernor
from .kinematics import CoupledKinematics


def runtime_class(parent):
    class CoordinatedRuntime(parent):
        def __init__(self, job):
            self._coord_fields_installed = False
            self._coord_active = False
            from .assets import constructor
            with constructor(job): super().__init__(job)
            from interaction_runtime.prismatic_planning_v73 import representable_joint_bounds
            lo, hi = representable_joint_bounds(self.model.limits, self.policy.data['joint_margin_rad'])
            native = np.tile(np.asarray(self.mount['native_limits_m'], float), (2, 1))
            self.coupled = CoupledKinematics(self.solver, self.base,
                np.asarray(self.robot.get_joint_positions())[self.base_indices],
                np.vstack([native, np.column_stack([lo, hi])]),
                np.r_[np.full(2, .08), np.asarray(self.c['arm_speed_limits']) / 3])
            self.coupled_governor = CommandGovernor(self.coupled.fk, self.dt)
            self._coord_posture = np.asarray(self.robot.get_joint_positions())[self.arm].copy()
            self.g['v88_runtime'] = self
            self.record.metadata['v88_coordination'] = dict(
                policy=job['v88_coordination_policy'], native_DOF=['mobile_x', 'mobile_y'] + self.c['arm_joint_names'],
                physical_mount='existing driven XY carriage, fixed height/yaw; not wheeled hardware',
                base_and_arm_submitted_before_same_physics_step=True,
                command_governor='nonlinear world TCP FK of all nine commanded DOFs',
                actual_TCP_and_wrench_guards='original native owners unchanged',
                tool_inertia='existing segment-local measured world body COM acceleration',
                no_object_joint_commands=True, hidden_dynamics_input=False)
            self.record.metadata['v81_base']['moving_only_when_verified_unloaded'] = False
            self.record.metadata['v81_base']['loaded_authority'] = 'V88 joint solve, coupled path certificate and hard guards'

        def _install_coord_fields(self):
            if self._coord_fields_installed or not hasattr(self, 'record'): return
            previous = self.record.coordinate_fields
            def fields(row):
                q = np.asarray(row['q']); dq = np.asarray(row['joint_velocities'])
                ix = getattr(self, 'base_indices', None)
                xy = np.full(2, np.nan) if ix is None else q[ix]
                velocity = np.full(2, np.nan) if ix is None else dq[ix]
                return dict(previous(row), v88_base_joint_m=xy,
                    v88_base_velocity_m_s=velocity,
                    v88_base_command_m=np.asarray(getattr(self, 'base_goal', np.full(2, np.nan))),
                    v88_base_command_velocity_m_s=np.asarray(getattr(self, 'base_velocity', np.full(2, np.nan))),
                    v88_base_measured_xyzyaw=np.asarray(row.get('measured_base_xyzyaw', np.full(4, np.nan))),
                    v88_base_configuration_velocity_m_s=np.asarray(row.get('v88_base_configuration_velocity_m_s', np.full(2, np.nan))),
                    v88_base_configuration_acceleration_m_s2=np.asarray(row.get('v88_base_configuration_acceleration_m_s2', np.full(2, np.nan))),
                    v88_base_tracking_error_m=xy - np.asarray(getattr(self, 'base_goal', np.full(2, np.nan))),
                    v88_coordination_active=float(self._coord_active),
                    v88_arm_compensation_rad=q[self.arm] - getattr(self, '_coord_posture', q[self.arm]),
                    v88_joint_margin_rad=float(self.model.margin(q[self.arm])))
            self.record.coordinate_fields = fields
            self._coord_fields_installed = True

        def step(self):
            self._install_coord_fields()
            if hasattr(self, 'coupled'):
                from interaction_runtime.policy import SafetyStop
                lo, hi = self.mount['native_limits_m']
                if (not np.isfinite(np.r_[self.base_goal, self.base_velocity]).all()
                        or np.any(self.base_goal <= lo) or np.any(self.base_goal >= hi)
                        or np.any(abs(self.base_velocity) > .08 + 1e-12)):
                    raise SafetyStop('INVALID_NATIVE_BASE_COMMAND')
            row = super().step()
            if hasattr(self, 'coupled'):
                q = np.asarray(row['q'])[self.base_indices]
                dq = np.asarray(row['joint_velocities'])[self.base_indices]
                lo, hi = self.mount['native_limits_m']
                reason = None
                if not np.isfinite(np.r_[q, dq]).all(): reason = 'INVALID_NATIVE_BASE_STATE'
                elif np.any(q < lo) or np.any(q > hi): reason = 'NATIVE_BASE_JOINT_LIMIT'
                elif np.any(abs(dq) > .08 * (1 + 1e-6)): reason = 'NATIVE_BASE_VELOCITY'
                if reason:
                    from interaction_runtime.policy import SafetyStop
                    self.supervisor.latched = dict(reason=reason, phase=row['phase'], t=row['t'],
                        base_joint_m=q.tolist(), base_velocity_m_s=dq.tolist())
                    self.event('V88_NATIVE_BASE_HARD_STOP', evidence=self.supervisor.latched)
                    raise SafetyStop(reason, self.supervisor.latched)
            return row

        def begin_reference(self):
            super().begin_reference()
            if hasattr(self, 'coupled_governor') and not np.any(self.base_velocity) and np.max(abs(self.g['qvelocity'])) < 1e-10:
                self.coupled_governor.reset()

        def cartesian_step(self, goal, phase):
            loaded = (hasattr(self, 'coupled') and self.grasp_reference is not None
                and self.policy.phase(phase) == 'LOADED' and not self._decelerating)
            if not loaded: return super().cartesian_step(goal, phase)
            if not self._coord_active:
                self._coord_active = True; self.base_active = True
                self._coord_posture = np.asarray(self.robot.get_joint_positions())[self.arm].copy()
                self.event('V88_COORDINATED_CONTACT_STARTED', base=self.base.tolist(),
                    arm_posture=self._coord_posture.tolist(), independent_base_relocation=False)
            governor = self.coupled_governor
            self._coord_context = dict(q=np.r_[self.base_goal.copy(), self.qtarget[self.arm].copy()],
                velocity=np.r_[self.base_velocity.copy(), np.asarray(self.g['qvelocity']).copy()],
                twist=governor.previous_twist.copy(), calls=governor.calls, limited=governor.limited,
                latest=dict(governor.latest), accepted=False)
            original = self.solver.velocity_step
            def coupled_step(target, base, seed, dt):
                q = np.r_[self.base_goal, seed]
                motion = self.motion_limits(phase)
                acc = np.r_[np.full(2, motion['acceleration_m_s2']),
                    np.full(7, self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)])]
                velocity = self.coupled.velocity(target, q, self._coord_context['velocity'],
                    dt, acc, motion, self._coord_posture)
                self._coord_context['proposed_base'] = q[:2] + velocity[:2] * dt
                return q[2:] + velocity[2:] * dt
            try:
                # The inherited reference, startup, retention and bounded
                # transverse unloading execute unchanged around this joint solve.
                with patch.object(self.solver, 'velocity_step', coupled_step):
                    return super().cartesian_step(goal, phase)
            finally:
                context = self._coord_context
                if not context['accepted']:
                    self.base_goal = context['q'][:2].copy()
                    self.base_velocity = context['velocity'][:2].copy()
                    governor.previous_twist = context['twist']
                    governor.calls = context['calls']; governor.limited = context['limited']
                    governor.latest = context['latest']
                self._coord_context = None

        def _project_proposal(self, proposal, context):
            combined = getattr(self, '_coord_context', None)
            if combined is None: return super()._project_proposal(proposal, context)
            if context.get('proposal') is not None and np.array_equal(proposal, context['proposal']):
                return context['projected'], context['velocity']
            governor = self.coupled_governor
            governor.previous_twist = combined['twist'].copy()
            governor.calls = combined['calls']; governor.limited = combined['limited']
            wanted = (np.r_[combined.get('proposed_base', combined['q'][:2]), proposal] - combined['q']) / self.dt
            motion = context['motion']
            acc = np.r_[np.full(2, motion['acceleration_m_s2']),
                np.full(7, self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(context['phase'])])]
            velocity = governor.velocity(combined['q'], wanted, combined['velocity'],
                self.coupled.speeds, acc, motion)
            command = combined['q'] + velocity * self.dt
            if (np.any(command <= self.coupled.limits[:, 0]) or
                    np.any(command >= self.coupled.limits[:, 1])):
                raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:COUPLED_COMMAND_LIMIT')
            self.base_goal = command[:2].copy(); self.base_velocity = velocity[:2].copy()
            context.update(proposal=np.asarray(proposal).copy(), projected=command[2:].copy(), velocity=velocity[2:].copy())
            return context['projected'], context['velocity']

        def check_q(self, q):
            if not self._coord_active: return super().check_q(q)
            if self.model.margin(q) <= self.policy.data['joint_margin_rad']: return False, 'JOINT_MARGIN'
            fingers = np.asarray(self.robot.get_joint_positions())[self.fingers]
            poses = self.model.poses(q, self.coupled.base(self.base_goal), finger_q=fingers)
            return self.collision.check(poses, self.moving(), self.intentional_contact())

        def enrich_measurement(self, row, pre_contact_poses):
            result = super().enrich_measurement(row, pre_contact_poses)
            if hasattr(self, 'coupled'):
                # Both the rail clock and compact record must see the combined
                # world command, not arm FK at a previous measured station.
                row['command_TCP'] = self.coupled.fk(np.r_[self.base_goal, self.qtarget[self.arm]])
                xy = np.asarray(row['q'])[self.base_indices].copy()
                previous = getattr(self, '_coord_previous_base_sample', None)
                if previous is not None and abs(row['t'] - previous[0] - self.dt) < 1e-7:
                    velocity = (xy - previous[1]) / self.dt
                    row['v88_base_configuration_velocity_m_s'] = velocity
                    if previous[2] is not None:
                        row['v88_base_configuration_acceleration_m_s2'] = (velocity - previous[2]) / self.dt
                else: velocity = None
                self._coord_previous_base_sample = (row['t'], xy, velocity)
                context = getattr(self, '_coord_context', None)
                if context is not None: context['accepted'] = True
            return result

        def stop_command(self):
            if not self._coord_active: return super().stop_command()
            from interaction_runtime.policy import SafetyStop
            requested = self.g['phase']
            phase = self._v87_last_command_phase or requested
            motion = self._v87_last_command_motion or self.motion_limits(phase)
            self.g['phase'] = phase; self._decelerating = True
            acc = np.r_[np.full(2, motion['acceleration_m_s2']),
                np.full(7, self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)])]
            try:
                for _ in range(int(5 / self.dt)):
                    velocity = np.r_[self.base_velocity, self.g['qvelocity']]
                    if np.max(abs(velocity)) <= 1e-12: return
                    q = np.r_[self.base_goal, self.qtarget[self.arm]]
                    v = self.coupled_governor.velocity(q, np.zeros(9), velocity,
                        self.coupled.speeds, acc, motion)
                    command = q + v * self.dt
                    self.base_goal = command[:2]; self.base_velocity = v[:2]
                    okay, why = self.check_q(command[2:])
                    if not okay: raise SafetyStop('STOP_ROUTE_COLLISION:' + why)
                    self.qtarget[self.arm] = command[2:]; self.g['qvelocity'] = v[2:]
                    self.step()
                raise SafetyStop('COUPLED_CONTROLLED_STOP_NOT_VERIFIED')
            finally:
                self._decelerating = False; self.g['phase'] = requested

        def move_path(self, *args, **kwargs):
            if self._coord_active:
                self.stop_command()
                self._coord_active = False; self.base_active = False
                self.base_generation += 1
                self.event('V88_COORDINATED_CONTACT_ENDED', measured_base=self.base.tolist())
            return super().move_path(*args, **kwargs)

    return CoordinatedRuntime
