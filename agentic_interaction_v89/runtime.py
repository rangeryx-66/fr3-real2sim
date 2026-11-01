"""Explicit replacement of V87/V88 online governance, reference IK and stop.

ABC, native scene, wrist sensing, contact physics, measured emergency
supervision and segment-local Physics-ID are inherited. No solver/check_q
patch, governor, nonlinear projection or iterative stop preview is installed.
"""
import time
from types import SimpleNamespace
import numpy as np

from .controller import Bounds, CommandOwner, StopState, WeightedDLS, pose_error
from .kinematics import WorldKinematics


def runtime_class(parent):
    class DLSRuntime(parent):
        def __init__(self, job):
            self.command_owner = None
            self._v89_fields_installed = False
            from agentic_interaction_v88.assets import constructor
            with constructor(job):
                super().__init__(job)
            # Isaac readback is float32. The submitted nominal command is a
            # float64 integration state; do not round it on every assignment or
            # mistake the rounding residual for a second external proposal.
            self.qtarget = np.asarray(self.qtarget, dtype=np.float64).copy()
            self.g['qtarget'] = self.qtarget
            from interaction_runtime.prismatic_planning_v73 import representable_joint_bounds
            low, high = representable_joint_bounds(self.model.limits, self.policy.data['joint_margin_rad'])
            limits = np.vstack([np.tile(self.mount['native_limits_m'], (2, 1)), np.c_[low, high]])
            self.coupled = WorldKinematics(self.solver, self.base,
                np.asarray(self.robot.get_joint_positions())[self.base_indices], limits,
                np.r_[.08, .08, np.asarray(self.c['arm_speed_limits']) / 3])
            self.kernel = WeightedDLS(self.coupled)
            self.command_owner = CommandOwner(self.coupled, self._requested_position(), self.dt)
            self._last_bounds = self._bounds(self.g['phase'])
            self._last_phase = self.g['phase']
            self._v89_mobile = False
            self._v89_controlled_fixture_done = False
            self._last_proposal = None
            self.g['v88_runtime'] = self
            # Explicit instrumentation only; no implicit solver substitution.
            from agentic_interaction_v87.constraint_feedback import ActionForceMonitor
            self.supervisor.force = ActionForceMonitor(self.policy, self, previous=self.supervisor.force)
            if hasattr(self, 'v75_load_trend'):
                from agentic_interaction_v87.constraint_feedback import PredictiveLoadRegulator
                self.v75_load_trend = PredictiveLoadRegulator(self, self.v75_load_trend)
            from agentic_interaction_v87.recovery import annotate_interrupted_end
            self.record.end = annotate_interrupted_end(self.record.end)
            self.record.metadata['v89_controller'] = dict(
                controller=job['v89_controller'], dt_s=self.dt,
                command_owner='CommandOwner -> NativeDrive.apply_coordinated (XY+arm+G1)',
                IK='weighted DLS, 6x6 solve; gain=6/s; dimensionless damping=.03; arm_cost=4',
                orientation='world Log(R_goal R_current.T), radians; pose7 quaternion xyzw',
                stop='common-scalar analytic joint-velocity ray, certified at acceptance; measured dwell before release',
                old_governor_installed=False, old_cartesian_reference_limiter=False,
                no_object_commands=True, actuator_authority_unchanged=True)
            self.record.metadata['v81_base']['moving_only_when_verified_unloaded'] = False
            self.record.metadata['v81_base']['loaded_authority'] = 'single transactional DLS owner'
            self._install_fields()

        def _requested_position(self):
            return np.r_[self.base_goal, self.qtarget[self.arm]]

        def _measured_position(self):
            q = np.asarray(self.robot.get_joint_positions())
            return np.r_[q[self.base_indices], q[self.arm]]

        def _bounds(self, phase):
            m = dict(self.policy.motion(phase))
            action = getattr(self, 'probe_profile', None)
            if action and self.policy.phase(phase) == 'LOADED':
                m['acceleration_m_s2'] = min(m['acceleration_m_s2'],
                    action.get('reference_acceleration_m_s2', m['acceleration_m_s2']))
            # Action speed/acceleration and warning adaptation set the reference
            # demand, never instantaneously shrink final-command authority.
            return Bounds(self.coupled.limits, self.coupled.speeds,
                np.r_[np.full(2, m['acceleration_m_s2']),
                      np.full(7, self.policy.data['arm_command_acceleration_rad_s2'][self.policy.phase(phase)])],
                m['speed_m_s'], m['angular_speed_rad_s'],
                m['acceleration_m_s2'], m['angular_acceleration_rad_s2'])

        def _geometry(self):
            fingers = np.asarray(self.robot.get_joint_positions())[self.fingers].copy()
            moving = self.moving().copy()
            allow = self.intentional_contact()
            def check(q):
                with self.profile.measure('V89_command_collision'):
                    return self.collision.check(self.model.poses(q[2:], self.coupled.base(q[:2]),
                        finger_q=fingers), moving, allow)
            return check

        def _install_fields(self):
            if self._v89_fields_installed or not hasattr(self, 'record'):
                return
            previous = self.record.coordinate_fields
            def fields(row):
                owner = self.command_owner
                xy = np.asarray(row['q'])[self.base_indices] if hasattr(self, 'base_indices') else np.full(2, np.nan)
                p = getattr(self, '_last_proposal', None)
                return dict(previous(row),
                    v88_base_joint_m=xy,
                    v88_base_velocity_m_s=np.asarray(row['joint_velocities'])[self.base_indices] if hasattr(self, 'base_indices') else np.full(2, np.nan),
                    v88_base_command_m=np.asarray(getattr(self, 'base_goal', np.full(2, np.nan))),
                    v88_base_command_velocity_m_s=np.asarray(getattr(self, 'base_velocity', np.full(2, np.nan))),
                    v89_command_position=np.full(9, np.nan) if owner is None else owner.command.position,
                    v89_command_velocity=np.full(9, np.nan) if owner is None else owner.command.velocity,
                    v89_command_twist=np.full(6, np.nan) if owner is None else owner.command.twist,
                    v89_stop_state=float(list(StopState).index(owner.state)) if owner else -1.,
                    v89_command_zero=float(owner.command_zero) if owner else 1.,
                    v89_measured_stationary=float(owner.measured_stationary) if owner else 0.,
                    v89_solve_s=0. if p is None else p.solve_s,
                    v89_stop_check_s=0. if p is None else p.stop_s)
            self.record.coordinate_fields = fields
            self._v89_fields_installed = True

        def _write_command(self):
            c = self.command_owner.command
            self.base_goal, self.base_velocity = c.position[:2].copy(), c.velocity[:2].copy()
            self.qtarget[self.arm] = c.position[2:]
            self.g['qvelocity'] = c.velocity[2:].copy()

        def submit_drive(self):
            # Called once before each World.step by the base physics owner.
            # No separate XY hook or second arm writer exists in this profile.
            self.controller.apply_coordinated(self.base_goal, self.base_velocity,
                self.qtarget[self.arm], np.asarray(self.g['qvelocity']), self.jaw_width)

        def step(self):
            self._install_fields()
            owner = self.command_owner
            if owner is not None:
                # Timed free-space/parking/station paths enter the SAME owner.
                request = self._requested_position()
                if not np.array_equal(request, owner.command.position):
                    wanted = (request - owner.command.position) / self.dt
                    p = owner.propose(wanted, self._bounds(self.g['phase']), self._geometry())
                    self._last_proposal = p
                    if not p.accepted:
                        self._write_command()
                        self.stop_command()
                        raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:' + p.reason)
                    owner.commit(p)
                    self._last_bounds = self._bounds(self.g['phase'])
                    self._last_phase = self.g['phase']
                self._write_command()
            row = super().step()
            if owner is not None:
                q = np.asarray(row['q'])[self.base_indices]
                dq = np.asarray(row['joint_velocities'])[self.base_indices]
                lo, hi = self.mount['native_limits_m']
                reason = ('INVALID_NATIVE_BASE_STATE' if not np.isfinite(np.r_[q, dq]).all() else
                    'NATIVE_BASE_JOINT_LIMIT' if np.any(q < lo) or np.any(q > hi) else
                    'NATIVE_BASE_VELOCITY' if np.any(abs(dq) > .08 * (1 + 1e-6)) else None)
                if reason:
                    from interaction_runtime.policy import SafetyStop
                    self.supervisor.latched = dict(reason=reason, phase=row['phase'], t=row['t'])
                    raise SafetyStop(reason, self.supervisor.latched)
            return row

        def begin_reference(self):
            if self.command_owner is None:
                return super().begin_reference()
            measured = self._measured_position()
            self.seed = measured[2:].copy()
            self.reference_bias = self.qtarget[self.arm] - self.seed
            self.reference_drive = SimpleNamespace(pose=self.coupled.fk(measured))
            self._reference_bias_world = pose_error(self.coupled.fk(self.command_owner.command.position),
                                                   self.reference_drive.pose)
            self._reference_previous = self.reference_drive.pose.copy()
            self.integral_limited = False
            if hasattr(self, 'retention'):
                self.retention_epoch = self.retention.offset.copy() if self.retention is not None else np.zeros(3)
            if hasattr(self, 'unloading'):
                self.unloading_epoch = self.unloading.offset.copy() if self.unloading is not None else np.zeros(3)
            self.event('V89_REFERENCE_HANDOVER', command_preserved=True)

        def motion_limits(self, phase):
            limits = super().motion_limits(phase)
            if self.policy.phase(phase) == 'LOADED' and not self._decelerating:
                from agentic_interaction_v87.constraint_feedback import motion
                return motion(self, limits)
            return limits

        def _contact_goal(self, goal, phase):
            # Keep existing bounded contact centering, explicitly before IK.
            if hasattr(self, 'retention_goal'):
                goal = self.retention_goal(goal, phase)
            if (getattr(self, 'transverse_unloading_enabled', False)
                    and self.grasp_reference is not None and self.policy.phase(phase) == 'LOADED'):
                from interaction_runtime.prismatic_unloading_v751 import TransverseUnloading, causal_world_wrench
                if self.unloading is None:
                    c = self.unloading_closure
                    if c is None:
                        raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:MISSING_CURRENT_CLOSURE')
                    self.unloading = TransverseUnloading(self.closure_policy, self.contact_evidence.axis,
                        self.filtered_loads.maxlen, c.command[:3, 3] - c.origin[:3, 3])
                rows = list(self.rows)[-self.filtered_loads.maxlen:]
                force = np.mean([causal_world_wrench(r, self.job['ft_calibration']['bias_sensor'])[:3] for r in rows], axis=0)
                centres = []
                for native, logical in [('left_finger', 'gripper_link1'), ('right_finger', 'gripper_link2')]:
                    T = self.pose(native) @ self.pad_frames[native]
                    centres.append((T @ np.r_[self.model.pads[logical].mean(0), 1])[:3])
                update = self.unloading.update(self.tick, self.dt, force, centres)
                if update is not None:
                    self.event('V89_TRANSVERSE_UNLOADING', **update)
                goal = np.array(goal, copy=True)
                goal[:3, 3] += self.unloading.offset - self.unloading_epoch
            return goal

        def cartesian_step(self, goal, phase):
            if self.command_owner is None:
                return super().cartesian_step(goal, phase)
            self.ensure_execution_budget(phase)
            self.g['phase'] = self.phase = phase
            if self.reference_drive is None:
                self.begin_reference()
            goal = self._contact_goal(goal, phase)
            bounds = self._bounds(phase)
            m = self.motion_limits(phase)
            # Zero soft demand decelerates through the same owner. It is not an
            # illegal zero actuator bound and does not interrupt the segment.
            measured = self._measured_position()
            self._v89_mobile = self.grasp_reference is not None and self.policy.phase(phase) == 'LOADED'
            self.base_active = self._v89_mobile
            demand = Bounds(bounds.position, bounds.speed, bounds.acceleration,
                max(m['speed_m_s'], 1e-12), max(m['angular_speed_rad_s'], 1e-12),
                bounds.linear_acceleration, bounds.angular_acceleration)
            current = self.coupled.fk(measured)
            deficit = pose_error(self.coupled.fk(self.command_owner.command.position), current) - self._reference_bias_world
            allowance = bounds.linear_speed**2 / (2 * bounds.linear_acceleration) + bounds.linear_speed * self.dt
            self.integral_limited = np.linalg.norm(deficit[:3]) >= allowance and deficit[:3] @ pose_error(goal, current)[:3] > 0
            fixture = self.job.get('v87_controlled_interruption_fixture')
            if fixture and not self._v89_controlled_fixture_done and self.record.segments:
                segment = self.record.segments[-1]['id']
                if getattr(self, '_fixture_segment', None) != segment:
                    self._fixture_segment, self._fixture_start = segment, float(self.actual())
                if abs(self.actual() - self._fixture_start) >= fixture['progress_native']:
                    self._v89_controlled_fixture_done = True
                    self.stop_command()
                    raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:REGISTERED_ENGINEERING_FIXTURE')
            began = time.perf_counter()
            try:
                wanted = (np.zeros(9) if self.integral_limited else
                    self.kernel.velocity(goal, measured, demand, mobile=self._v89_mobile))
                solve_s = time.perf_counter() - began
                with self.profile.measure('V89_command_and_stop'):
                    p = self.command_owner.propose(wanted, bounds, self._geometry())
                from dataclasses import replace
                p = replace(p, solve_s=solve_s)
            except (ValueError, np.linalg.LinAlgError) as error:
                from .controller import Proposal
                p = Proposal(None, None, 'NUMERICAL_IK:' + str(error), self.command_owner.command.generation)
            self._last_proposal = p
            if not p.accepted:
                self.event('V89_CANDIDATE_REJECTED', reason=p.reason, committed_generation=self.command_owner.command.generation,
                    measured_hard_event=False, proposal_submitted=False)
                self.stop_command()  # consume prior certified ray, never retry failed IK
                raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:' + p.reason)
            self.command_owner.commit(p)
            self._last_bounds = bounds
            self._last_phase = phase
            self._write_command()
            self.reference_drive.pose = np.array(goal, copy=True)
            self.seed = self.qtarget[self.arm] - self.reference_bias
            return self.step()

        def stop_command(self):
            owner = self.command_owner
            if owner is None:
                return super().stop_command()
            from interaction_runtime.policy import SafetyStop
            if self.supervisor.latched is not None:
                raise SafetyStop('LATCHED_' + self.supervisor.latched['reason'], self.supervisor.latched)
            requested = self.g['phase']
            self.g['phase'] = self.phase = self._last_phase
            # Brake under the accepted phase's bounds. Release is entered only
            # once the measured TCP and both joint groups are quiet.
            self._decelerating = True
            began = time.perf_counter()
            try:
                commands = owner.start_stop()
                self.event('V89_STOP_BRAKING', command_zero=owner.command_zero, measured_stationary=False,
                    steps=len(commands))
                check = self._geometry()
                for command in commands:
                    valid, reason = check(command.position)
                    if not valid:
                        owner.state = StopState.BLOCKED
                        raise RuntimeError('STOP_EXECUTION_GEOMETRY_CHANGED:' + str(reason))
                    owner.commit_brake(command)
                    self._write_command()
                    self.step()
                owner.settling()
                for _ in range(math_ceil(2. / self.dt)):
                    self._write_command()
                    row = self.step()
                    dq = np.asarray(row['joint_velocities'])
                    object_speed = row['joint_coordinate_velocity']
                    if self.job['articulation_v7']['joint_type'] == 'revolute':
                        from interaction_runtime.progress import hinge_radius
                        p = self.g['v75_motion_prior'] if 'v75_motion_prior' in self.g else None
                        if p is not None:
                            object_speed *= hinge_radius(row['T_tcp'][:3, 3], p['origin_world_m'], p['axis_world'])
                    if owner.observe_stop(np.r_[dq[self.base_indices], dq[self.arm]], row['tcp_speed_m_s'], object_speed):
                        self.event('V89_STOP_CONFIRMED', command_zero=True, measured_stationary=True,
                            wall_s=time.perf_counter() - began)
                        return
                # Motion is held and supervised, but no release or new B is
                # authorized. A bounded settling failure is not a hard hazard.
                self.event('V89_STOP_PENDING', command_zero=True, measured_stationary=False)
                from .controller import MeasuredStopPending
                raise MeasuredStopPending()
            finally:
                self._decelerating = False
                self.g['phase'] = self.phase = requested

        def enrich_measurement(self, row, pre_contact_poses):
            result = super().enrich_measurement(row, pre_contact_poses)
            if hasattr(self, 'coupled'):
                row['command_TCP'] = self.coupled.fk(self._requested_position())
            return result

        def move_path(self, *args, **kwargs):
            if self.command_owner is not None:
                self.stop_command()
                if self._v89_mobile:
                    self.base_generation += 1
                self._v89_mobile = self.base_active = False
                self.reference_drive = None
            return super().move_path(*args, **kwargs)

    return DLSRuntime


def math_ceil(value):
    import math
    return math.ceil(value)
