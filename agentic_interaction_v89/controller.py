"""World-TCP velocity IK and one transactional nine-DOF command owner.

The weighted DLS algebra is adapted from HiddenKinematics control_ik at
d0318abec626fc8416f194619f21fbe7145e2ebf (MIT; third_party_licenses/
HiddenKinematics.txt). Its action convention, timestep-free joint deltas,
quaternion mismatch and finger drives are NOT carried over.

q = [XY metres, seven arm radians]. TCP is the manufacturer's tcp_link.
All twists/errors are world expressed, linear then angular; orientation
error is Log(R_goal R_current.T), in radians. Quaternions enter only through
pose_xyzw. Commands integrate VELOCITIES with explicit seconds, never measured
positions. Rejected proposals cannot change the committed command or stop.
"""
from dataclasses import dataclass
from enum import Enum
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v81.motion_dwell import STOP_DWELL_S


def pose_xyzw(position_m, quaternion_xyzw):
    p = np.asarray(position_m, float)
    q = np.asarray(quaternion_xyzw, float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all():
        raise ValueError('FINITE_POSITION_AND_XYZW_QUATERNION_REQUIRED')
    if np.linalg.norm(q) < 1e-12:
        raise ValueError('ZERO_QUATERNION')
    T = np.eye(4)
    T[:3, 3] = p
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    return T


def pose_error(goal, current):
    goal, current = np.asarray(goal, float), np.asarray(current, float)
    if goal.shape != (4, 4) or current.shape != (4, 4) or not np.isfinite([goal, current]).all():
        raise ValueError('FINITE_WORLD_TCP_TRANSFORMS_REQUIRED')
    return np.r_[goal[:3, 3] - current[:3, 3],
                 Rotation.from_matrix(goal[:3, :3] @ current[:3, :3].T).as_rotvec()]


def _freeze(a):
    a = np.array(a, dtype=float, copy=True)
    a.setflags(write=False)
    return a


def _fraction(vector, bound):
    return min(1., float(np.min(np.asarray(bound) / np.maximum(abs(vector), 1e-30))))


class StopState(str, Enum):
    RUNNING = 'RUNNING'
    BRAKING = 'BRAKING'
    SETTLING = 'SETTLING'
    STOPPED = 'STOPPED'
    BLOCKED = 'BLOCKED'


class MeasuredStopPending(RuntimeError):
    """Command is zero, measurements do not yet authorize release or restart."""
    def __init__(self):
        super().__init__('CONTROLLED_EXECUTION_INTERRUPT:MEASURED_STOP_PENDING')


class MeasuredMotion:
    """Consecutive native configuration motion; projected dq stays separate.

    PhysX constraint stabilization can leave projected dq inconsistent with
    integrated poses. This causal observation follows the project's existing
    MotionDwell convention, extended to XY and TCP orientation. It does not
    replace any native-velocity emergency supervision.
    """
    def __init__(self, dt):
        self.dt, self.previous, self.latest = dt, None, None

    def update(self, t, q9, tcp, coordinate):
        current = (float(t), np.asarray(q9, float).copy(), np.asarray(tcp, float).copy(), float(coordinate))
        result = None
        if self.previous is not None:
            t0, q0, T0, x0 = self.previous
            elapsed = t - t0
            if abs(elapsed - self.dt) <= self.dt * 1e-5:
                twist = pose_error(tcp, T0) / elapsed
                result = dict(velocity=(current[1]-q0)/elapsed,
                    tcp_speed=float(np.linalg.norm(twist[:3])),
                    tcp_angular_speed=float(np.linalg.norm(twist[3:])),
                    object_native_speed=(coordinate-x0)/elapsed)
        self.previous, self.latest = current, result
        return result


@dataclass(frozen=True)
class Bounds:
    position: np.ndarray
    speed: np.ndarray
    acceleration: np.ndarray
    linear_speed: float
    angular_speed: float
    linear_acceleration: float
    angular_acceleration: float
    linear_guard_speed: float | None = None

    def __post_init__(self):
        if self.linear_guard_speed is None:
            object.__setattr__(self, 'linear_guard_speed', self.linear_speed)
        for k in ('position', 'speed', 'acceleration'):
            object.__setattr__(self, k, _freeze(getattr(self, k)))
        if (self.position.shape != (9, 2) or self.speed.shape != (9,)
                or self.acceleration.shape != (9,)
                or not np.isfinite(np.r_[self.position.ravel(), self.speed, self.acceleration,
                    self.linear_speed, self.angular_speed,
                    self.linear_acceleration, self.angular_acceleration, self.linear_guard_speed]).all()
                or np.any(self.position[:, 0] >= self.position[:, 1])
                or np.any(self.speed <= 0) or np.any(self.acceleration <= 0)
                or min(self.linear_speed, self.angular_speed,
                    self.linear_acceleration, self.angular_acceleration, self.linear_guard_speed) <= 0):
            raise ValueError('POSITIVE_FINITE_NINE_DOF_BOUNDS_REQUIRED')


@dataclass(frozen=True)
class Command:
    position: np.ndarray
    velocity: np.ndarray
    twist: np.ndarray
    generation: int
    tcp_design_ratios: tuple = (0., 0., 0., 0.)

    def __post_init__(self):
        for k in ('position', 'velocity', 'twist'):
            object.__setattr__(self, k, _freeze(getattr(self, k)))


@dataclass(frozen=True)
class StopPlan:
    commands: tuple
    reason: str = ''
    checked_configurations: int = 0

    @property
    def valid(self):
        return not self.reason


@dataclass(frozen=True)
class Proposal:
    command: Command | None
    stop: StopPlan | None
    reason: str
    parent_generation: int
    solve_s: float = 0.
    stop_s: float = 0.

    @property
    def accepted(self):
        return self.command is not None and self.stop is not None and self.stop.valid and not self.reason


class WeightedDLS:
    """One 6x6 solve. Gain is s^-1, damping is dimensionless after scaling.

    XY/arm allocation follows the actual Jacobian, available joint margin and
    a shared arm cost, rather than an object-specific displacement ratio.
    """
    def __init__(self, kinematics, gain_s=6., damping=.03, arm_cost=4.):
        if min(gain_s, damping, arm_cost) <= 0:
            raise ValueError('POSITIVE_DLS_PARAMETERS_REQUIRED')
        self.kinematics = kinematics
        self.gain_s, self.damping, self.arm_cost = gain_s, damping, arm_cost

    def velocity(self, goal, measured, bounds, *, feedforward=None, mobile=True, speed_limits=None):
        q = np.asarray(measured, float)
        if q.shape != (9,) or not np.isfinite(q).all():
            raise ValueError('INVALID_MEASURED_STATE')
        J = self.kinematics.jacobian(q)
        desired = self.gain_s * pose_error(goal, self.kinematics.fk(q))
        if feedforward is not None:
            desired += np.asarray(feedforward, float)
        # One task-speed saturation; feedback cannot request arbitrarily large
        # corrections when contact compliance produces a tracking deficit.
        linear, angular = ((bounds.linear_speed, bounds.angular_speed)
                           if speed_limits is None else speed_limits)
        if not np.isfinite([linear, angular]).all() or min(linear, angular) < 0:
            raise ValueError('FINITE_NONNEGATIVE_REFERENCE_SPEED_REQUIRED')
        desired *= min(1., linear / max(np.linalg.norm(desired[:3]), 1e-30),
                       angular / max(np.linalg.norm(desired[3:]), 1e-30))
        half = (bounds.position[:, 1] - bounds.position[:, 0]) / 2
        margin = np.minimum(q - bounds.position[:, 0], bounds.position[:, 1] - q) / half
        mobility = bounds.speed.copy()
        mobility[2:] *= np.clip(margin[2:], .1, 1.) / self.arm_cost
        if not mobile:
            mobility[:2] = 0.
        scales = np.r_[np.full(3, bounds.linear_speed), np.full(3, bounds.angular_speed)]
        A = J * mobility / scales[:, None]
        # Adapted weighted form of J.T solve(J J.T + lambda^2 I, error).
        v = mobility * (A.T @ np.linalg.solve(A @ A.T + self.damping**2 * np.eye(6), desired / scales))
        if not np.isfinite(v).all():
            raise ValueError('NONFINITE_DLS_RESULT')
        return v


class CommandOwner:
    """Pure proposals; explicit commit is the only state mutation for motion.

    A finite analytic common-scalar brake preserves XY/arm cancellation.
    Preview evaluates this exact discrete trajectory; it never calls IK or a
    command optimizer. A failed new proposal leaves the last certified brake
    available. Numerical/candidate rejection is not a measured hard event.
    """
    def __init__(self, kinematics, position, dt, *, velocity=None, previous_twist=None):
        if not math.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError('EXPLICIT_CONTROL_DT_SECONDS_REQUIRED')
        self.kinematics, self.dt = kinematics, float(dt)
        q = np.asarray(position, float)
        v = np.zeros(9) if velocity is None else np.asarray(velocity, float)
        twist = np.zeros(6) if previous_twist is None else np.asarray(previous_twist, float)
        if q.shape != (9,) or v.shape != (9,) or twist.shape != (6,) or not np.isfinite(np.r_[q, v, twist]).all():
            raise ValueError('FINITE_INITIAL_COMMAND_REQUIRED')
        self.command = Command(q, v, twist, 0)
        self.stop = StopPlan(()) if not np.any(v) else None
        self.state = StopState.STOPPED if not np.any(v) else StopState.RUNNING
        self._settled_s = 0.
        self.command_zero = not np.any(v)
        self.measured_stationary = False

    def _command(self, q, v, previous, bounds, *, pose=None, previous_pose=None):
        if not np.isfinite(np.r_[q, v]).all():
            return None, 'NONFINITE_CANDIDATE'
        if np.any(q < bounds.position[:, 0]) or np.any(q > bounds.position[:, 1]):
            return None, 'CANDIDATE_JOINT_BOUND'
        if np.any(abs(v) > bounds.speed + 1e-10):
            return None, 'CANDIDATE_JOINT_SPEED'
        if np.any(abs(v - previous.velocity) > bounds.acceleration * self.dt + 1e-10):
            return None, 'CANDIDATE_JOINT_ACCELERATION'
        twist = pose_error(self.kinematics.fk(q) if pose is None else pose,
            self.kinematics.fk(previous.position) if previous_pose is None else previous_pose) / self.dt
        acc = (twist - previous.twist) / self.dt
        norms = (np.linalg.norm(twist[:3]), np.linalg.norm(twist[3:]),
                 np.linalg.norm(acc[:3]), np.linalg.norm(acc[3:]))
        limits = (bounds.linear_speed, bounds.angular_speed,
                  bounds.linear_acceleration, bounds.angular_acceleration)
        # TCP reference speed/acceleration are DESIGN bounds. Their exact-FK
        # deviations are telemetry, not a second nonlinear motion veto. The
        # original independently supervised dangerous-speed boundary remains
        # an explicit candidate guard. Actuator rate/position and geometry
        # checks above are unchanged. No measured hard event is suppressed.
        if norms[0] > bounds.linear_guard_speed * (1 + 1e-7) + 1e-9:
            return None, 'CANDIDATE_TCP_GUARD_SPEED:%.9g>%.9g' % (norms[0], bounds.linear_guard_speed)
        return Command(q, v, twist, previous.generation + 1,
            tuple(float(x/y) for x,y in zip(norms,limits))), ''

    def brake(self, command, bounds, check=None):
        v0 = command.velocity
        if not np.any(v0):
            return StopPlan(())
        # Half the acceleration budget is reserved for FK curvature. Joint and
        # world-TCP limits stay unchanged; no loaded->release limit jump.
        duration = max(float(np.max(abs(v0) / bounds.acceleration)),
            np.linalg.norm(command.twist[:3]) / bounds.linear_acceleration,
            np.linalg.norm(command.twist[3:]) / bounds.angular_acceleration) * 2
        steps = max(2, math.ceil(duration / self.dt))
        if steps > math.ceil(5 / self.dt):
            return StopPlan((), 'BRAKE_EXCEEDS_FINITE_HORIZON')
        samples, current, checked = [], command, 0
        current_pose = self.kinematics.fk(command.position)
        last_checked = command.position
        resolution = np.r_[.0005, .0005, np.full(7, .005)]
        for i in range(1, steps + 1):
            v = v0 * ((steps - i) / steps)
            q = current.position + self.dt * v
            next_pose = self.kinematics.fk(q)
            next_command, reason = self._command(q, v, current, bounds,
                pose=next_pose, previous_pose=current_pose)
            if reason:
                return StopPlan((), 'BRAKE_' + reason, checked)
            # Spatial sampling includes ALL9 coordinates, so cancellation cannot
            # hide carriage/arm travel behind a stationary TCP. Slow motion does
            # not repeat the same costly geometry query every few microseconds.
            if check is not None and (i == 1 or i == steps or
                    np.max(abs(next_command.position-last_checked)/resolution) >= 1.):
                checked += 1
                valid, reason = check(next_command.position)
                if not valid:
                    return StopPlan((), 'BRAKE_GEOMETRY:' + str(reason), checked)
                last_checked = next_command.position
            samples.append(next_command)
            current = next_command
            current_pose = next_pose
        # Zero dq needs one more sample to confirm the command acceleration at
        # the terminal hold, independently of measured stationarity.
        terminal, reason = self._command(current.position, np.zeros(9), current, bounds,
            pose=current_pose, previous_pose=current_pose)
        if reason:
            return StopPlan((), 'BRAKE_TERMINAL_' + reason, checked)
        return StopPlan(tuple(samples + [terminal]), checked_configurations=checked)

    def propose(self, wanted, bounds, check=None):
        previous = self.command
        try:
            v = np.asarray(wanted, float).copy()
            if v.shape != (9,) or not np.isfinite(v).all():
                raise ValueError('INVALID_WANTED_VELOCITY')
            margin = np.where(v >= 0, bounds.position[:, 1] - previous.position,
                              previous.position - bounds.position[:, 0])
            # Reserve the common-scalar brake, including one discrete interval.
            speed = np.minimum(bounds.speed, np.maximum(0.,
                np.sqrt((bounds.acceleration * self.dt)**2 + bounds.acceleration * np.maximum(margin, 0))
                - bounds.acceleration * self.dt))
            v *= _fraction(v, speed)
            J = self.kinematics.jacobian(previous.position)
            twist = J @ v
            v *= min(1., bounds.linear_speed / max(np.linalg.norm(twist[:3]), 1e-30),
                       bounds.angular_speed / max(np.linalg.norm(twist[3:]), 1e-30))
            delta = v - previous.velocity
            task_change = J @ delta
            # One continuous task/joint rate calculation, with half the task
            # acceleration allocated to feedback. Exact FK design deviations
            # do not freeze velocity, trigger a projection or end the action.
            fraction = min(_fraction(delta, bounds.acceleration * self.dt),
                .5 * bounds.linear_acceleration * self.dt / max(np.linalg.norm(task_change[:3]), 1e-30),
                .5 * bounds.angular_acceleration * self.dt / max(np.linalg.norm(task_change[3:]), 1e-30))
            v = previous.velocity + min(1., fraction) * delta
            return self._certify(previous.position + v * self.dt, v, bounds, check)
        except (ValueError, np.linalg.LinAlgError) as error:
            return Proposal(None, None, 'NUMERICAL_CANDIDATE:' + str(error), previous.generation)

    def _certify(self, position, velocity, bounds, check):
        previous=self.command
        if self.state in (StopState.BRAKING,StopState.SETTLING,StopState.BLOCKED):
            reason=('MEASURED_STOP_PENDING' if self.state==StopState.SETTLING
                else 'MOTION_FORBIDDEN_IN_STOP_STATE:'+self.state.value)
            return Proposal(None,None,reason,previous.generation)
        command, reason=self._command(position,velocity,previous,bounds)
        if reason:return Proposal(None,None,reason,previous.generation)
        if check is not None:
            valid,reason=check(command.position)
            if not valid:return Proposal(None,None,'CANDIDATE_GEOMETRY:'+str(reason),previous.generation)
        began=time.perf_counter();stop=self.brake(command,bounds,check)
        return Proposal(command if stop.valid else None,stop,stop.reason,
            previous.generation,stop_s=time.perf_counter()-began)

    def propose_position(self, position, bounds, check=None):
        """Validate an already timed native joint-path sample, without retiming.

        A second rate filter would lag and drop the route's final endpoint.
        The same owner still validates exact discrete rates, geometry and its
        brake before committing; a rejected position cannot mutate state.
        """
        try:
            q=np.asarray(position,float)
            if q.shape!=(9,) or not np.isfinite(q).all():raise ValueError('FINITE_NINE_DOF_POSITION_REQUIRED')
            return self._certify(q,(q-self.command.position)/self.dt,bounds,check)
        except (ValueError,np.linalg.LinAlgError) as error:
            return Proposal(None,None,'NUMERICAL_CANDIDATE:'+str(error),self.command.generation)

    def track(self, kernel, goal, measured, bounds, check=None, **kwargs):
        began = time.perf_counter()
        try:
            velocity = kernel.velocity(goal, measured, bounds, **kwargs)
        except (ValueError, np.linalg.LinAlgError) as error:
            return Proposal(None, None, 'NUMERICAL_IK:' + str(error), self.command.generation,
                            solve_s=time.perf_counter() - began)
        solve_s = time.perf_counter() - began
        p = self.propose(velocity, bounds, check)
        return Proposal(p.command, p.stop, p.reason, p.parent_generation, solve_s, p.stop_s)

    def commit(self, proposal):
        if not proposal.accepted or proposal.parent_generation != self.command.generation:
            raise ValueError('REJECTED_OR_STALE_PROPOSAL_CANNOT_COMMIT')
        if self.state in (StopState.BRAKING, StopState.SETTLING, StopState.BLOCKED):
            raise ValueError('MOTION_FORBIDDEN_IN_STOP_STATE')
        self.command, self.stop = proposal.command, proposal.stop
        self.command_zero = not np.any(self.command.velocity)
        self.measured_stationary = False
        self._settled_s = 0.
        self.state = StopState.RUNNING

    def start_stop(self):
        if self.state == StopState.BLOCKED:
            raise ValueError('STOP_BLOCKED_REQUIRES_EXTERNAL_DIAGNOSIS')
        if self.stop is None or not self.stop.valid:
            self.state = StopState.BLOCKED
            raise ValueError('NO_CERTIFIED_BRAKE')
        self.state = StopState.BRAKING
        return self.stop.commands

    def commit_brake(self, command):
        if self.state != StopState.BRAKING or command.generation != self.command.generation + 1:
            raise ValueError('OUT_OF_ORDER_BRAKE')
        self.command = command
        self.command_zero = not np.any(command.velocity)
        if self.command_zero:
            self._settled_s = 0.
        # Keep BRAKING through its terminal acceleration-confirmation sample.

    def settling(self):
        if not self.command_zero:
            raise ValueError('NONZERO_COMMAND_CANNOT_SETTLE')
        self.stop = StopPlan(())
        self.state = StopState.SETTLING

    def observe_stop(self, base_velocity, *, body_stationary, dt=None):
        """Reuse the native SI body-stop witness; independently verify XY.

        PhysicalMotionDwell owns arm/TCP/articulated-object confirmation,
        including revolute surface speed and angular motion. Do not duplicate
        those thresholds here or confuse object rad/s with m/s.
        """
        base_velocity = np.asarray(base_velocity, float)
        if base_velocity.shape != (2,) or not np.isfinite(base_velocity).all():
            self.state = StopState.BLOCKED
            return False
        quiet = np.max(abs(base_velocity)) <= 1e-4
        self._settled_s = self._settled_s + (self.dt if dt is None else dt) if quiet and self.command_zero else 0.
        self.measured_stationary = bool(body_stationary and self.command_zero
            and self._settled_s >= STOP_DWELL_S - 1e-10)
        if self.measured_stationary:
            self.state = StopState.STOPPED
        elif self.command_zero and self.state == StopState.STOPPED:
            self.state = StopState.SETTLING
        return self.measured_stationary
