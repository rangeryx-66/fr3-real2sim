"""One bounded nine-variable solve, followed by nonlinear command governance.

XY and arm targets are calculated together, before either is submitted. The
objective prefers arm posture retention; it specifies no displacement split.
Only measured/current kinematics and native limits enter this calculation.
"""
import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation


class CoupledKinematics:
    def __init__(self, solver, base, base_joints, limits, speeds):
        self.solver = solver
        self.base_origin = np.asarray(base, float).copy()
        self.base_origin[:2] -= np.asarray(base_joints, float)
        self.limits = np.asarray(limits, float)
        self.speeds = np.asarray(speeds, float)
        self.data = solver.robot.createData()
        self.frame = solver.robot.getFrameId('tcp_link')
        self.latest = {}

    def base(self, xy):
        pose = self.base_origin.copy()
        pose[:2] += xy
        return pose

    def fk(self, q):
        from interaction_reference_handover import seed_pose
        return seed_pose(self.solver, self.base(np.asarray(q)[:2]), np.asarray(q)[2:])

    def jacobian(self, q):
        import pinocchio as pin
        from articulated_demo.kinematics import transform
        q = np.asarray(q, float)
        local = pin.computeFrameJacobian(self.solver.robot, self.data,
            q[2:][self.solver.indices], self.frame, pin.LOCAL_WORLD_ALIGNED)
        B = transform(self.base_origin[:3], [0, 0, np.deg2rad(self.base_origin[3])])
        J = np.zeros((6, 9)); J[:2, :2] = np.eye(2)
        J[:3, 2:] = B[:3, :3] @ local[:3]
        J[3:, 2:] = B[:3, :3] @ local[3:]
        return J

    def velocity(self, target, q, previous, dt, acceleration, motion, posture,
                 *, command=None, bias=None, integrated=False):
        q = np.asarray(q, float); previous = np.asarray(previous, float)
        command_feedback = command is not None
        command = q if command is None else np.asarray(command, float)
        bias = np.zeros_like(q) if bias is None else np.asarray(bias, float)
        current = self.fk(q)
        error = np.r_[target[:3, 3] - current[:3, 3],
            Rotation.from_matrix(target[:3, :3] @ current[:3, :3].T).as_rotvec()]
        desired = .75 * error / dt  # preserved native FrameTask feedback gain
        task_scales = np.r_[np.full(3, motion['speed_m_s']),
            np.full(3, motion['angular_speed_rad_s'])]
        if not command_feedback:
            # Offline pose continuation may start far from a target; take
            # bounded geometric steps rather than recruit the arm to rush it.
            for sl in (slice(0, 3), slice(3, 6)):
                norm = np.linalg.norm(desired[sl] / task_scales[sl])
                if norm > 1: desired[sl] /= norm
        J = self.jacobian(q)
        # Native measured-state IK returns a NEW seed position, not a command
        # velocity. Translate that proposal into all nine COMMAND coordinates
        # before applying rate bounds. Clipping the seed's corrective twist
        # first lets compliance drift accumulate and breaks the XY/arm pair.
        if not integrated: desired += J @ ((q + bias - command) / dt)
        acceleration = np.broadcast_to(acceleration, q.shape)
        low = np.maximum.reduce([-self.speeds, previous - acceleration * dt,
            (self.limits[:, 0] - command) / dt])
        high = np.minimum.reduce([self.speeds, previous + acceleration * dt,
            (self.limits[:, 1] - command) / dt])
        if not np.isfinite(np.r_[q, previous, low, high, desired]).all() or np.any(low >= high):
            raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:COUPLED_RATE_INFEASIBLE')
        # Penalize task-space effect of arm motion, strengthened near limits.
        # This is a shared posture objective, not an asset-specific travel cap.
        half = (self.limits[2:, 1] - self.limits[2:, 0]) / 2
        fraction = np.minimum(command[2:] - self.limits[2:, 0],
            self.limits[2:, 1] - command[2:]) / half
        effect = np.linalg.norm(J[:, 2:] / task_scales[:, None], axis=0)
        cost = np.r_[1 / task_scales[:2],
            np.sqrt(16 * len(q[2:])) * effect / np.maximum(fraction, 1e-6)]
        regularizer = 1e-3 * np.diag(cost * self.speeds)
        restoring = np.zeros(9)
        restoring[2:] = np.clip(.75 * (np.asarray(posture) - command[2:]),
            -self.speeds[2:], self.speeds[2:])
        A = np.vstack([J * self.speeds / task_scales[:, None], regularizer])
        b = np.r_[desired / task_scales, regularizer @ (restoring / self.speeds)]
        result = lsq_linear(A, b, bounds=(low / self.speeds, high / self.speeds),
            tol=1e-10, max_iter=80)
        v = result.x * self.speeds
        if not result.success or not np.isfinite(v).all() or np.any(v < low - 1e-12) or np.any(v > high + 1e-12):
            raise RuntimeError('CONTROLLED_EXECUTION_INTERRUPT:COUPLED_IK_UNAVAILABLE')
        self.latest = dict(base_command_speed_m_s=float(np.linalg.norm(v[:2])),
            arm_command_speed_rad_s=float(np.linalg.norm(v[2:])),
            task_residual=float(np.linalg.norm(J @ v - desired)),
            arm_posture_penalty='16 * arm_DOF / normalized_joint_margin_squared',
            base_and_arm_solved_together=True)
        return v

    def solve(self, target, q, posture=None):
        q = np.asarray(q, float).copy()
        posture = q[2:].copy() if posture is None else posture
        motion = dict(speed_m_s=float(min(self.speeds[:2])), angular_speed_rad_s=.2)
        for _ in range(80):
            pose = self.fk(q)
            if (np.linalg.norm(pose[:3, 3] - target[:3, 3]) < 1e-5 and
                    Rotation.from_matrix(pose[:3, :3] @ target[:3, :3].T).magnitude() < 1e-4):
                return q
            try:
                v = self.velocity(target, q, np.zeros(9), .04,
                    np.full(9, np.inf), motion, posture)
            except RuntimeError as error:
                if not str(error).startswith('CONTROLLED_EXECUTION_INTERRUPT:'): raise
                return None  # a finite candidate rejection, never a hard latch
            q += .04 * v
        return None
