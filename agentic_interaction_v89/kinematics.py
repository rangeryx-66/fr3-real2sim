"""Known robot geometry only; no object dynamics or archived grasp input."""
import numpy as np
from .controller import Bounds, WeightedDLS, pose_error


class WorldKinematics:
    def __init__(self, solver, base, base_joints, limits, speeds):
        self.solver = solver
        self.base_origin = np.array(base, float, copy=True)
        self.base_origin[:2] -= base_joints
        self.limits = np.asarray(limits, float)
        self.speeds = np.asarray(speeds, float)
        self.data = solver.robot.createData()
        self.frame = solver.robot.getFrameId('tcp_link')

    def base(self, xy):
        base = self.base_origin.copy()
        base[:2] += xy
        return base

    def fk(self, q):
        from interaction_reference_handover import seed_pose
        q = np.asarray(q)
        return seed_pose(self.solver, self.base(q[:2]), q[2:])

    def jacobian(self, q):
        import pinocchio as pin
        from articulated_demo.kinematics import transform
        # Pin model DOF order must be converted back to native arm order.
        local = pin.computeFrameJacobian(self.solver.robot, self.data,
            np.asarray(q)[2:][self.solver.indices], self.frame, pin.LOCAL_WORLD_ALIGNED)
        inverse = np.argsort(self.solver.indices)
        R = transform(self.base_origin[:3], [0, 0, np.deg2rad(self.base_origin[3])])[:3, :3]
        J = np.zeros((6, 9))
        J[:2, :2] = np.eye(2)
        J[:3, 2:] = R @ local[:3, inverse]
        J[3:, 2:] = R @ local[3:, inverse]
        return J

    def solve(self, target, q, posture=None):
        """Finite OFFLINE continuation for sampled path admission, not control."""
        q = np.asarray(q, float).copy()
        bounds = Bounds(self.limits, self.speeds, np.ones(9), .08, .2, 1., 1.)
        kernel = WeightedDLS(self)
        for _ in range(120):
            error = pose_error(target, self.fk(q))
            if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[3:]) < 1e-4:
                return q
            try:
                v = kernel.velocity(target, q, bounds)
            except (ValueError, np.linalg.LinAlgError):
                return None
            q += .04 * v
            if np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]):
                return None
        return None
