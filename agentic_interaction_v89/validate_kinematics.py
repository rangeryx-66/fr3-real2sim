"""CPU check using the real robot URDF, not a simulated contact experiment."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from .controller import Bounds, CommandOwner, WeightedDLS, pose_error
from .kinematics import WorldKinematics


def validate(config):
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / 'scripts'))
    from cross_robot_adapter import RobotPinkIK
    c = json.loads(Path(config).read_text())
    solver = RobotPinkIK(c)
    limits = np.vstack([np.tile([-1., 1.], (2, 1)), solver.original_limits + [.05, -.05]])
    speeds = np.r_[.08, .08, np.asarray(c['arm_speed_limits']) / 3]
    # Deliberately rotated nonzero station tests world/local conversion.
    k = WorldKinematics(solver, [.3, -.5, .2, 35.], [.1, -.1], limits, speeds)
    q = np.r_[.1, -.1, c['home_q']]
    J = k.jacobian(q)
    eps = 1e-6
    finite = np.column_stack([pose_error(k.fk(q + np.eye(9)[i]*eps), k.fk(q - np.eye(9)[i]*eps))/(2*eps) for i in range(9)])
    error = float(np.max(abs(J - finite)))
    if error > 1e-6:
        raise AssertionError(('WORLD_TCP_JACOBIAN', error))
    b = Bounds(limits, speeds, np.r_[.04, .04, np.full(7, .8)], .012, .2, .04, .8)
    o = CommandOwner(k, q, 1/240)
    kernel = WeightedDLS(k)
    initial = k.fk(q)
    times, stops, failures = [], [], []
    max_error = 0.
    for i in range(240):
        target = initial.copy()
        target[0, 3] += .003 * min(i / 120, 1.)
        began = time.perf_counter()
        p = o.track(kernel, target, o.command.position, b)
        times.append(time.perf_counter() - began)
        if not p.accepted:
            failures.append(p.reason)
            break
        o.commit(p)
        stops.append(p.stop_s)
        max_error = max(max_error, float(np.linalg.norm(pose_error(target, k.fk(o.command.position))[:3])))
    o.stop = o.brake(o.command, b)
    if o.stop.valid:
        for sample in o.start_stop(): o.commit_brake(sample)
        o.settling()
        for _ in range(74): o.observe_stop(np.zeros(2), body_stationary=True)
    # Exercise the actual native C2 timed-reference generator against the new
    # position transaction. No second rate filter or lost terminal sample.
    from interaction_runtime.motion import timed_path
    from dataclasses import replace
    timed_bounds=replace(b,linear_guard_speed=.03)
    joint_target=q[2:].copy();joint_target[-1]+=.02
    joint_owner=CommandOwner(k,q,1/240);timed_steps=0
    motion=dict(speed_m_s=b.linear_speed,angular_speed_rad_s=b.angular_speed,
        acceleration_m_s2=b.linear_acceleration,angular_acceleration_rad_s2=b.angular_acceleration)
    for sample,_ in timed_path(solver,k.base(q[:2]),[q[2:],joint_target],1/240,
            motion,speeds[2:],.8,lambda values:(True,'')):
        p=joint_owner.propose_position(np.r_[q[:2],sample],timed_bounds)
        if not p.accepted:raise AssertionError(('NATIVE_TIMED_REFERENCE',p.reason))
        joint_owner.commit(p);timed_steps+=1
    timed_error=float(np.max(abs(joint_owner.command.position[2:]-joint_target)))
    if timed_error>1e-10:raise AssertionError(('TIMED_ENDPOINT_DROPPED',timed_error))
    return dict(physical_simulation=False, collision_or_contact_validation=False,
        world_jacobian_max_absolute_error=error, motion_steps=len(times), failures=failures,
        command_stop_valid=o.stop.valid, measured_stationarity='SYNTHETIC_ZERO_VELOCITY_ONLY',
        commanded_XY_displacement_m=(o.command.position[:2] - q[:2]).tolist(),
        commanded_arm_displacement_rad=(o.command.position[2:] - q[2:]).tolist(),
        max_reference_error_m=max_error,
        native_timed_reference_steps=timed_steps,native_timed_endpoint_error_rad=timed_error,
        solve_and_command_median_s=float(np.median(times)),
        solve_and_command_p95_s=float(np.quantile(times, .95)),
        stop_check_median_s=float(np.median(stops)) if stops else None)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); result = validate(args.config)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
