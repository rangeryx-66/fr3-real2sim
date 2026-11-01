"""Closed-only command brake comparison; no simulator or dynamics GT input.

Float32 legacy arm commands are approximate; this is not a bit-exact replay of
the live K failure and does not test contact, release or measured stopping.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from .controller import Bounds, CommandOwner, pose_error
from .kinematics import WorldKinematics


def replay(episode):
    episode = Path(episode)
    if json.loads((episode / 'PROCESS_COMPLETION.json').read_text()).get('actor_exited') is not True:
        raise ValueError('CLOSED_EPISODE_REQUIRED')
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / 'scripts'))
    from diagnose_probing_v3 import columns
    from cross_robot_adapter import RobotPinkIK
    from agentic_interaction_v87.command_governor import CommandGovernor
    job = json.loads((episode / 'job.json').read_text())
    config = json.loads((Path(job['v8_robot_io']) / 'NATIVE_CONFIG.json').read_text())
    solver = RobotPinkIK(config)
    base = json.loads((Path(job['source']) / 'report.json').read_text())['robot_base_pose']
    policy = json.loads((root / 'configs/interaction_runtime_v1.json').read_text())
    limits = np.vstack([np.tile([-1., 1.], (2, 1)), solver.original_limits + [.05, -.05]])
    speeds = np.r_[.08, .08, np.asarray(config['arm_speed_limits']) / 3]
    k = WorldKinematics(solver, base, np.zeros(2), limits, speeds)
    a, index = columns(episode)
    dt = 1 / 240
    results = []
    for segment in index['segments']:
        indices = np.flatnonzero(a['segment'] == segment['id'])
        if len(indices) < 2:
            continue
        for offset in (.3, 0.):
            j = indices[np.argmin(abs(a['t'][indices] - (a['t'][indices[-1]] - offset)))]
            if j < 1 or a['step'][j] - a['step'][j-1] != 1:
                continue
            q = np.r_[a['v88_base_command_m'][j], a['command_q'][j]]
            previous = np.r_[a['v88_base_command_m'][j-1], a['command_q'][j-1]]
            v = np.r_[a['v88_base_command_velocity_m_s'][j], a['command_dq'][j]]
            twist = pose_error(k.fk(q), k.fk(previous)) / dt
            m = dict(policy['phases']['LOADED'])
            action = segment['declared_action']
            m['speed_m_s'] = min(m['speed_m_s'], action['reference_speed_m_s'])
            m['acceleration_m_s2'] = min(m['acceleration_m_s2'], action['reference_acceleration_m_s2'])
            acc = np.r_[np.full(2, m['acceleration_m_s2']),
                        np.full(7, policy['arm_command_acceleration_rad_s2']['LOADED'])]
            bounds = Bounds(limits, speeds, acc, m['speed_m_s'], m['angular_speed_rad_s'],
                            m['acceleration_m_s2'], m['angular_acceleration_rad_s2'])
            owner = CommandOwner(k, q, dt, velocity=v, previous_twist=twist)
            began = time.perf_counter()
            plan = owner.brake(owner.command, bounds)
            new_s = time.perf_counter() - began
            governor = CommandGovernor(k.fk, dt, preserve_direction=True)
            governor.previous_twist = twist.copy()
            x, velocity = q.copy(), v.copy()
            began = time.perf_counter(); reason = ''; steps = 0
            try:
                for _ in range(round(5 / dt)):
                    if np.max(abs(velocity)) <= 1e-12: break
                    velocity = governor.velocity(x, np.zeros(9), velocity, speeds, acc, m)
                    x += velocity * dt; steps += 1
                else: reason = 'STOP_HORIZON_EXHAUSTED'
            except RuntimeError as error:
                reason = str(error)
            results.append(dict(segment=segment['id'], role=segment['role'], t_s=float(a['t'][j]),
                old=dict(reason=reason, command_stopped=not reason, steps=steps,
                         wall_s=time.perf_counter() - began),
                new=dict(reason=plan.reason, command_stopped=plan.valid, steps=len(plan.commands), wall_s=new_s)))
    return dict(episode=episode.name, cases=results,
        reconstruction='float32 arm, float64 XY; previous twist from adjacent recorded commands',
        physical_motion_verified=False, collisions_checked=False, release_verified=False,
        measured_stationarity_verified=False, hidden_dynamics_read=False)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('episode', type=Path); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = replay(args.episode)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
