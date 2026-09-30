"""Plot recorded observations; never supplies a trajectory to the controller."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    observations = json.loads((args.run / 'observations.json').read_text())
    report = json.loads((args.run / 'report.json').read_text())
    samples = observations['samples']
    t = np.array([s['t'] for s in samples])
    ideal = np.array([s['T_ee_ideal'] for s in samples])
    actual = np.array([s['T_ee_robot'] for s in samples])
    force = np.array([s['applied_wrench_tcp_world'][:3] for s in samples])
    fit = report['robot_ee_estimate']
    fig, axs = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    delta = 1000 * (ideal[:, :3, 3] - ideal[0, :3, 3])
    measured = 1000 * (actual[:, :3, 3] - actual[0, :3, 3])
    axs[0, 0].plot(delta[:, 0], delta[:, 1], label='Ideal attached TCP')
    axs[0, 0].plot(measured[:, 0], measured[:, 1], '--', label='Actual R1 EE')
    axs[0, 0].scatter(*measured[0, :2], label='Start', color='black', s=20)
    axs[0, 0].set(xlabel='World X displacement (mm)', ylabel='World Y displacement (mm)', title='Observed XY motion')
    axs[0, 0].axis('equal'); axs[0, 0].legend(fontsize=8)
    for index, label in enumerate(('X', 'Y', 'Z')):
        axs[0, 1].plot(t, measured[:, index], label=label)
        axs[0, 2].plot(t, force[:, index], label=label)
    axs[0, 1].set(title='Actual EE displacement', ylabel='mm')
    axs[0, 2].set(title='Applied world force (no measured wrench)', ylabel='N')
    axs[0, 1].legend(); axs[0, 2].legend()
    if fit['joint_type'] == 'revolute':
        axis = np.array(fit['revolute']['axis'])
        angles = Rotation.from_matrix(actual[:, :3, :3] @ actual[0, :3, :3].T).as_rotvec() @ axis
        axs[1, 0].plot(t, np.rad2deg(angles))
    axs[1, 0].set(title='Angle from EE trajectory fit', ylabel='Degrees')
    axs[1, 1].plot(t, [1000 * s['tracking_error_m'] for s in samples])
    axs[1, 1].set(title='Actual EE to ideal TCP tracking error', ylabel='mm')
    axs[1, 2].plot(t, [s['joint_margin_rad'] for s in samples])
    axs[1, 2].axhline(.05, color='red', linestyle='--', label='0.05 rad threshold')
    axs[1, 2].set(title='Minimum measured arm joint margin', ylabel='rad')
    axs[1, 2].legend()
    for ax in axs.flat:
        ax.grid(alpha=.25)
    for ax in axs.flat[1:]:
        ax.set_xlabel('Simulation time (s)')
    fig.suptitle('Unknown-joint force probe: ideal attachment, NOT a real Dex1 grasp', fontsize=14)
    fig.savefig(args.run / 'trajectory.png', dpi=140)
    plt.close(fig)


if __name__ == '__main__':
    main()
