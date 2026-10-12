"""Finite-authority PhysX implicit drive, approximate simulation actuator only.

The previous CAD-inertia gains and effort ceilings are retained. Gravity is
represented as a drive equilibrium offset, not a second external torque servo.
Commands are applied before each 240 Hz integration; PhysX solves the coupled
drive implicitly. No callback computes explicit derivative feedback.
"""
from pathlib import Path
import json
import numpy as np
import pinocchio as pin


class ImplicitNero:
    def __init__(self, robot, config, output):
        from isaacsim.core.utils.types import ArticulationAction
        self.Action = ArticulationAction
        self.robot = robot
        self.raw = robot.get_articulation_controller()
        self.names = list(robot.dof_names)
        self.arm = np.array([self.names.index(n) for n in config['arm_joint_names']])
        self.fingers = np.array([self.names.index(n) for n in ['gripper_joint1', 'gripper_joint2']])
        previous = Path(config['sim_urdf']).parent / 'integration_probe/APPROXIMATE_ACTUATOR.json'
        source = json.loads(previous.read_text())
        self.kp = np.array(source['kp_nm_rad'])
        self.kd = np.array(source['kd_nm_s_rad'])
        self.limit = np.array(source['maximum_arm_effort_nm'])
        self.pmodel = pin.buildModelFromUrdf(str(Path(config['sim_urdf']).with_name('nero_dynamics_coordinates.urdf')))
        self.pdata = self.pmodel.createData()
        self.pin_names = list(self.pmodel.names)[1:]
        self.pin_to_native = [self.names.index(n) for n in self.pin_names]
        self.pin_arm = [self.pin_names.index(n) for n in config['arm_joint_names']]
        kp = np.zeros(len(self.names)); kd = kp.copy()
        kp[self.arm] = self.kp; kd[self.arm] = self.kd
        kp[self.fingers] = 1000.; kd[self.fingers] = 40.
        self.raw.set_gains(kps=kp, kds=kd, save_to_usd=True)
        limits = np.zeros(len(self.names)); limits[self.arm] = self.limit; limits[self.fingers] = 10.
        self.robot._articulation_view.set_max_efforts(limits.reshape(1, -1))
        speeds = np.array([float(x['velocity']) for x in config['joint_limits']] + [.15, .15])
        self.robot._articulation_view.set_max_joint_velocities(speeds.reshape(1, -1))
        self.desired = np.asarray(robot.get_joint_positions())[self.arm].copy()
        self.desired_velocity = np.zeros(7)
        self.aperture = .08
        self.last = {}
        metadata = dict(source)
        metadata.update(classification='APPROXIMATE_SIMULATION_ACTUATOR_NOT_HARDWARE_CALIBRATED',
                        equation='PhysX implicit force drive: Kp*(q_des + g_URDF(q_des)/Kp - q) + Kd*(dq_des-dq), finite maxForce',
                        external_arm_effort_command_nm=[0.]*7,
                        gravity_compensation='equilibrium position bias inside the same finite-force drive; no external gravity torque',
                        gain_change=False, effort_limit_change=False,
                        command_timing='apply before world.step; no explicit torque physics callback',
                        arm_implicit_drive_gains={'kp':self.kp.tolist(), 'kd':self.kd.tolist()},
                        native_speed_limits=speeds.tolist())
        Path(output, 'APPROXIMATE_ACTUATOR.json').write_text(json.dumps(metadata, indent=2))

    def command(self):
        q = np.asarray(self.robot.get_joint_positions()).copy()
        q[self.arm] = self.desired
        gravity = pin.computeGeneralizedGravity(self.pmodel, self.pdata, q[self.pin_to_native])[self.pin_arm].copy()
        target = self.desired + gravity / self.kp
        self.raw.apply_action(self.Action(joint_positions=target, joint_velocities=self.desired_velocity,
                                         joint_efforts=np.zeros(7), joint_indices=self.arm))
        self.raw.apply_action(self.Action(joint_positions=np.array([self.aperture/2, -self.aperture/2]),
                                         joint_velocities=np.zeros(2), joint_efforts=np.zeros(2), joint_indices=self.fingers))
        self.last = {'q_reference':self.desired.tolist(), 'dq_reference':self.desired_velocity.tolist(),
                     'implicit_position_target':target.tolist(), 'gravity_bias_nm':gravity.tolist(),
                     'external_effort_command_nm':[0.]*7, 'aperture_command_m':self.aperture}
