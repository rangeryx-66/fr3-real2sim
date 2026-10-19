"""Verify that URDF geometry produces a rigid TCP arc and obeys limits."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from articulated_demo.kinematics import URDFChain, transform


class ArticulatedKinematicsTest(unittest.TestCase):
    def test_revolute_arc_and_fixed_grasp(self):
        urdf = '''<robot name="fixture"><link name="base"/><link name="door"/>
        <joint name="hinge" type="revolute"><parent link="base"/><child link="door"/>
        <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
        <limit lower="0" upper="1.6" effort="1" velocity="1"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'fixture.urdf'; path.write_text(urdf)
            chain = URDFChain(path)
            moving = transform((1, 0, 0))
            tcp = transform((2, 0, 0))
            predicted = chain.target_tcp(moving_link='door', joint_name='hinge',
                q_now=0, q_target=np.pi/2, T_world_moving_now=moving,
                T_world_tcp_now=tcp)
            np.testing.assert_allclose(predicted[:3, 3], [1, 1, 0], atol=1e-6)
            np.testing.assert_allclose(np.linalg.inv(chain.root_to_link('door', {'hinge':np.pi/2}))
                                       @ predicted, np.linalg.inv(moving) @ tcp, atol=1e-6)
            with self.assertRaises(ValueError):
                chain.target_tcp(moving_link='door', joint_name='hinge', q_now=0,
                    q_target=2, T_world_moving_now=moving,T_world_tcp_now=tcp)


if __name__ == '__main__': unittest.main()
