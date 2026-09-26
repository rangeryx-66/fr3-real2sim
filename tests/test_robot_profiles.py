import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from robot_profile import FR3,PIPER,gripper_positions
from frames import grasp_to_robot_tcp

def test_fr3_profile_preserves_frozen_names_and_geometry():
    assert FR3.arm_joints==tuple(f'fr3_joint{i}' for i in range(1,8))
    assert FR3.grasp_tip_offset_m==.0095
    assert gripper_positions(FR3,.08)=={'fr3_finger_joint1':.04,'fr3_finger_joint2':.04}

def test_piper_uses_official_joint_and_opening_conventions():
    assert PIPER.arm_joints==tuple(f'joint{i}' for i in range(1,7))
    assert PIPER.open_width_m==.10
    assert gripper_positions(PIPER,.08)=={'gripper':.08,'gripper_joint1':.04,'gripper_joint2':-.04}

def test_anygrasp_to_tcp_orientation_and_robot_depth_offsets():
    T=np.eye(4);R=np.eye(3);p=np.array([.1,.2,.3]);depth=.04
    _,fr3,_=grasp_to_robot_tcp(T,R,p,depth,FR3.grasp_tip_offset_m)
    _,piper,_=grasp_to_robot_tcp(T,R,p,depth,PIPER.grasp_tip_offset_m)
    assert np.allclose(fr3[:3,:3],piper[:3,:3])
    assert np.allclose(piper[:3,3]-fr3[:3,3],[FR3.grasp_tip_offset_m-PIPER.grasp_tip_offset_m,0,0])
