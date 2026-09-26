"""Robot-specific names and geometry used by the shared grasp executor."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import os
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

@dataclass(frozen=True)
class RobotProfile:
    name: str
    base_link: str
    tcp_link: str
    move_group: str
    arm_joints: tuple[str, ...]
    touch_links: tuple[str, str]
    gripper_action: str
    arm_action: str
    urdf: Path
    srdf: Path
    usd_prim: str
    home: tuple[float, ...]
    open_width_m: float
    command_gripper_joint: str
    physical_finger_joints: tuple[str, ...]
    grasp_tip_offset_m: float
    robot_link_prefixes: tuple[str, ...]

FR3 = RobotProfile(
    name='fr3', base_link='fr3_link0', tcp_link='fr3_hand_tcp', move_group='fr3_arm',
    arm_joints=tuple(f'fr3_joint{i}' for i in range(1,8)),
    touch_links=('fr3_leftfinger','fr3_rightfinger'),
    gripper_action='/franka_gripper/gripper_action',
    arm_action='/fr3_arm_controller/follow_joint_trajectory',
    urdf=ROOT/'config/fr3.urdf', srdf=ROOT/'config/fr3.srdf', usd_prim='/World/FR3',
    home=(0.,-.5,0.,-2.,0.,1.5,.785398), open_width_m=.08,
    command_gripper_joint='fr3_finger_joint1',
    physical_finger_joints=('fr3_finger_joint1','fr3_finger_joint2'),
    grasp_tip_offset_m=.0095,
    robot_link_prefixes=('fr3_',),
)

PIPER = RobotProfile(
    name='piper', base_link='base_link', tcp_link='tcp_link', move_group='arm',
    arm_joints=tuple(f'joint{i}' for i in range(1,7)),
    touch_links=('gripper_link1','gripper_link2'),
    gripper_action='/piper_gripper/gripper_action',
    arm_action='/piper_arm_controller/follow_joint_trajectory',
    urdf=ROOT/'config/piper.urdf', srdf=ROOT/'config/piper.srdf', usd_prim='/World/Piper',
    # Collision-free ready pose from the official joint limits, facing the table.
    home=(0.,1.15,-1.35,0.,.20,0.), open_width_m=.10,
    command_gripper_joint='gripper',
    physical_finger_joints=('gripper_joint1','gripper_joint2'),
    # Generated TCP is at the official distal finger plane.  A 20 mm insertion
    # places the object inside the 76.5 mm Piper finger span instead of at its edge.
    grasp_tip_offset_m=-.020,
    robot_link_prefixes=('base_link','link','flange_link','gripper_','tcp_link'),
)

PROFILES = {'fr3': FR3, 'piper': PIPER}

def get_profile(name: str | None = None) -> RobotProfile:
    key=(name or os.environ.get('GRASP_ROBOT','fr3')).lower()
    if key not in PROFILES:
        raise ValueError(f'unknown GRASP_ROBOT={key!r}; expected {sorted(PROFILES)}')
    return PROFILES[key]

def gripper_positions(profile: RobotProfile, width_m: float) -> dict[str,float]:
    """URDF joint positions for a requested total jaw opening."""
    w=float(np.clip(width_m,0.,profile.open_width_m))
    if profile.name=='fr3': return {j:w/2 for j in profile.physical_finger_joints}
    return {'gripper':w,'gripper_joint1':w/2,'gripper_joint2':-w/2}
