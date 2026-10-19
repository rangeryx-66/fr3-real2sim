"""MoveIt 2 for the independently generated official R1-7a + Dex1 model."""
from pathlib import Path
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


def generate_launch_description():
    subprocess.run([sys.executable, str(ROOT / 'scripts/prepare_r1a7_description.py')], check=True)
    variant = os.environ.get('R1A7_MODEL_VARIANT', 'full')
    if variant not in ('full', 'j7_fixed'):
        raise ValueError(f'unknown R1A7_MODEL_VARIANT={variant}')
    if variant == 'j7_fixed':
        subprocess.run([sys.executable, str(ROOT / 'scripts/make_r1a7_j7_fixed.py')], check=True)
    model_path = ROOT / ('config/r1a7_dex1_j7_fixed.urdf' if variant == 'j7_fixed' else 'config/r1a7_dex1.urdf')
    model = ET.parse(model_path).getroot()
    sys.path.insert(0, str(ROOT/'src'))
    from dex1_collision_profile import apply_profile
    apply_profile(model)
    for mesh in model.findall('.//mesh'):
        path = Path(mesh.get('filename'))
        if not path.is_file():
            raise RuntimeError(f'Missing official R1-7a/Dex1 mesh: {path}')
        mesh.set('filename', path.as_uri())
    urdf = ET.tostring(model, encoding='unicode')
    srdf = (ROOT / 'config/r1a7_dex1.srdf').read_text()
    arm = [j.get('name') for j in model.findall('joint') if j.get('name') in {f'J{i}' for i in range(1, 8)} and j.get('type') == 'revolute']
    limits = {}
    for joint in model.findall('joint'):
        limit = joint.find('limit')
        if limit is None:
            continue
        limits[joint.get('name')] = {
            'has_velocity_limits': True,
            'max_velocity': float(limit.get('velocity')),
            'has_acceleration_limits': True,
            'max_acceleration': 1.0,
        }
    config = {
        'robot_description': urdf,
        'robot_description_semantic': srdf,
        'use_sim_time': True,
        'robot_description_kinematics': {'r1a7_arm': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_timeout': 0.5,
            'kinematics_solver_search_resolution': 0.01,
        }},
        'robot_description_planning': {'joint_limits': limits},
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.01,
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0}},
            'r1a7_arm': {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': 0.005},
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': ['r1a7_arm_controller'],
            'r1a7_arm_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': arm,
            },
        },
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 3.0,
            'allowed_goal_duration_margin': 3.0,
            'allowed_start_tolerance': 0.025,
        },
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=[config]),
    ])
