from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch_ros.actions import Node
ROOT=Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0,str(ROOT/'src'))
from robot_profile import get_profile
def generate_launch_description():
    profile=get_profile();urdf=profile.urdf.read_text();srdf=profile.srdf.read_text()
    # Isaac's importer accepts absolute filenames; ROS resource_retriever needs file://.
    description=ET.fromstring(urdf)
    for mesh in description.findall('.//mesh'):
        filename=mesh.attrib['filename']
        if filename.startswith('/'):
            path=Path(filename)
            if not path.is_file():raise RuntimeError('Missing robot mesh: '+filename)
            mesh.attrib['filename']=path.as_uri()
    urdf=ET.tostring(description,encoding='unicode')
    (ROOT/f'config/{profile.name}_moveit.urdf').write_text(urdf)
    limits={}
    for j in ET.fromstring(urdf).findall('joint'):
        if j.attrib['type'] not in ['revolute','prismatic']:continue
        lim=j.find('limit');limits[j.attrib['name']]={'has_velocity_limits':True,'max_velocity':float(lim.attrib['velocity']),'has_acceleration_limits':True,'max_acceleration':1.0}
    config={'robot_description':urdf,'robot_description_semantic':srdf,'use_sim_time':True,
        'robot_description_kinematics':{profile.move_group:{'kinematics_solver':'kdl_kinematics_plugin/KDLKinematicsPlugin','kinematics_solver_timeout':.1,'kinematics_solver_search_resolution':.005}},
        'robot_description_planning':{'joint_limits':limits},
        'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
        'ompl':{'planning_plugin':'ompl_interface/OMPLPlanner','request_adapters':'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints','start_state_max_bounds_error':.01,'planner_configs':{'RRTConnect':{'type':'geometric::RRTConnect','range':.0}},profile.move_group:{'planner_configs':['RRTConnect'],'longest_valid_segment_fraction':.005}},
        'moveit_controller_manager':'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager':{'controller_names':[profile.arm_action.strip('/').split('/')[0]],profile.arm_action.strip('/').split('/')[0]:{'type':'FollowJointTrajectory','action_ns':'follow_joint_trajectory','default':True,'joints':list(profile.arm_joints)}},
        'trajectory_execution':{'allowed_execution_duration_scaling':3.,'allowed_goal_duration_margin':3.,'allowed_start_tolerance':.025},
        'publish_robot_description':True,'publish_robot_description_semantic':True,
        'publish_planning_scene':True,'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True}
    return LaunchDescription([Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf,'use_sim_time':True}]),Node(package='moveit_ros_move_group',executable='move_group',output='screen',parameters=[config])])
