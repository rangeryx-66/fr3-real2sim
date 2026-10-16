"""Versioned virtual six-axis cut at flange->G1; no motor/contact-force proxy.

The hardware sensor CAD/adapter is not present in the pinned official model.
This zero-thickness, infinite-stiffness interface does NOT model its 600 g
assembly, compliance, bandwidth or precision. Original tool geometry is retained.
"""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SENSOR_JOINT = 'wrist_ft_interface'
SENSOR_BODY = 'xarm_gripper_base_link'


def prepare(output):
    from xarm7_native_gripper import configuration
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    c = configuration()
    sources = {}
    for key in ['sim_urdf', 'dynamics_urdf']:
        source = Path(c[key]); sources[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
        tree = ET.parse(source)
        cut = tree.getroot().find("joint[@name='gripper_fix']")
        if cut is None or cut.find('parent').get('link') != 'link_eef' or cut.find('child').get('link') != SENSOR_BODY:
            raise ValueError('Unexpected physical mounting interface before initialization')
        cut.set('name', SENSOR_JOINT)
        target = output / (key + '.urdf'); tree.write(target, encoding='utf-8', xml_declaration=True)
        c[key] = str(target)
    c['urdf'] = c['sim_urdf']
    c['wrist_ft'] = dict(version='virtual_cut_v0', joint=SENSOR_JOINT, child_body=SENSOR_BODY,
        parent_body='link_eef', order=['Fx','Fy','Fz','Tx','Ty','Tz'], units=['N']*3+['N m']*3,
        frame='child joint frame, coincident G1 base origin; URDF gripper_fix origin unchanged',
        hardware_sensor='UFACTORY AI1500 reference only; not mechanical/dynamic replica',
        hardware_spec_url='https://docs.accessories.ufactory.cc/6_Axis_Force_Torque_Sensor/5.Technical_Specifications.html',
        simulation_rate_hz=240, noise='none modeled; not a precision claim', filtering='none',
        hardware_rate_hz=200, idealized_zero_thickness=True, hardware_mass_not_modeled=True,
        baseline_tool_and_camera_transforms_unchanged=True, source_urdf_hashes=sources)
    (output/'SENSOR_CONFIG.json').write_text(json.dumps(c, indent=2))
    return c


def wrench_at(wrench, T_world_sensor, origin_world):
    """Rotate [force,moment] and shift moment to a new world origin."""
    w = np.asarray(wrench); T = np.asarray(T_world_sensor)
    force = T[:3,:3] @ w[:3]
    return np.r_[force, T[:3,:3] @ w[3:] + np.cross(T[:3,3]-origin_world, force)]


def generalized_effort(robot_on_object_sensor, T_world_sensor, prior):
    """Explicit kinematic prior; no object dynamics/GT torque accepted."""
    axis = np.asarray(prior['axis_world'], dtype=float); axis /= np.linalg.norm(axis)
    w = wrench_at(robot_on_object_sensor, T_world_sensor, np.asarray(prior['origin_world_m']))
    return float(axis @ (w[3:] if prior['joint_type']=='revolute' else w[:3]))


class WristFT:
    def __init__(self, robot):
        self.robot = robot
        metadata = robot._articulation_view._metadata
        self.index = int(metadata.joint_indices[SENSOR_JOINT]) + 1
        self.mapping = dict(joint_names=list(metadata.joint_names), joint_indices=dict(metadata.joint_indices),
                            incoming_link_row=self.index, sensor_joint=SENSOR_JOINT)

    def read(self):
        # Native transmitted spatial reaction, including fixed joint. Never a
        # projected motor effort, sum of contacts or object generalized force.
        return np.asarray(self.robot.get_measured_joint_forces(), dtype=float)[self.index].copy()


def tool_gravity_world(body_states, sensor_origin, gravity=(0,0,-9.81)):
    result = np.zeros(6)
    for b in body_states:
        force = b['mass_kg'] * np.asarray(gravity)
        result[:3] += force
        result[3:] += np.cross(np.asarray(b['com_world_m'])-sensor_origin, force)
    return result


def tool_inertial_world(body_states, sensor_origin):
    """Newton-Euler tool wrench from measured body accelerations, no native dq shortcut."""
    result = np.zeros(6)
    for b in body_states:
        force = b['mass_kg'] * np.asarray(b['com_acceleration_world_m_s2'])
        omega = np.asarray(b['omega_world_rad_s']); I = np.asarray(b['inertia_world_kg_m2'])
        moment = I @ np.asarray(b['alpha_world_rad_s2']) + np.cross(omega, I @ omega)
        result[:3] += force
        result[3:] += moment + np.cross(np.asarray(b['com_world_m'])-sensor_origin, force)
    return result


def tool_properties(urdf):
    tree=ET.parse(urdf).getroot();descendants={SENSOR_BODY}
    for _ in tree.findall('link'):
        for j in tree.findall('joint'):
            if j.find('parent').get('link') in descendants:descendants.add(j.find('child').get('link'))
    from scipy.spatial.transform import Rotation
    result={}
    for link in tree.findall('link'):
        n=link.get('name');ins=link.find('inertial')
        if n not in descendants or ins is None:continue
        origin=ins.find('origin');v=ins.find('inertia').attrib
        I=np.array([[float(v['ixx']),float(v['ixy']),float(v['ixz'])],[float(v['ixy']),float(v['iyy']),float(v['iyz'])],[float(v['ixz']),float(v['iyz']),float(v['izz'])]])
        R=Rotation.from_euler('xyz',list(map(float,origin.get('rpy','0 0 0').split()))).as_matrix()
        result[n]=dict(mass_kg=float(ins.find('mass').get('value')),com_body_m=list(map(float,origin.get('xyz','0 0 0').split())),inertia_body_kg_m2=(R@I@R.T).tolist())
    return result


def install_passive(g):
    """Append the native cut measurement. No contact/control decisions here."""
    sensor=WristFT(g['robot']);props=tool_properties(g['xarm_config']['sim_urdf']);old=g['observe_step']
    def pose(n):
        p,q=g['actual_views'][n].get_world_poses();return g['matrix'](p[0],q[0])
    def observe_step(state,contacts):
        old(state,contacts)
        try:
            T=pose(SENSOR_BODY);bodies=[]
            for n,b in props.items():
                B=pose(n);bodies.append(dict(name=n,mass_kg=b['mass_kg'],T_world_body=B.tolist(),
                    com_world_m=(B[:3,:3]@b['com_body_m']+B[:3,3]).tolist(),inertia_world_kg_m2=(B[:3,:3]@b['inertia_body_kg_m2']@B[:3,:3].T).tolist()))
            raw=sensor.read()
            state['wrist_ft']=dict(raw_wrench_sensor=raw.tolist(),T_world_sensor=T.tolist(),valid=bool(np.isfinite(raw).all()),
                timestamp_s=state['t'],world_simulation_timestamp_s=float(g['world'].current_time),sample_period_s=g['dt'],body_states=bodies,
                tool_gravity_wrench_world=tool_gravity_world(bodies,T[:3,3]).tolist(),source='native fixed-joint incoming transmitted wrench',
                actuator_output_claim=False)
        except Exception as e:g['diagnostic_issue']('wrist_ft_passive',e)
    g['observe_step']=observe_step
    (g['a'].output/'WRIST_FT_MAPPING.json').write_text(json.dumps(dict(**sensor.mapping,tool_properties=props,configuration=g['xarm_config']['wrist_ft']),indent=2))
