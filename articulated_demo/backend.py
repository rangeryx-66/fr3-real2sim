"""MoveIt executor for a grasped URDF revolute part; Isaac owns the physics."""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from r1a7_backend import R1A7Backend, Failure, BASE, TOUCH, BASE_POSE, PEDESTAL_SIZE
from r1a7_backend import JOINTS, TCP
import r1a7_plant as plant
from urdf_chain import KinematicChain
from .kinematics import URDFChain


def matrix(position, quaternion_wxyz):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat(np.roll(quaternion_wxyz, -1)).as_matrix()
    out[:3, 3] = position
    return out


class ArticulatedBackend(R1A7Backend):
    def __init__(self, asset_urdf):
        super().__init__()
        self.chain = URDFChain(asset_urdf)
        self.robot_chain = KinematicChain(ROOT / 'config/r1a7_dex1.urdf',
            BASE, TCP, JOINTS)
        self.stage = 'READY'
        self.executions = []
        self.door_collision_meshes = self._door_collision_meshes(asset_urdf)

    @staticmethod
    def _door_collision_meshes(asset_urdf):
        """Load the seven convex collision STLs in the asset's moving link."""
        path=Path(asset_urdf)
        root=ET.parse(path).getroot()
        link=next(l for l in root.findall('link') if l.get('name')=='l_1')
        pieces=[]
        for collision in link.findall('collision'):
            spec=collision.find('geometry/mesh')
            if spec is None:continue
            scale=np.fromstring(spec.get('scale','1 1 1'),sep=' ')
            if len(scale)!=3:raise ValueError('invalid cabinet mesh scale')
            mesh_path=(path.parent/spec.get('filename')).resolve()
            coordinates=[]
            for line in mesh_path.read_text().splitlines():
                tokens=line.split()
                if tokens and tokens[0]=='vertex':
                    coordinates.append(np.asarray([float(x) for x in tokens[1:4]])*scale)
            if not coordinates or len(coordinates)%3:
                raise ValueError(f'unsupported or malformed cabinet collision STL: {mesh_path}')
            mesh=Mesh()
            mesh.vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2]))
                           for v in coordinates]
            mesh.triangles=[MeshTriangle(vertex_indices=[i,i+1,i+2])
                            for i in range(0,len(coordinates),3)]
            origin=collision.find('origin')
            xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            rpy=np.fromstring(origin.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();T[:3,3]=xyz
            pieces.append((mesh,T))
        if len(pieces)!=7:
            raise ValueError(f'expected 7 official cabinet door collision meshes, got {len(pieces)}')
        return pieces

    def measured(self):
        """Clamp submillimetre Isaac finger servo drift at the URDF bounds.

        The open Dex1 target is exactly -0.02 m. Isaac reports a few microns
        beyond it while settled; passing that value into MoveIt makes a valid
        arm IK appear as NO_IK. Larger excursions are retained for diagnosis.
        """
        state = super().measured()
        q = list(state.joint_state.position)
        for index, name in enumerate(state.joint_state.name):
            if name not in self.limits:
                continue
            lower, upper = self.limits[name]
            if lower - 1e-4 <= q[index] < lower:
                q[index] = lower
            elif upper < q[index] <= upper + 1e-4:
                q[index] = upper
        state.joint_state.position = q
        return state

    def robust_ik(self, T, seed):
        """Use URDF optimization as a 7D KDL seed, then MoveIt for authority.

        This does not replace MoveIt's collision and joint-margin validation.
        It prevents narrow redundant-arm basins from being called NO_IK after
        a handful of random KDL starts.
        """
        failures = []
        try:
            return self.ik(T, seed, random_seeds=8)
        except Failure as error:
            failures.append(error)
        limits = np.asarray([self.limits[name] for name in JOINTS])
        lower = limits[:,0] + .01
        upper = limits[:,1] - .01
        seed_values = dict(zip(seed.joint_state.name,seed.joint_state.position))
        initial = np.array([seed_values[name] for name in JOINTS])
        T = np.asarray(T)
        target_rotation = Rotation.from_matrix(T[:3,:3])
        def residual(q):
            predicted = self.robot_chain.forward(dict(zip(JOINTS,q)))
            rotation = (target_rotation.inv() *
                        Rotation.from_matrix(predicted[:3,:3])).as_rotvec()
            return np.r_[predicted[:3,3]-T[:3,3], .08*rotation]
        for q0 in [initial,*self.rng.uniform(lower,upper,size=(7,7))]:
            fit = least_squares(residual,np.clip(q0,lower,upper),
                bounds=(lower,upper),max_nfev=100,ftol=1e-4,
                xtol=1e-4,gtol=1e-4)
            if (np.linalg.norm(fit.fun[:3]) > .005 or
                    np.linalg.norm(fit.fun[3:])/.08 > np.deg2rad(5)):
                continue
            candidate = type(seed)()
            candidate.joint_state.name = list(seed.joint_state.name)
            values = dict(seed_values)
            values.update(zip(JOINTS,fit.x))
            candidate.joint_state.position = [float(values[name])
                for name in candidate.joint_state.name]
            try:
                return self.ik(T,candidate,random_seeds=0)
            except Failure as error:
                failures.append(error)
        for category in ('COLLISION','LOW_JOINT_MARGIN','JOINT_LIMIT'):
            selected = next((error for error in failures if error.category == category),None)
            if selected is not None:
                raise selected
        raise failures[0]

    def scene(self, collision_boxes=None, moving_pose_override=None,
              door_collision_mode='mesh'):
        """Keep the mounted cabinet in the world; allow only finger-door touch."""
        state = plant.state()
        req = ApplyPlanningScene.Request()
        scene = req.scene
        scene.is_diff = True
        scene.robot_state.is_diff = True
        geometry = [{'id':'table','center':[.5, 0, -.025],
                     'size':[.9, .9, .05]}]
        if BASE_POSE[2] > 0:
            geometry.append({'id':'r1a7_pedestal',
                'center':[BASE_POSE[0], BASE_POSE[1], PEDESTAL_SIZE[2]/2],
                'size':list(PEDESTAL_SIZE)})
        geometry += state['collision_boxes'] if collision_boxes is None else collision_boxes
        for item in geometry:
            name,center,size = item['id'],item['center'],item['size']
            if name=='cabinet_door' and door_collision_mode=='mesh':
                continue
            obj = CollisionObject(); obj.id = name; obj.header.frame_id = BASE
            obj.operation = CollisionObject.ADD
            shape = SolidPrimitive(); shape.type = SolidPrimitive.BOX
            shape.dimensions = [float(x) for x in size]
            pose = Pose(); pose.position.x,pose.position.y,pose.position.z = map(float,center)
            quat = item.get('quaternion_wxyz',[1.,0.,0.,0.])
            pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z = map(float,quat)
            obj.primitives = [shape]; obj.primitive_poses = [pose]
            scene.world.collision_objects.append(obj)
        if door_collision_mode=='mesh':
            moving=(matrix(state['moving_pose']['position'],
                           state['moving_pose']['quaternion_wxyz'])
                    if moving_pose_override is None else moving_pose_override)
            l1=moving @ self.chain.joints['l_1'].origin
            if moving_pose_override is None and 'door_link_pose' in state:
                measured_link=matrix(state['door_link_pose']['position'],
                                     state['door_link_pose']['quaternion_wxyz'])
                position_error=np.linalg.norm(l1[:3,3]-measured_link[:3,3])
                angle_error=(Rotation.from_matrix(l1[:3,:3]).inv()*
                             Rotation.from_matrix(measured_link[:3,:3])).magnitude()
                if position_error>.002 or angle_error>.02:
                    raise Failure('FRAME_ERROR',
                        f'URDF/Isaac cabinet moving link mismatch: {position_error:.4f} m, '
                        f'{angle_error:.4f} rad')
            door=CollisionObject();door.id='cabinet_door';door.header.frame_id=BASE
            door.operation=CollisionObject.ADD
            for mesh,local in self.door_collision_meshes:
                T=l1@local
                pose=Pose()
                pose.position.x,pose.position.y,pose.position.z=map(float,T[:3,3])
                quat=Rotation.from_matrix(T[:3,:3]).as_quat()
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=map(float,quat)
                door.meshes.append(mesh);door.mesh_poses.append(pose)
            scene.world.collision_objects.append(door)
        get = GetPlanningScene.Request()
        get.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = self.call('get_planning_scene',get).scene.allowed_collision_matrix
        for name in ['cabinet_door'] + TOUCH:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values: row.enabled.append(False)
                row = AllowedCollisionEntry(); row.enabled = [False] * len(acm.entry_names)
                acm.entry_values.append(row)
        target_index = acm.entry_names.index('cabinet_door')
        for name in TOUCH:
            index = acm.entry_names.index(name)
            acm.entry_values[target_index].enabled[index] = True
            acm.entry_values[index].enabled[target_index] = True
        scene.allowed_collision_matrix = acm
        if not self.call('apply_planning_scene',req).success:
            raise Failure('NO_PLAN','MoveIt articulated planning scene rejected')

    def candidate_plan(self, candidate):
        grasp = candidate.T_B_TCP
        pre = grasp.copy(); pre[:3, 3] -= .08 * grasp[:3, 2]
        self.stage = 'CANDIDATE_CHECK'
        home = self.measured()
        grasp_state = self.robust_ik(grasp, home)
        pre_state = self.robust_ik(pre, grasp_state)
        approach = self.cartesian(pre_state, grasp)
        self.require_margin(self.trajectory_margin(approach), 'approach')
        pre_traj = self.plan(home, pre_state)
        return {'pre':pre,'grasp':grasp,'pre_traj':pre_traj,'approach':approach,
                'grasp_margin_rad':self.margin(grasp_state),
                'path_margin_rad':min(self.trajectory_margin(pre_traj),
                                      self.trajectory_margin(approach))}

    def follow_joint(self, q_start, q_goal, *, step_rad=math.radians(2), stop_requested=None):
        """Follow URDF-derived TCP arc; never command the cabinet's joint."""
        if q_goal <= q_start: raise Failure('JOINT_LIMIT','opening direction unavailable')
        steps = int(np.ceil((q_goal-q_start)/step_rad))
        initial = plant.state()
        T_moving = matrix(initial['moving_pose']['position'],
                          initial['moving_pose']['quaternion_wxyz'])
        T_tcp = matrix(initial['tcp'],initial['tcp_quat'])
        T_moving_tcp = np.linalg.inv(T_moving) @ T_tcp
        record = []
        for index in range(1,steps+1):
            if stop_requested is not None and stop_requested():
                raise Failure('CUTOFF_05_00','05:00 experiment cutoff reached')
            before = plant.state()
            current_q = float(before['joint_q'])
            target_q = min(q_start+index*step_rad,q_goal)
            target_pose = self.chain.target_tcp(moving_link=before['moving_link'],
                joint_name=before['joint_name'],q_now=current_q,q_target=target_q,
                T_world_moving_now=matrix(before['moving_pose']['position'],
                    before['moving_pose']['quaternion_wxyz']),
                T_world_tcp_now=matrix(before['tcp'],before['tcp_quat']))
            # Measure the live contact transform: a 5 mm or 10 deg drift is slip.
            current_relative = np.linalg.inv(matrix(before['moving_pose']['position'],
                before['moving_pose']['quaternion_wxyz'])) @ matrix(before['tcp'],before['tcp_quat'])
            translation_error = np.linalg.norm(current_relative[:3,3]-T_moving_tcp[:3,3])
            rotation_error = (Rotation.from_matrix(T_moving_tcp[:3,:3]).inv() *
                              Rotation.from_matrix(current_relative[:3,:3])).magnitude()
            if translation_error > .005 or rotation_error > math.radians(10):
                raise Failure('CONTACT_LOSS',f'TCP moved relative to door: {translation_error:.4f} m, {rotation_error:.3f} rad')
            self.scene()
            self.stage = f'OPEN_{index}'
            trajectory = self.cartesian(self.measured(),target_pose)
            self.execute(trajectory,'NO_PLAN')
            after = plant.state()
            self.scene()
            self.validate(self.measured())
            after_relative = np.linalg.inv(matrix(after['moving_pose']['position'],
                after['moving_pose']['quaternion_wxyz'])) @ matrix(after['tcp'],after['tcp_quat'])
            after_translation_error = float(np.linalg.norm(after_relative[:3,3]-T_moving_tcp[:3,3]))
            after_rotation_error = float((Rotation.from_matrix(T_moving_tcp[:3,:3]).inv() *
                Rotation.from_matrix(after_relative[:3,:3])).magnitude())
            record.append({'requested_joint_q':target_q,'actual_joint_q':after['joint_q'],
                'tcp':after['tcp'],'forces':after['forces'],
                'joint_margin_rad':self.trajectory_margin(trajectory),
                'relative_tcp_slip_m':after_translation_error,
                'relative_tcp_slip_rad':after_rotation_error})
            if after_translation_error > .005 or after_rotation_error > math.radians(10):
                raise Failure('CONTACT_LOSS','Dex1 moved relative to the door during opening')
            if not all(float(force) > .2 for force in after['forces']):
                raise Failure('CONTACT_LOSS','Dex1 lost bilateral handle contact during opening')
            if index >= 3 and after['joint_q']-q_start < math.radians(1):
                raise Failure('JOINT_STUCK','robot moved but cabinet joint did not follow')
            if after['joint_q'] < current_q-math.radians(2):
                raise Failure('CONTACT_LOSS','cabinet joint moved opposite opening direction')
        return record
