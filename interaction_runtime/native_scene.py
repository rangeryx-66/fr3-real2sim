"""Explicit versioned native scene assembly derived from eb80bf3. No motion/controller owner is imported. Geometry, inertias, drives, contacts and 240 Hz physics are preserved."""

def build(job, c):
    import json
    from types import SimpleNamespace
    from pathlib import Path
    import os
    output = Path(job['output'])
    output.mkdir(parents=True, exist_ok=True)
    source = json.loads(Path(job['source'], 'report.json').read_text())
    place = source['asset_installation']
    a = SimpleNamespace(gpu=job['gpu'], asset_root=Path(job['asset_root']), asset_x=place['x_m'], asset_y=place['y_m'], asset_yaw_deg=place['yaw_deg'], fixture_height_m=place['fixture_height_m'], door_mass_model='geometry', door_sanity_only=None, enable_isolation_diagnostics=True, overview_dir=None, planning_preview_report=None, record_overview=True, camera_offset=(0.2, -0.75, 0.36), output=output)
    os.environ['R1A7_BASE_POSE'] = ','.join(map(str, source['robot_base_pose']))
    os.environ['R1A7_SUPPORT_BOTTOM_Z'] = '-.56'
    os.environ['R1A7_PEDESTAL_SIZE'] = '.10,.10,.46'
    os.environ['R1A7_RUN_DIR'] = str(output)
    os.environ['OMNI_KIT_ACCEPT_EULA'] = 'YES'
    os.environ['ACCEPT_EULA'] = 'Y'
    import argparse
    import hashlib
    import json
    import os
    import queue
    import shutil
    import subprocess
    import threading
    import time
    import itertools
    import sys
    import xml.etree.ElementTree as ET
    from pathlib import Path
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    import numpy as np
    from scipy.spatial.transform import Rotation
    ROOT = Path(__file__).resolve().parents[1]
    pass
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'active_gpu': a.gpu, 'physics_gpu': a.gpu, 'multi_gpu': False, 'anti_aliasing': 0, 'disable_viewport_updates': True})
    try:
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.prims import SingleArticulation, RigidPrim, SingleXFormPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.sensors.camera import Camera
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
        import omni.usd
        DT = 1 / 240
        HOME = np.array([0, -0.5, 0, 1.0, 0, 0.5, 0], dtype=float)
        BASE_POSE = np.array([float(v) for v in os.environ.get('R1A7_BASE_POSE', '0.329,-0.175,0.237,56.3').split(',')])
        PEDESTAL_SIZE = np.array([float(v) for v in os.environ.get('R1A7_PEDESTAL_SIZE', '0.10,0.10,0.20').split(',')])
        manifest = json.loads((a.asset_root / 'manifest.json').read_text())
        asset_id = manifest['asset_id']
        asset_urdf = a.asset_root / 'urdf' / f'{asset_id}.urdf'
        from interactive_twin.visual_import import compatible_urdf
        asset_import_urdf, visual_import_audit = compatible_urdf(asset_urdf, a.asset_root / 'visual_compatibility')
        moving_link = manifest.get('moving_link', 'abstract_2_1')
        door_link_name = manifest.get('door_link', 'l_1')
        asset_usd = a.asset_root / 'visual_compatibility' / 'no_legacy_visual_cache.usd'
        if not asset_usd.is_file():
            version = hashlib.sha256(asset_import_urdf.read_bytes()).hexdigest()[:12]
            cache = a.asset_root / f'usd/imported_path_{version}.txt'
            if cache.exists() and Path(cache.read_text().strip()).is_file():
                asset_usd = Path(cache.read_text().strip())
            else:
                directory = a.asset_root / 'usd' / version
                directory.mkdir(parents=True, exist_ok=True)
                imported = URDFImporter(URDFImporterConfig(urdf_path=str(asset_import_urdf), usd_path=str(directory), fix_base=True, allow_self_collision=False, merge_fixed_joints=False, joint_drive_type='force', joint_target_type='position')).import_urdf()
                asset_usd = Path(imported)
                cache.write_text(str(asset_usd))
        world = World(stage_units_in_meters=1.0, physics_dt=DT, rendering_dt=1 / 30, backend='numpy', device='cpu')
        world.scene.add_default_ground_plane(z_position=-0.76)
        mat = PhysicsMaterial('/World/grasp_material', static_friction=0.8, dynamic_friction=0.7, restitution=0.0)
        world.scene.add(FixedCuboid('/World/table', name='table', position=[0.5, 0, -0.025], scale=[0.9, 0.9, 0.05], physics_material=mat))
        support_bottom = float(os.environ.get('R1A7_SUPPORT_BOTTOM_Z', '0'))
        if BASE_POSE[2] > support_bottom:
            world.scene.add(FixedCuboid('/World/r1a7_pedestal', name='r1a7_pedestal', position=[BASE_POSE[0], BASE_POSE[1], (BASE_POSE[2] + support_bottom) / 2], scale=[PEDESTAL_SIZE[0], PEDESTAL_SIZE[1], BASE_POSE[2] - support_bottom], physics_material=mat))
        stage = omni.usd.get_context().get_stage()
        asset_parent = UsdGeom.Xform.Define(stage, '/World/articulated_pose')
        xyz_op = asset_parent.AddTranslateOp()
        quat_op = asset_parent.AddOrientOp()
        scale = float(manifest['scale_source_to_meters'])
        static_low_y = float(manifest['static_source_bounds'][0][1])
        asset_xyz = np.array([a.asset_x, a.asset_y, -scale * static_low_y])
        asset_xyz[2] += a.fixture_height_m
        source_to_world = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        asset_rotation = Rotation.from_euler('z', a.asset_yaw_deg, degrees=True).as_matrix() @ source_to_world
        handle_bounds = np.asarray(manifest['grasp_mesh_bounds_source'], dtype=float)
        handle_center = asset_xyz + asset_rotation @ (handle_bounds.mean(axis=0) * scale)
        sys.path.insert(0, str(ROOT))
        from articulated_demo.kinematics import URDFChain
        from articulated_demo.fixture_geometry import fixture_box
        asset_chain = URDFChain(asset_urdf)
        fixture = fixture_box(manifest, asset_chain, asset_urdf, asset_rotation, asset_xyz, a.fixture_height_m)
        fixture_size = np.array(fixture['size'])
        fixture_quat = np.array(fixture['quaternion_wxyz'])
        if a.fixture_height_m > 0.0:
            world.scene.add(FixedCuboid('/World/cabinet_fixture', name='cabinet_fixture', position=fixture['center'], scale=fixture_size, orientation=fixture_quat, physics_material=mat))
        asset_quat = np.roll(Rotation.from_matrix(asset_rotation).as_quat(), 1)
        xyz_op.Set(Gf.Vec3d(*asset_xyz))
        quat_op.Set(Gf.Quatf(*asset_quat))
        asset_path = '/World/articulated_pose/asset'
        add_reference_to_stage(str(asset_usd), asset_path)
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(asset_path) and prim.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Get(prim, 'angular')
                if drive:
                    drive.CreateStiffnessAttr().Set(0.0)
                    drive.CreateDampingAttr().Set(0.0)
                    drive.CreateMaxForceAttr().Set(0.0)
        articulation = world.scene.add(SingleArticulation(asset_path, name='cabinet'))
        asset_joint_name = manifest['joint_name']
        asset_cache = ROOT / 'assets'
        asset_cache.mkdir(exist_ok=True)
        model_hash = hashlib.sha256(Path(c['sim_urdf']).read_bytes()).hexdigest()[:12]
        asset_file = asset_cache / f'piper_mobile_asset_path_{model_hash}.txt'
        if asset_file.exists() and Path(asset_file.read_text().strip()).exists():
            robot_usd = asset_file.read_text().strip()
        else:
            robot_usd = URDFImporter(URDFImporterConfig(urdf_path=str(Path(c['sim_urdf'])), usd_path=str(asset_cache), fix_base=True, allow_self_collision=True, merge_fixed_joints=False, joint_drive_type='force', joint_target_type='position')).import_urdf()
            asset_file.write_text(robot_usd)
        if not a.door_sanity_only:
            add_reference_to_stage(robot_usd, '/World/Piper')
            robot = world.scene.add(SingleArticulation('/World/Piper', name='piper'))
            robot.set_world_pose(BASE_POSE[:3], np.roll(Rotation.from_euler('z', BASE_POSE[3], degrees=True).as_quat(), 1))

        def one_prim(name, under):
            matches = [prim for prim in stage.Traverse() if prim.GetName() == name and str(prim.GetPath()).startswith(under) and (prim.HasAPI(UsdPhysics.RigidBodyAPI) or name == 'tcp_link')]
            if len(matches) != 1:
                raise RuntimeError(f'expected one {name} under {under}, got {len(matches)}')
            return str(matches[0].GetPath())
        moving_path = one_prim(moving_link, asset_path)
        moving = SingleXFormPrim(moving_path)
        contact_target_path = one_prim(door_link_name, moving_path)
        door_link = SingleXFormPrim(contact_target_path)
        sys.path.insert(0, str(ROOT))
        from articulated_demo.kinematics import URDFChain
        asset_chain = URDFChain(asset_urdf)
        mass_audit = []
        geometry_bounds = np.asarray(manifest['moving_source_bounds']) * scale
        geometry_center = geometry_bounds.mean(axis=0)
        geometry_size = geometry_bounds[1] - geometry_bounds[0]
        urdf_links = {l.get('name'): l for l in ET.parse(asset_urdf).findall('link')}
        for link_name, link_path in ((moving_link, moving_path), (door_link_name, contact_target_path)):
            api = UsdPhysics.MassAPI(stage.GetPrimAtPath(link_path))
            previous = {'center_of_mass': list(api.GetCenterOfMassAttr().Get()), 'diagonal_inertia': list(api.GetDiagonalInertiaAttr().Get()), 'mass_kg': float(api.GetMassAttr().Get())}
            mass = float(urdf_links[link_name].find('inertial/mass').get('value'))
            local = np.linalg.inv(asset_chain.root_to_link(link_name, {})) @ np.r_[geometry_center, 1.0]
            inertia = mass / 12 * np.array([geometry_size[1] ** 2 + geometry_size[2] ** 2, geometry_size[0] ** 2 + geometry_size[2] ** 2, geometry_size[0] ** 2 + geometry_size[1] ** 2])
            if a.door_mass_model == 'geometry':
                api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*local[:3]))
                api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*inertia))
                api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))
            mass_audit.append({'link': link_name, 'before': previous, 'geometry_com_local_m': local[:3].tolist(), 'geometry_inertia_kg_m2': inertia.tolist()})
        static_bounds = np.asarray(manifest['static_source_bounds'], dtype=float) * scale

        def oriented_box(bounds, world_matrix):
            """Convert a USD box and its live transform to a MoveIt oriented box."""
            low = np.asarray(bounds.GetMin(), dtype=float)
            high = np.asarray(bounds.GetMax(), dtype=float)
            center_local = Gf.Vec3d(*(low + high) / 2)
            center = np.asarray(world_matrix.Transform(center_local), dtype=float)
            basis = np.column_stack([np.asarray(world_matrix.TransformDir(Gf.Vec3d(*axis)), dtype=float) for axis in np.eye(3)])
            lengths = np.linalg.norm(basis, axis=0)
            rotation = basis / lengths
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=0.0001):
                raise RuntimeError('cabinet USD collision transform contains shear')
            quat = np.roll(Rotation.from_matrix(rotation).as_quat(), 1)
            return {'center': center.tolist(), 'size': ((high - low) * lengths).tolist(), 'quaternion_wxyz': quat.tolist()}

        def collision_boxes():
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
            door_bound = cache.ComputeWorldBound(stage.GetPrimAtPath(moving_path))
            static = {'id': 'cabinet_static', 'center': (asset_xyz + asset_rotation @ static_bounds.mean(axis=0)).tolist(), 'size': (static_bounds[1] - static_bounds[0]).tolist(), 'quaternion_wxyz': asset_quat.tolist()}
            door = {'id': 'cabinet_door', **oriented_box(door_bound.GetBox(), door_bound.GetMatrix())}
            return [static, door, {'id': 'cabinet_fixture', 'center': fixture['center'], 'size': fixture_size.tolist(), 'quaternion_wxyz': fixture_quat.tolist()}]
        contact_paths = [one_prim(name, '/World/Piper') for name in ('left_finger', 'right_finger')]
        for path in contact_paths:
            PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.0)
        finger_views = [world.scene.add(RigidPrim(prim_paths_expr=path, name=f'finger_contact_{i}', contact_filter_prim_paths_expr=[contact_target_path], track_contact_forces=True, prepare_contact_sensors=True, max_contact_count=256)) for i, path in enumerate(contact_paths)]
        body_views = []
        if a.enable_isolation_diagnostics:
            body_views = [world.scene.add(RigidPrim(prim_paths_expr=one_prim(name, '/World/Piper'), name=f'finger_body_contact_{i}', contact_filter_prim_paths_expr=[contact_target_path], track_contact_forces=True, prepare_contact_sensors=True, max_contact_count=256)) for i, name in enumerate(('xarm_gripper_base_link', 'link6'))]
        door_contacts = None
        if a.enable_isolation_diagnostics:
            diagnostic_contact_paths = [str(prim.GetPath()) for prim in stage.Traverse() if str(prim.GetPath()).startswith(asset_path) and (not str(prim.GetPath()).startswith(moving_path)) and prim.HasAPI(UsdPhysics.RigidBodyAPI)] + [path for path in ['/World/table', '/World/cabinet_fixture'] if stage.GetPrimAtPath(path).IsValid()]
            door_contacts = world.scene.add(RigidPrim(prim_paths_expr=contact_target_path, name='door_diagnostic_contacts', contact_filter_prim_paths_expr=diagnostic_contact_paths, track_contact_forces=True, prepare_contact_sensors=True, max_contact_count=256))
        scene_monitor_views = []
        for monitor_name in ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'xarm_gripper_base_link', 'left_outer_knuckle', 'left_inner_knuckle', 'right_outer_knuckle', 'right_inner_knuckle', 'left_finger', 'right_finger']:
            filters = list(diagnostic_contact_paths)
            if monitor_name not in ['left_finger', 'right_finger']:
                filters.append(contact_target_path)
            scene_monitor_views.append((monitor_name, world.scene.add(RigidPrim(prim_paths_expr=one_prim(monitor_name, '/World/Piper'), name='scene_contact_' + monitor_name, contact_filter_prim_paths_expr=filters, track_contact_forces=True, prepare_contact_sensors=True, max_contact_count=256))))
        tcp = SingleXFormPrim(one_prim('tcp_link', '/World/Piper'))
        eye = handle_center + np.array(a.camera_offset)
        focus = handle_center + [0.0, 0.0, 0.02]
        forward = (focus - eye) / np.linalg.norm(focus - eye)
        right = np.cross(forward, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        down = np.cross(forward, right)
        T_B_C = np.eye(4)
        T_B_C[:3, :3] = np.column_stack((right, down, forward))
        T_B_C[:3, 3] = eye
        camera = Camera('/World/camera', position=eye, resolution=(640, 480), frequency=30)
        camera.set_world_pose(position=eye, orientation=np.roll(Rotation.from_matrix(T_B_C[:3, :3]).as_quat(), 1), camera_axes='ros')
        camera.set_clipping_range(0.05, 3.0)
        from interactive_twin.plant import author_passive_resistance
        plant_resistance_audit = author_passive_resistance(stage, asset_path, {})
        from interactive_twin.initial_state import restore_baked_mass_properties
        baked_mass_audit = restore_baked_mass_properties(stage, asset_path, asset_urdf, manifest)
        from interactive_twin_refinement.assembly import capture_assembly
        capture_assembly(locals(), Path(os.environ['R1A7_RUN_DIR']) / 'native_authored_assembly_private.json')
        from xarm7_native_gripper import configure_stage, configuration, NativeDrive, GRIPPER_JOINTS, NativeGripperGeometry
        xarm_config = c
        configure_stage(stage, '/World/Piper', xarm_config, Path(os.environ['R1A7_RUN_DIR']))
        from xarm_ft_v1_scene import author
        case = json.loads(Path(job['private_physics_configuration']).read_text())[job.get('identification_case', 0)]
        _id_physics_audit = author(stage, locals(), case, a.output)
        del case
        world.reset()
        # This legacy scene camera is diagnostic-only. Do not allocate a depth
        # render product when no operation requests it. Wrist RGB-D is installed
        # independently and retains its original calibration and resolution.
        overview = []
        if a.overview_dir or a.planning_preview_report or a.record_overview:
            focus = np.array([a.asset_x + 0.05, a.asset_y - 0.05, 0.32])
            for index, offset in enumerate(([1.1, -1.4, 0.9], [-1.1, -1.1, 0.8])):
                position = focus + offset
                direction = (focus - position) / np.linalg.norm(focus - position)
                xaxis = np.cross(direction, [0.0, 0.0, 1.0])
                xaxis /= np.linalg.norm(xaxis)
                rotation = np.column_stack((xaxis, np.cross(direction, xaxis), direction))
                view = Camera(f'/World/overview_{index}', position=position, resolution=(1280, 960), frequency=30)
                view.set_focal_length(2.0)
                view.set_world_pose(position=position, orientation=np.roll(Rotation.from_matrix(rotation).as_quat(), 1), camera_axes='ros')
                # Runtime initializes only the diagnostic view it actually
                # records, on the first requested event/EVALUATION frame.
                overview.append(view)
        names = robot.dof_names
        arm = [names.index(f'joint{i}') for i in range(1, 8)]
        fingers = [names.index(n) for n in GRIPPER_JOINTS]
        asset_names = articulation.dof_names
        if asset_names != [asset_joint_name]:
            raise RuntimeError(f'asset DOF mismatch: {asset_names}, expected {asset_joint_name}')
        controller = robot.get_articulation_controller()
        kp = np.zeros(len(names))
        kd = np.zeros(len(names))
        kp[arm] = [1200.0, 1400.0, 1200.0, 850.0, 500.0, 500.0, 300.0]
        kd[arm] = [10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0]
        kp[fingers] = 1000.0
        kd[fingers] = 40.0
        controller.set_gains(kps=kp, kds=kd, save_to_usd=True)
        q = np.zeros(len(names))
        q[arm] = HOME
        q[fingers] = NativeGripperGeometry(xarm_config).angle(0.084)
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros(len(q)))
        controller = NativeDrive(robot, xarm_config, Path(os.environ['R1A7_RUN_DIR']))
        controller.apply_action(ArticulationAction(joint_positions=q[arm], joint_velocities=np.zeros(7), joint_indices=arm))
        controller.command_width(0.084)
        articulation.set_joint_positions(np.array([0.0]))
        articulation.set_joint_velocities(np.array([0.0]))
        startup_angles = []
        xarm_startup_records = []
        for i in range(240):
            controller.apply_action(ArticulationAction(joint_positions=q[arm], joint_velocities=np.zeros(7), joint_indices=arm))
            world.step(render=False)
            try:
                xarm_startup_records.append({'step': i, 't': float(world.current_time), 'q': robot.get_joint_positions().tolist(), 'dq': robot.get_joint_velocities().tolist(), 'projected_efforts': robot.get_measured_joint_efforts().tolist(), 'joint_reactions': robot.get_measured_joint_forces().tolist(), 'actuator': controller.last.copy()})
            except Exception:
                pass
            startup_angles.append(float(articulation.get_joint_positions()[0]))
        try:
            native_tensor = {}
            for method in ['get_masses', 'get_inertias', 'get_coms']:
                native_tensor[method] = np.asarray(getattr(robot._articulation_view._physics_view, method)()).tolist()
            (Path(os.environ['R1A7_RUN_DIR']) / 'NATIVE_MASS_TENSOR_READBACK.json').write_text(json.dumps(native_tensor, indent=2))
        except Exception:
            pass
        return locals()
    except BaseException as error:
        # Isaac fast shutdown may exit before the caller can export the error.
        import traceback
        (output/'INITIALIZATION_FAILURE.json').write_text(json.dumps(dict(error=repr(error),traceback=traceback.format_exc()),indent=2))
        app.close()
        raise
