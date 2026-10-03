"""Scene-only mobile hook; delegates manipulation to the frozen episode runner."""
import json
import time
from pathlib import Path


def bootstrap_mobile(args, final_base, job):
    import numpy as np
    from scipy.spatial.transform import Rotation
    import interactive_twin.plant as plant
    original_adapt = plant.adapt_loader_source
    initial = job.get('mobile_route', {}).get('initial_base', final_base)

    def add_chassis(source, config):
        source = original_adapt(source, config)
        marker = 'world.reset(); camera.initialize()'
        if source.count(marker) != 1:
            raise RuntimeError('FROZEN_LOADER_RESET_HOOK_CHANGED')
        creation = """
world.scene.add(FixedCuboid('/World/mobile_chassis',name='mobile_chassis',
 position=np.array([BASE_POSE[0],BASE_POSE[1],-.66]),scale=np.array([.34,.30,.20]),
 orientation=np.roll(Rotation.from_euler('z',BASE_POSE[3],degrees=True).as_quat(),1),
 physics_material=mat))
"""
        return source.replace(marker, creation+'\n'+marker)

    plant.adapt_loader_source = add_chassis
    try:
        scene = _ORIGINAL_BOOTSTRAP(args, initial, job)
    finally:
        plant.adapt_loader_source = original_adapt
    if job.get('mobile_route'):
        try:
            execute_route(scene, args, job['mobile_route'], final_base)
        except Exception as error:
            (args.output/'report.json').write_text(json.dumps({'status':str(error),'success':False,
                 'failure_stage':'base_reposition','bilateral_hold_established':False,
                 'object_actuation':False,'attachments':False},indent=2))
            scene['app'].close()
            raise
    return scene


def execute_route(scene, args, route, final_base):
    import numpy as np
    import cv2
    import subprocess
    import shutil
    from scipy.spatial.transform import Rotation
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.types import ArticulationAction
    from piper_mobile_demo.cooked_geometry import export_cooked
    from interaction_identification.contact_probe import robot_only_model
    from articulated_interaction.physical_baseline import WholeFingerReports
    from interactive_twin_recovery.mobile import MobileRuntimeScene
    if not route.get('valid') or not route.get('arm_locked_at_home'):
        raise RuntimeError('UNVERIFIED_BASE_ROUTE')
    root = Path(__file__).resolve().parents[1]
    world, robot = scene['world'], scene['robot']
    dt = scene['DT']; arm = scene['arm']; fingers = scene['fingers']
    model = robot_only_model(root/'config/piper.urdf')
    export_cooked(scene['stage'], args.output/'mobile_initial_cooked.json')
    export = json.loads((args.output/'mobile_initial_cooked.json').read_text())
    export['shapes'] = [e for e in export['shapes'] if e['path']!='/World/mobile_chassis']
    initial = route['initial_base']
    allowed = [e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]
    support = SingleXFormPrim('/World/r1a7_pedestal')
    chassis = SingleXFormPrim('/World/mobile_chassis')
    q_home = np.asarray(robot.get_joint_positions(), float).copy()
    current = list(initial); records = []; tick = 0
    guard = MobileRuntimeScene(root,export,model,initial)
    monitor = WholeFingerReports(scene['stage'], export, allowed, world, dt, lambda: {})
    video = subprocess.Popen([shutil.which('ffmpeg'),'-y','-f','rawvideo','-pix_fmt','rgb24',
        '-s','1280x960','-r','30','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21',
        '-pix_fmt','yuv420p',str(args.output/'base_reposition.mp4')],stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,stderr=(args.output/'mobile_ffmpeg.log').open('w'))
    route_wall_start = time.time()
    try:
        for target in route['waypoints'][1:]:
            delta = np.subtract(target, current)
            duration = max(np.linalg.norm(delta[:2])/.01, abs(delta[3])/5., dt)
            for fraction in np.linspace(0.,1.,max(2,int(np.ceil(duration/dt))+1))[1:]:
                if time.time() >= route['deadline_unix_s']:
                    raise RuntimeError('CUTOFF_05_00_DURING_BASE_ROUTE')
                pose = (np.asarray(current)+fraction*delta).tolist()
                quat = np.roll(Rotation.from_euler('z',pose[3],degrees=True).as_quat(),1)
                robot.set_world_pose(np.asarray(pose[:3]),quat)
                support.set_world_pose(np.array([pose[0],pose[1],-.33]),quat)
                chassis.set_world_pose(np.array([pose[0],pose[1],-.66]),quat)
                scene['controller'].apply_action(ArticulationAction(joint_positions=q_home))
                monitor.clear(); world.step(render=False,update_fabric=True)
                q = np.asarray(robot.get_joint_positions(),float)
                P = model.poses(q[arm],pose,finger_q=q[fingers])
                ok, why = True, 'SAFE'
                if tick%8==0:
                    ok, why = guard.check(P,pose)
                    actual_position,actual_quaternion=robot.get_world_pose()
                    if np.linalg.norm(np.asarray(actual_position)-pose[:3])>1e-4:
                        raise RuntimeError('BASE_ROUTE_ROOT_TRACKING_FAILURE')
                contacts = monitor.state()
                loaded_ok, loaded_why = guard.contact_guard(contacts['contacts'],P)
                if not ok or not loaded_ok or model.margin(q[arm])<=.05:
                    raise RuntimeError('BASE_ROUTE_STOP:'+str(why if not ok else loaded_why if not loaded_ok else 'LOW_JOINT_MARGIN'))
                if tick%8==0:
                    world.render(); image=np.asarray(scene['overview'][0].get_rgba())[:,:,:3].copy()
                    cv2.putText(image,'BASE REPOSITION / ARM HOME / NO MANIPULATION',(12,35),
                                cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
                    video.stdin.write(np.ascontiguousarray(image).tobytes())
                    records.append({'t':tick*dt,'base':pose,'q':q[arm].tolist(),
                                    'joint_margin_rad':float(model.margin(q[arm])),
                                    'contacts':contacts['contacts']})
                tick += 1
                if tick%240==0:
                    (args.output/'base_route_progress.json').write_text(json.dumps({
                        'phase':'BASE_REPOSITION','t':tick*dt,'base':pose,
                        'arm_margin_rad':float(model.margin(q[arm])),
                        'base_locked_for_manipulation':False},indent=2))
            current = list(target)
        # Fixed-base manipulation starts only after a stationary settling window.
        for _ in range(int(1./dt)):
            scene['controller'].apply_action(ArticulationAction(joint_positions=q_home))
            world.step(render=False,update_fabric=True)
        scene['BASE_POSE'] = np.asarray(final_base,float)
        (args.output/'mobile_route_result.json').write_text(json.dumps({
            'status':'REPOSITIONED_STOPPED_LOCKED','initial_base':initial,'final_base':final_base,
            'route':route,'observations':records,'elapsed_wall_s':time.time()-route_wall_start,
            'object_commands':False,'moving_during_manipulation':False,
            'platform_simulation':'kinematic SE2 route; wheel and navigation dynamics unmodeled'},indent=2))
    finally:
        # The frozen manipulation loop registers this same clock name. Route
        # diagnostics must release it before that loop creates its reporter.
        world.remove_physics_callback('piper_owned_contact_clock')
        monitor.subscription = None
        video.stdin.close(); video.wait(timeout=30)
        (args.output/'base_route_observations.json').write_text(json.dumps(records,indent=2))


def install():
    global _ORIGINAL_BOOTSTRAP
    import interactive_twin.plant as plant
    _ORIGINAL_BOOTSTRAP = plant.bootstrap_job
    plant.bootstrap_job = bootstrap_mobile
