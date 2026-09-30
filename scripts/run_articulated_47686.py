"""One honest R1/Dex1 + SAM3 + GraspGenX + PhysX-Mobility door trial."""
import argparse
from datetime import datetime, timedelta
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
from zoneinfo import ZoneInfo

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'src'))
from articulated_demo.backend import ArticulatedBackend, Failure, matrix
from articulated_demo.preflight import candidate_preflight, probe_ik
from collections import Counter
from scipy.spatial.transform import Rotation
from grasp_compare.adapters import read_candidates
from grasp_compare.scene import load_scene
from run_r1a7_generalization import SIM, start, stop
import r1a7_plant as plant
import rclpy

ASSET = Path('/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1')
DEFAULT_GRASPGENX = Path('/data1/home/rangeryx/GraspGenX/.venv/bin/python')
DEFAULT_FILTER = Path('/data1/home/rangeryx/.conda/envs/anygrasp/bin/python')


def ready(port, processes, deadline):
    from urllib.request import urlopen
    while time.monotonic() < deadline:
        if any(proc.poll() is not None for proc in processes):
            raise RuntimeError('Isaac/MoveIt/bridge exited during startup')
        try:
            with urlopen(f'http://127.0.0.1:{port}',timeout=1) as stream:
                state = json.load(stream)
            if state.get('joint_name'):
                return state
        except Exception: pass
        time.sleep(1)
    raise TimeoutError('articulated Isaac plant not ready')


def run(args):
    if args.action != 'open' or args.target_part not in ('cabinet door handle','microwave door handle'):
        raise ValueError('this minimal demo supports opening one door by its handle only')
    manifest=json.loads((args.asset_root/'manifest.json').read_text())
    asset_id=manifest['asset_id']
    plant.URL = f'http://127.0.0.1:{args.port}'
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    tz = ZoneInfo('Asia/Shanghai')
    now = datetime.now(tz)
    deadline = (datetime.fromisoformat(args.cutoff_local).astimezone(tz)
                if args.cutoff_local else
                now.replace(hour=5,minute=0,second=0,microsecond=0))
    if not args.cutoff_local and deadline <= now:
        deadline += timedelta(days=1)
    if args.stage in ('full','preflight') and datetime.now(tz) >= deadline:
        raise TimeoutError(f'articulated experiment cutoff reached: {deadline.isoformat()}')
    output = args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    report = {'schema':'r1a7_articulated_demo/v1','asset_id':asset_id,
        'task':args.task,'target_part':args.target_part,
        'action':args.action,'deadline':deadline.isoformat(),'stages':[],'candidates':[],
        'asset_installation':{'x_m':args.asset_x,'y_m':args.asset_y,
            'yaw_deg':args.asset_yaw_deg,'fixture_height_m':args.fixture_height_m},
        'home_q':args.home_q,
        'camera_offset_m':args.camera_offset,
        'robot_base_pose':args.base_pose,'support_bottom_z_m':args.support_bottom_z,
        'success_criterion':{'joint_delta_deg':20,'hold_s':2,'no_slip':True},
        'status':'STARTED'}
    def save(): (output/'report.json').write_text(json.dumps(report,indent=2,default=str))
    save()
    if args.stage in ('full','preflight'):
        if not args.sam3_checkpoint or not args.sam3_checkpoint.is_file():
            raise FileNotFoundError('SAM3 checkpoint unavailable; official facebook/sam3 access is required')
        if not args.sam3_python.is_file(): raise FileNotFoundError(args.sam3_python)
        if not args.graspgenx_python.is_file(): raise FileNotFoundError(args.graspgenx_python)
        if not args.filter_python.is_file(): raise FileNotFoundError(args.filter_python)
    env = {**os.environ, 'R1A7_BASE_POSE':','.join(map(str,args.base_pose)),
        'R1A7_SUPPORT_BOTTOM_Z':str(args.support_bottom_z),
        'R1A7_PEDESTAL_SIZE':'0.10,0.10,0.20','R1A7_PLANT_PORT':str(args.port),
        'R1A7_RUN_DIR':str(output), 'ROS_DOMAIN_ID':str(args.ros_domain),
        'ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost',
        'no_proxy':'127.0.0.1,localhost','OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
    processes = []; node = None; recording = False
    try:
        # Generate before either process reads it; avoid a startup mount race.
        subprocess.run([sys.executable,str(ROOT/'scripts/prepare_r1a7_description.py')],env=env,check=True)
        processes.append(start([SIM,'-u',str(ROOT/'src/r1a7_articulated_sim_server.py'),
            '--gpu',str(args.gpu),'--port',str(args.port),'--asset-root',str(args.asset_root),
            '--asset-x',str(args.asset_x),'--asset-y',str(args.asset_y),
            '--asset-yaw-deg',str(args.asset_yaw_deg),
            '--fixture-height-m',str(args.fixture_height_m),
            '--door-mass-model',args.door_mass_model,
            '--home-q',*[str(value) for value in args.home_q],
            '--camera-offset',*[str(value) for value in args.camera_offset]],
            output/'isaac.log',env))
        if args.stage != 'smoke':
            processes.append(start(['ros2','launch',str(ROOT/'src/r1a7_moveit.launch.py')],
                output/'moveit.log',env))
            processes.append(start([sys.executable,'-u',str(ROOT/'src/r1a7_ros_bridge.py')],
                output/'bridge.log',env))
        state = ready(args.port,processes,time.monotonic()+150)
        report['stages'].append('ISAAC_READY')
        report['asset_joint'] = {'name':state['joint_name'],'limits':state['joint_limits'],
            'initial_q_rad':state['joint_q'],'moving_link':state['moving_link'],
            'fixture_height_m':state['fixture_height_m']}
        if args.stage in ('preflight','full'):
            plant.settle(3.)
            report['startup_physics']={k:plant.state().get(k) for k in
                ('door_mass_model','mass_audit','startup_joint_range_rad')}
            frozen=plant.command({'op':'pause'},timeout=10)
            if not frozen.get('ok'):raise RuntimeError('Isaac physics pause failed')
            report['planning_joint_q_rad']=float(plant.state()['joint_q'])
            report['stages'].append('PHYSICS_PAUSED_FOR_PLANNING');save()
            if args.stage == 'full' and abs(report['planning_joint_q_rad']) > np.deg2rad(2):
                raise Failure('INITIAL_DOOR_DRIFT',
                    'door moved more than 2 degrees before the 0→22 degree trial')
        capture = output/'capture.npz'
        result = plant.command({'op':'capture','path':str(capture),
            'evaluation_mask_path':str(output/'eval_gt_handle_mask.npy')},timeout=90)
        if not result['ok']: raise RuntimeError(result)
        with np.load(capture) as frame:
            rgb = np.asarray(frame['rgb'],dtype=np.uint8)
            depth = np.asarray(frame['depth_m'])
            points = np.asarray(frame['points'])
            K = np.asarray(frame['K'])
            T_B_C = np.asarray(frame['T_B_C'])
        cv2.imwrite(str(output/'rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        report['capture'] = {'path':str(capture),'sha256':hashlib.sha256(capture.read_bytes()).hexdigest(),
            'rgb_path':str(output/'rgb.png'),'valid_depth_pixels':int(np.sum(np.isfinite(depth)&(depth>.05)))}
        report['capture']['evaluation_only'] = result.get('evaluation_only')
        report['stages'].append('RGBD_CAPTURED'); save()
        if args.stage == 'smoke':
            report['status']='SIM_CAPTURE_SMOKE_PASS'; return report
        if args.stage == 'motion-smoke':
            rclpy.init(); node = ArticulatedBackend(args.asset_root/'urdf'/f'{asset_id}.urdf')
            node.scene()
            report['fk_tcp_check'] = node.check_fk()
            node.validate(node.measured())
            report['home_state_validity']={'valid':True}
            report['stages'].append('MOVEIT_SCENE_AND_FK_PASS')
            report['status'] = 'MOTION_SMOKE_PASS'
            return report
        if args.stage in ('full','preflight') and datetime.now(tz) >= deadline: raise TimeoutError('05:00 cutoff before SAM3')
        sam_cmd = [str(args.sam3_python),str(ROOT/'scripts/infer_sam3_part.py'),
            '--rgb',str(output/'rgb.png'),'--prompt',args.sam3_prompt,
            '--min-score',str(args.sam3_min_score),
            '--center-crop-size',str(args.sam3_center_crop_size),
            '--checkpoint',str(args.sam3_checkpoint),'--output',str(output/'handle_mask.npy')]
        sam_env = env.copy(); sam_env.pop('PYTHONPATH',None)
        sam_env['PYTHONPATH'] = (os.environ.get('SAM3_DEPENDENCY_OVERLAY',
            '/data1/home/rangeryx/sam3_overlay') + ':' +
            os.environ.get('SAM3_ROOT','/data1/home/rangeryx/sam3_official'))
        with (output/'sam3.log').open('w') as log:
            subprocess.run(sam_cmd,env=sam_env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=600)
        mask = np.load(output/'handle_mask.npy').astype(bool)
        if mask.shape != depth.shape: raise RuntimeError('SAM3 mask/camera image shape mismatch')
        valid = np.isfinite(depth)&(depth>.05)&(depth<3.)
        mask &= valid
        if mask.sum()<30: raise RuntimeError('SAM3 handle mask has fewer than 30 valid depth pixels')
        report['sam3'] = json.loads((output/'handle_mask.json').read_text())
        report['sam3']['valid_depth_pixels'] = int(mask.sum())
        oracle = np.load(output/'eval_gt_handle_mask.npy').astype(bool)
        overlap = np.count_nonzero(mask & oracle)
        report['sam3']['evaluation_only_handle_iou'] = float(overlap / max(1,np.count_nonzero(mask | oracle)))
        report['sam3']['evaluation_only_handle_recall'] = float(overlap / max(1,oracle.sum()))
        if report['sam3']['evaluation_only_handle_iou'] < .2:
            raise Failure('BAD_MASK','SAM3 mask does not localize the cabinet handle')
        report['stages'].append('SAM3_HANDLE_MASK'); save()
        scene_path = output/'grasp_scene.npz'
        np.savez_compressed(scene_path,points=points,mask=mask.reshape(-1),
                            T_B_C=T_B_C,rgb=rgb,K=K,depth_m=depth)
        native = output/'graspgenx_native.json'
        cmd = [str(args.graspgenx_python),str(ROOT/'src/infer_graspgenx.py'),
               '--input',str(scene_path),'--output',str(native),'--top-k','100']
        with (output/'graspgenx.log').open('w') as log:
            grasp_env=env.copy();grasp_env.pop('PYTHONPATH',None)
            subprocess.run(cmd,env=grasp_env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=900)
        filter_dir = output/'dex1_filter'
        filter_env = env.copy(); filter_env.pop('PYTHONPATH',None)
        cmd = [str(args.filter_python),str(ROOT/'compare_models.py'),
            '--scene',str(scene_path),'--models','graspgenx',
            '--predictions',f'graspgenx={native}','--output-dir',str(filter_dir),
            '--top-k','100']
        with (output/'dex1_filter.log').open('w') as log:
            subprocess.run(cmd,env=filter_env,stdout=log,stderr=subprocess.STDOUT,
                           check=True,timeout=900)
        common = json.loads((filter_dir/'candidates.json').read_text())
        if common['models']['graspgenx']['status']!='OK':
            raise Failure('COLLISION',common['models']['graspgenx'].get('reason','Dex1 filter failed'))
        scene = load_scene(scene_path)
        raw = read_candidates(native,'graspgenx',scene)
        by_rank = {candidate.rank:candidate for candidate in raw}
        filtered = []
        for entry in common['candidates']['graspgenx']:
            candidate = by_rank[entry['rank']]
            candidate.checks = entry['checks']
            filtered.append(candidate)
        counts = common['models']['graspgenx']['counts']
        report['candidate_counts'] = counts
        report['graspgenx_native'] = str(native)
        report['raw_collision_filter'] = str(filter_dir/'candidates.json')
        report['stages'].append('GRASP_CANDIDATES_FILTERED'); save()
        variants_file = output/'dex1_variants.json'
        cmd = [str(args.filter_python),str(ROOT/'scripts/filter_articulated_variants.py'),
            '--scene',str(scene_path),'--native',str(native),'--output',str(variants_file),
            '--per-raw-limit',str(args.variant_limit),'--top-k','100']
        with (output/'dex1_variants.log').open('w') as log:
            subprocess.run(cmd,env=filter_env,stdout=log,stderr=subprocess.STDOUT,
                           check=True,timeout=900)
        variants = json.loads(variants_file.read_text())
        report['variant_counts'] = variants['counts']
        report['variant_decisions_file'] = str(variants_file)
        variant_candidates = [SimpleNamespace(rank=row['raw_rank'],score=row['score'],
            T_B_TCP=np.asarray(row['T_B_TCP']),variant=row['variant'],
            checks={'dex1_scene_collision':row['collision'],
                    'target_distance_m':row['target_distance_m'],
                    'adaptation_translation_m':row['translation_m'],
                    'adaptation_rotation_rad':row['rotation_rad']})
            for row in variants['candidates']]
        candidate_pool = ([c for c in filtered
                           if c.checks['dex1_scene_collision']['status']=='FREE'] + variant_candidates
                          if args.stage == 'preflight'
                          else variant_candidates + [c for c in filtered
                           if c.checks['dex1_scene_collision']['status']=='FREE'])
        if not candidate_pool: raise Failure('NO_GRASP','no Dex1 collision-free handle candidates')
        if args.stage in ('full','preflight') and datetime.now(tz) >= deadline: raise TimeoutError('05:00 cutoff before MoveIt')
        rclpy.init(); node = ArticulatedBackend(args.asset_root/'urdf'/f'{asset_id}.urdf')
        node.scene()
        report['fk_tcp_check'] = node.check_fk()
        report['stages'].append('MOVEIT_READY'); save()
        try:
            node.validate(node.measured())
            report['home_state_validity']={'valid':True}
        except Failure as error:
            report['home_state_validity']={'valid':False,
                'category':error.category,'detail':str(error)}
            report['status']='HOME_COLLISION';save()
            if args.stage == 'preflight':return report
            raise
        chosen = None
        for candidate in candidate_pool[:args.max_candidates]:
            if args.stage in ('full','preflight') and datetime.now(tz) >= deadline: raise TimeoutError('05:00 cutoff during candidate search')
            record = {'rank':candidate.rank,'score':candidate.score,
                      'variant':getattr(candidate,'variant','raw'),
                      'T_B_TCP':candidate.T_B_TCP.tolist(),'checks':candidate.checks}
            report['candidates'].append(record)
            if candidate.checks['dex1_scene_collision']['status']!='FREE':
                record['status']='COLLISION'; continue
            try:
                if args.stage == 'preflight':
                    preflight = candidate_preflight(node,candidate.T_B_TCP,
                        random_seeds=args.ik_random_seeds,timeout_s=args.ik_timeout_s,
                        optimize_redundancy=True)
                    record.update(preflight)
                    if preflight['status']=='FULL_PATH_PLANNED' and chosen is None:
                        chosen = (candidate,None)
                else:
                    plan={}
                    preflight = candidate_preflight(node,candidate.T_B_TCP,
                        random_seeds=args.ik_random_seeds,timeout_s=args.ik_timeout_s,
                        optimize_redundancy=True,plan_sink=plan)
                    record['full_path_preflight']=preflight
                    if preflight['status']!='FULL_PATH_PLANNED':
                        record['status']=preflight['status']
                        save();continue
                    record.update(status='EXECUTABLE',
                                  path_margin_rad=preflight['full_arc_min_margin_rad'])
                    chosen = (candidate,plan); break
            except Failure as error:
                record.update(status=error.category,detail=str(error))
            save()
        if args.stage == 'preflight':
            home = node.measured()
            report['ik_seed_timeout_ablation'] = []
            for sample in candidate_pool[:min(5,len(candidate_pool))]:
                trials=[]
                for seeds,timeout,optimized in ((0,.1,0),(8,.25,2),(16,.5,4)):
                    probe,_=probe_ik(node,sample.T_B_TCP,home,
                                     random_seeds=seeds,timeout_s=timeout,
                                     optimized_seeds=optimized)
                    trials.append({'random_seeds':seeds,'optimized_seeds':optimized,
                        'timeout_s':timeout,
                        'kinematic_ik':probe['kinematic_ik'],
                        'collision_free':probe['collision_free'],
                        'margin_over_005':probe['margin_over_005']})
                report['ik_seed_timeout_ablation'].append({'rank':sample.rank,
                    'variant':getattr(sample,'variant','raw'),'trials':trials})
            report['candidate_funnel'] = {
                'raw_collision_free':sum(c.checks['dex1_scene_collision']['status']=='FREE' for c in filtered),
                'variant_collision_free':len(variant_candidates),
                'moveit_checked':len(report['candidates']),
                'exact_kinematic_ik':sum(c.get('grasp',{}).get('kinematic_ik',0)>0 for c in report['candidates']),
                'collision_free_ik':sum(c.get('grasp',{}).get('collision_free',0)>0 for c in report['candidates']),
                'margin_over_005':sum(c.get('grasp',{}).get('margin_over_005',0)>0 for c in report['candidates']),
                'margin_over_008':sum(c.get('grasp',{}).get('margin_over_008',0)>0 for c in report['candidates']),
                'pregrasp_ik':sum(c.get('pregrasp',{}).get('kinematic_ik',0)>0 for c in report['candidates']),
                'approach':sum('approach_points' in c for c in report['candidates']),
                'pregrasp_plan':sum('pregrasp_plan_points' in c for c in report['candidates']),
                'full_arc':sum(c.get('status')=='FULL_PATH_PLANNED' for c in report['candidates']),
                'failure_reasons':dict(Counter(c['status'] for c in report['candidates']))}
            report['status']='PREFLIGHT_FULL_PATH' if chosen else 'PREFLIGHT_BLOCKED'
            save()
            return report
        if chosen is None: raise Failure('NO_PLAN','no complete pregrasp/approach candidate')
        report['selected_rank'] = chosen[0].rank; save()
        video = output/'execution.mp4'
        if not plant.command({'op':'video_start','path':str(video)})['ok']:
            raise RuntimeError('Isaac video recorder failed')
        recording = True
        from articulated_demo.execute_preflight import execute_preflight
        execute_preflight(node,chosen[1],report,save,
            stop_requested=lambda: datetime.now(tz)>=deadline)
        return report
    finally:
        if node is not None:
            report['trajectories'] = node.executions
            try:
                live = plant.state()
                report['actual_joint_q_rad'] = live['joint_q']
                report['joint_history'] = live['history']
                report['final_forces'] = live['forces']
            except Exception as error:
                report['state_readback_error'] = str(error)
        if recording:
            try: report['video_result']=plant.command({'op':'video_stop'},timeout=90)
            except Exception as error: report['video_error']=str(error)
        try:
            if node is not None:
                node.destroy_node()
                if rclpy.ok(): rclpy.shutdown()
        finally:
            save(); stop(processes)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=('smoke','motion-smoke','preflight','full'),default='full')
    parser.add_argument('--task',default='抓住把手打开柜门')
    parser.add_argument('--target-part',default='cabinet door handle')
    parser.add_argument('--sam3-prompt',default='metal handle')
    parser.add_argument('--sam3-min-score',type=float,default=.05)
    parser.add_argument('--sam3-center-crop-size',type=int,default=256)
    parser.add_argument('--action',default='open')
    parser.add_argument('--output',type=Path,default=ROOT/'results/articulated_47686_demo')
    parser.add_argument('--asset-root',type=Path,default=ASSET)
    parser.add_argument('--asset-x',type=float,default=.45)
    parser.add_argument('--asset-y',type=float,default=.05)
    parser.add_argument('--asset-yaw-deg',type=float,default=-90.)
    parser.add_argument('--fixture-height-m',type=float,default=.18)
    parser.add_argument('--base-pose',type=float,nargs=4,default=(.329,-.175,.237,56.295),metavar=('X','Y','Z','YAW_DEG'))
    parser.add_argument('--support-bottom-z',type=float,default=0.)
    parser.add_argument('--door-mass-model',choices=('geometry','legacy'),default='geometry')
    parser.add_argument('--home-q',type=float,nargs=7,
                        default=(0.,1.3,1.,-1.3,0.,0.,0.))
    parser.add_argument('--camera-offset',type=float,nargs=3,default=(.20,-.75,.36),
                        metavar=('DX','DY','DZ'))
    parser.add_argument('--sam3-checkpoint',type=Path)
    parser.add_argument('--sam3-python',type=Path,default=Path('/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'))
    parser.add_argument('--graspgenx-python',type=Path,default=DEFAULT_GRASPGENX)
    parser.add_argument('--filter-python',type=Path,default=DEFAULT_FILTER)
    parser.add_argument('--gpu',type=int,default=5)
    parser.add_argument('--port',type=int,default=18795)
    parser.add_argument('--ros-domain',type=int,default=230)
    parser.add_argument('--cutoff-local',default=os.environ.get('ARTICULATED_CUTOFF_LOCAL'),
                        help='ISO 8601 cutoff, e.g. 2026-09-30T05:00:00+08:00')
    parser.add_argument('--max-candidates',type=int,default=80)
    parser.add_argument('--variant-limit',type=int,default=16)
    parser.add_argument('--ik-random-seeds',type=int,default=8)
    parser.add_argument('--ik-timeout-s',type=float,default=.25)
    args=parser.parse_args()
    try:
        report=run(args)
        print(json.dumps({'status':report['status'],'output':str(args.output)}),flush=True)
    except Exception as error:
        report_path=args.output.resolve()/'report.json'
        report=json.loads(report_path.read_text()) if report_path.exists() else {}
        report.update(status=error.category if isinstance(error,Failure) else type(error).__name__,
                      detail=str(error))
        report_path.parent.mkdir(parents=True,exist_ok=True)
        report_path.write_text(json.dumps(report,indent=2,default=str))
        print(json.dumps({'status':report['status'],'detail':report['detail']}),flush=True)
        raise


if __name__=='__main__':main()
