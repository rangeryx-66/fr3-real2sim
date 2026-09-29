"""One honest R1/Dex1 + SAM3 + GraspGenX + PhysX-Mobility door trial."""
import argparse
from datetime import datetime
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
    if args.action != 'open' or args.target_part != 'cabinet door handle':
        raise ValueError('this v1 demo supports opening the 47686 cabinet door handle only')
    plant.URL = f'http://127.0.0.1:{args.port}'
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    tz = ZoneInfo('Asia/Shanghai')
    deadline = (datetime.fromisoformat(args.cutoff_local).astimezone(tz)
                if args.cutoff_local else
                datetime.now(tz).replace(hour=5,minute=0,second=0,microsecond=0))
    if datetime.now(tz) >= deadline:
        raise TimeoutError(f'articulated experiment cutoff reached: {deadline.isoformat()}')
    output = args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    report = {'schema':'r1a7_articulated_demo/v1','asset_id':'47686',
        'task':args.task,'target_part':args.target_part,
        'action':args.action,'deadline':deadline.isoformat(),'stages':[],'candidates':[],
        'asset_installation':{'x_m':args.asset_x,'y_m':args.asset_y,
            'yaw_deg':args.asset_yaw_deg,'fixture_height_m':args.fixture_height_m},
        'camera_offset_m':args.camera_offset,
        'success_criterion':{'joint_delta_deg':20,'hold_s':2,'no_slip':True},
        'status':'STARTED'}
    def save(): (output/'report.json').write_text(json.dumps(report,indent=2,default=str))
    save()
    if args.stage == 'full':
        if not args.sam3_checkpoint or not args.sam3_checkpoint.is_file():
            raise FileNotFoundError('SAM3 checkpoint unavailable; official facebook/sam3 access is required')
        if not args.sam3_python.is_file(): raise FileNotFoundError(args.sam3_python)
        if not args.graspgenx_python.is_file(): raise FileNotFoundError(args.graspgenx_python)
        if not args.filter_python.is_file(): raise FileNotFoundError(args.filter_python)
    env = {**os.environ, 'R1A7_BASE_POSE':'0.329,-0.175,0.237,56.295',
        'R1A7_PEDESTAL_SIZE':'0.10,0.10,0.20','R1A7_PLANT_PORT':str(args.port),
        'R1A7_RUN_DIR':str(output), 'ROS_DOMAIN_ID':str(args.ros_domain),
        'ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost',
        'no_proxy':'127.0.0.1,localhost','OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y'}
    processes = []; node = None; recording = False
    try:
        processes.append(start([SIM,'-u',str(ROOT/'src/r1a7_articulated_sim_server.py'),
            '--gpu',str(args.gpu),'--port',str(args.port),'--asset-root',str(args.asset_root),
            '--asset-x',str(args.asset_x),'--asset-y',str(args.asset_y),
            '--asset-yaw-deg',str(args.asset_yaw_deg),
            '--fixture-height-m',str(args.fixture_height_m),
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
            rclpy.init(); node = ArticulatedBackend(args.asset_root/'urdf/47686.urdf')
            node.scene()
            report['fk_tcp_check'] = node.check_fk()
            report['stages'].append('MOVEIT_SCENE_AND_FK_PASS')
            report['status'] = 'MOTION_SMOKE_PASS'
            return report
        if datetime.now(tz) >= deadline: raise TimeoutError('05:00 cutoff before SAM3')
        sam_cmd = [str(args.sam3_python),str(ROOT/'scripts/infer_sam3_part.py'),
            '--rgb',str(output/'rgb.png'),'--prompt',args.target_part,
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
            subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=900)
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
        variant_candidates = [SimpleNamespace(rank=row['raw_rank'],score=row['score'],
            T_B_TCP=np.asarray(row['T_B_TCP']),variant=row['variant'],
            checks={'dex1_scene_collision':row['collision'],
                    'target_distance_m':row['target_distance_m'],
                    'adaptation_translation_m':row['translation_m'],
                    'adaptation_rotation_rad':row['rotation_rad']})
            for row in variants['candidates']]
        candidate_pool = filtered + variant_candidates
        if not candidate_pool: raise Failure('NO_GRASP','no Dex1 collision-free handle candidates')
        if datetime.now(tz) >= deadline: raise TimeoutError('05:00 cutoff before MoveIt')
        rclpy.init(); node = ArticulatedBackend(args.asset_root/'urdf/47686.urdf')
        node.scene()
        report['fk_tcp_check'] = node.check_fk()
        report['stages'].append('MOVEIT_READY'); save()
        chosen = None
        for candidate in candidate_pool[:args.max_candidates]:
            if datetime.now(tz) >= deadline: raise TimeoutError('05:00 cutoff during candidate search')
            record = {'rank':candidate.rank,'score':candidate.score,
                      'variant':getattr(candidate,'variant','raw'),
                      'T_B_TCP':candidate.T_B_TCP.tolist(),'checks':candidate.checks}
            report['candidates'].append(record)
            if candidate.checks['dex1_scene_collision']['status']!='FREE':
                record['status']='COLLISION'; continue
            try:
                plan = node.candidate_plan(candidate)
                record.update(status='EXECUTABLE',grasp_margin_rad=plan['grasp_margin_rad'],
                              path_margin_rad=plan['path_margin_rad'])
                chosen = (candidate,plan); break
            except Failure as error:
                record.update(status=error.category,detail=str(error))
            save()
        if chosen is None: raise Failure('NO_PLAN','no complete pregrasp/approach candidate')
        report['selected_rank'] = chosen[0].rank; save()
        video = output/'execution.mp4'
        if not plant.command({'op':'video_start','path':str(video)})['ok']:
            raise RuntimeError('Isaac video recorder failed')
        recording = True
        plan = chosen[1]
        node.stage='PREGRASP'; node.execute(plan['pre_traj'],'NO_PLAN')
        node.stage='APPROACH'; node.execute(node.cartesian(node.measured(),plan['grasp']),'NO_PLAN')
        report['stages'].append('MOVEIT_GRASP_POSE'); save()
        node.stage='CLOSE'; close_result = node.gripper(0.)
        plant.settle(.4)
        closed = plant.state()
        bilateral_force = all(float(force)>.2 for force in closed['forces'])
        report['contact_at_close'] = {'finger_forces_n':closed['forces'],
            'bilateral_force':bilateral_force,'gripper_stalled':bool(close_result.stalled),
            'gripper_reached_goal':bool(close_result.reached_goal)}
        if not bilateral_force:
            raise Failure('BAD_CONTACT','Dex1 lacks bilateral measured handle contact')
        q0 = float(closed['joint_q'])
        if q0 > np.deg2rad(10): raise Failure('BAD_CONTACT','door moved too far during approach')
        report['stages'].append('DEX1_CLOSED'); save()
        target_q = min(q0+np.deg2rad(22),float(plant.state()['joint_limits']['upper'])-.02)
        report['joint_path'] = node.follow_joint(q0,target_q,
            stop_requested=lambda: datetime.now(tz)>=deadline)
        report['stages'].append('ARTICULATION_PATH_EXECUTED'); save()
        hold_start = plant.state()['t']
        plant.settle(2.2)
        final = plant.state()
        joint_delta = float(final['joint_q']-q0)
        report['actual_joint_delta_deg'] = float(np.rad2deg(joint_delta))
        report['actual_joint_q_rad'] = final['joint_q']
        report['joint_history'] = final['history']
        report['trajectories'] = node.executions
        hold = [sample for sample in final['history'] if sample['t'] >= hold_start]
        T_at_close = np.linalg.inv(matrix(closed['moving_pose']['position'],
            closed['moving_pose']['quaternion_wxyz'])) @ matrix(closed['tcp'],closed['tcp_quat'])
        T_at_end = np.linalg.inv(matrix(final['moving_pose']['position'],
            final['moving_pose']['quaternion_wxyz'])) @ matrix(final['tcp'],final['tcp_quat'])
        slip_m = float(np.linalg.norm(T_at_end[:3,3]-T_at_close[:3,3]))
        slip_rad = float((Rotation.from_matrix(T_at_close[:3,:3]).inv() *
            Rotation.from_matrix(T_at_end[:3,:3])).magnitude())
        report['hold'] = {'samples':len(hold),
            'duration_s':float(hold[-1]['t']-hold[0]['t']) if len(hold)>1 else 0.,
            'min_joint_delta_deg':float(np.rad2deg(min(sample['joint_q']-q0 for sample in hold))) if hold else None,
            'relative_tcp_slip_m':slip_m,'relative_tcp_slip_rad':slip_rad,
            'final_finger_forces_n':final['forces']}
        if joint_delta < np.deg2rad(20):
            raise Failure('CONTACT_LOSS','door failed to reach 20 degrees')
        if report['hold']['duration_s'] < 2. or report['hold']['min_joint_delta_deg'] < 20:
            raise Failure('CONTACT_LOSS','door did not remain open 20 degrees for 2 seconds')
        if slip_m > .005 or slip_rad > np.deg2rad(10):
            raise Failure('CONTACT_LOSS','Dex1 slipped relative to the door during hold')
        report['status']='SUCCESS'
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
        if node is not None:
            node.destroy_node(); rclpy.shutdown()
        save(); stop(processes)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',choices=('smoke','motion-smoke','full'),default='full')
    parser.add_argument('--task',default='抓住把手打开柜门')
    parser.add_argument('--target-part',default='cabinet door handle')
    parser.add_argument('--action',default='open')
    parser.add_argument('--output',type=Path,default=ROOT/'results/articulated_47686_demo')
    parser.add_argument('--asset-root',type=Path,default=ASSET)
    parser.add_argument('--asset-x',type=float,default=.45)
    parser.add_argument('--asset-y',type=float,default=.05)
    parser.add_argument('--asset-yaw-deg',type=float,default=-90.)
    parser.add_argument('--fixture-height-m',type=float,default=.18)
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
