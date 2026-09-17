"""Cold-start MoveIt and the isolated Real2Sim simulator for one acquisition."""
from __future__ import annotations
import argparse,json,os,signal,subprocess,time,urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
ISAAC=os.environ.get('ISAAC_PYTHON', '/data1/home/rangeryx/isaaclab-arena/.venv/bin/python')

def stop(process):
    if process and process.poll() is None:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()

def wait_sim(port,process):
    for _ in range(300):
        if process.poll() is not None:raise RuntimeError(f'simulator exited {process.returncode}')
        try:
            state=json.loads(urllib.request.urlopen(f'http://127.0.0.1:{port}/calibration',timeout=2).read())
            if 'calibration' in state:return
        except Exception:pass
        time.sleep(1)
    raise TimeoutError('simulator startup')

def main():
    p=argparse.ArgumentParser();p.add_argument('--target',default='soup');p.add_argument('--seed',type=int,default=1030);p.add_argument('--mode',choices=['GT','ANYGRASP'],default='GT');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gpu',type=int,default=3);p.add_argument('--port',type=int,default=18920);p.add_argument('--domain',type=int,default=120);p.add_argument('--gt-path');p.add_argument('--scan-pass',type=int,default=0);p.add_argument('--scan-profile',choices=['sparse','continuous','held'],default='sparse');p.add_argument('--capture-payload',action='store_true');p.add_argument('--baseline-only',action='store_true');p.add_argument('--center-q');p.add_argument('--skip-scan',action='store_true')
    p.add_argument('--held-regrasp',action='store_true');p.add_argument('--held-keyframes',type=int,default=90);p.add_argument('--held-frames-per-pose',type=int,default=10)
    p.add_argument('--fixed-replay-npz',help='calibration-only rigid replay source capture')
    p.add_argument('--payload-v2',action='store_true',help='use the independent two-stage PayloadID module')
    p.add_argument('--mass-com-only',action='store_true',help='record only the static mass/CoM protocol; skip inertia excitation')
    p.add_argument('--dynamic-only',action='store_true',help='record only the official-style Fourier dynamic protocol')
    p.add_argument('--gripper-opening-mm',type=float,help='calibration-only Franka Hand opening (0..80 mm)')
    p.add_argument('--calibration-force',type=float,default=30.0,
                   help='total bilateral calibration grasp force in N (payload calibration only)')
    p.add_argument('--calibration-mu',type=float,default=.7,
                   help='finger-pad friction used by the calibration simulator')
    p.add_argument('--protocol',default=str(ROOT/'ARENA_COMPLEX_PROTOCOL.json'));p.add_argument('--asset-directory',default=str(ROOT/'assets/arena_complex'));a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    # The remote shell may export an HTTP proxy.  wait_sim talks to a local
    # Isaac HTTP endpoint and must bypass that proxy in the launcher process
    # itself, not only in the child simulator environment.
    os.environ['NO_PROXY']='127.0.0.1,localhost';os.environ['no_proxy']='127.0.0.1,localhost'
    ros=Path(os.environ.get('ROS_ENV', str(ROOT/'ros_env')));site=f'{ros}/lib/python312/site-packages:{ros}/lib/python3.12/site-packages'
    env={**os.environ,'FR3_ARENA_TARGET':a.target,'CALIBRATION_SCENE_SEED':str(a.seed),'UNSEEN_PROTOCOL':str(Path(a.protocol).resolve()),'FR3_ASSET_DIRECTORY':str(Path(a.asset_directory).resolve()),
         'UNSEEN_NO_CLUTTER':'1','CALIBRATION_MU':str(a.calibration_mu),
         'CALIBRATION_FORCE_N':str(a.calibration_force),
         'PAYLOAD_MASS_COM_ONLY':'1' if a.mass_com_only else '0',
         'CALIBRATION_PORT':str(a.port),'ROS_DOMAIN_ID':str(a.domain),'ROS_LOCALHOST_ONLY':'1','UNSEEN_GPU':str(a.gpu),'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y',
         'NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost','AMENT_PREFIX_PATH':str(ros),'PATH':f'{ros}/bin:'+os.environ['PATH'],'PYTHONPATH':f'{ROOT}:{ROOT}/src:{ROOT}/calibration:{site}','OPENBLAS_NUM_THREADS':'1'}
    if a.held_regrasp:
        # Close-range fixed camera for the held-object display.  The values are
        # an acquisition choice only; the frozen grasp/execution parameters are
        # untouched.  sim_real2sim.py consumes these strings at startup.
        env['REAL2SIM_SCAN_CAMERA_POSITION']='.78,-.38,.38'
        env['REAL2SIM_SCAN_LOOK_AT']='.50,0.,.32'
    env.pop('CUDA_VISIBLE_DEVICES',None)
    moveit=sim=None
    with (a.output/'moveit.log').open('w') as ml,(a.output/'sim.log').open('w') as sl,(a.output/'pipeline.log').open('w') as pl:
        try:
            moveit=subprocess.Popen([str(ros/'bin/ros2'),'launch',str(ROOT/'src/moveit.launch.py')],cwd=ROOT,env=env,stdout=ml,stderr=subprocess.STDOUT,start_new_session=True);time.sleep(8)
            sim=subprocess.Popen([ISAAC,'-u','calibration/sim_real2sim.py','--gpu',str(a.gpu),'--port',str(a.port)],cwd=ROOT,env=env,stdout=sl,stderr=subprocess.STDOUT,start_new_session=True);wait_sim(a.port,sim)
            cmd=[str(ros/'bin/python'),'-u','calibration/run_real2sim_pipeline.py','--seed',str(a.seed),'--mode',a.mode,'--output',str(a.output)]
            if a.gt_path:cmd+=['--gt-path',a.gt_path]
            cmd+=['--scan-pass',str(a.scan_pass)]
            cmd+=['--scan-profile',a.scan_profile]
            if a.held_regrasp:cmd+=['--held-regrasp','--held-keyframes',str(a.held_keyframes),'--held-frames-per-pose',str(a.held_frames_per_pose)]
            if a.capture_payload:cmd.append('--capture-payload')
            if a.baseline_only:cmd.append('--baseline-only')
            if a.payload_v2:cmd.append('--payload-v2')
            if a.mass_com_only:cmd.append('--mass-com-only')
            if a.dynamic_only:cmd.append('--dynamic-only')
            if a.gripper_opening_mm is not None:cmd += ['--gripper-opening-mm',str(a.gripper_opening_mm)]
            if a.center_q:cmd+=['--center-q',a.center_q]
            if a.skip_scan:cmd.append('--skip-scan')
            if a.fixed_replay_npz:cmd+=['--fixed-replay-npz',a.fixed_replay_npz]
            # A 900-frame held-object pass plus the physical lower/release/
            # regrasp and second pass is intentionally a long acquisition.  The
            # old one-hour wrapper timeout could terminate a safe scan midway
            # through pass two (the v7 audit stopped at 1105/1800 frames).
            # Extend only this scan orchestration timeout; grasp selection and
            # execution parameters remain untouched.
            run_timeout_s = 4 * 3600 if a.held_regrasp else 3600
            subprocess.run(cmd,cwd=ROOT,env=env,stdout=pl,stderr=subprocess.STDOUT,check=True,timeout=run_timeout_s)
        finally:stop(sim);stop(moveit)
if __name__=='__main__':main()
