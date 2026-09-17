"""Run one target's five frozen pose seeds in an isolated ROS domain."""
import argparse,hashlib,json,os,signal,subprocess,time,urllib.request
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'
PROTOCOL=json.loads((ROOT/'SETTLING_REGRASP_PROTOCOL.json').read_text())

def wait_http(port,process,timeout=300):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if process.poll() is not None:raise RuntimeError(f'simulator exited {process.returncode}')
        try:urllib.request.urlopen(f'http://127.0.0.1:{port}/calibration',timeout=2).read();return
        except Exception:time.sleep(1)
    raise TimeoutError('simulator startup')

def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()

def main():
    p=argparse.ArgumentParser();p.add_argument('--target',required=True);p.add_argument('--gpu',type=int,required=True);p.add_argument('--port',type=int,required=True);p.add_argument('--domain',type=int,required=True);p.add_argument('--output',required=True);p.add_argument('--thresholds',required=True);a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    files=[ROOT/'calibration/stability_backend.py',ROOT/'calibration/settling_gate.py',ROOT/'src/grasp_refinement.py',ROOT/'src/mesh_hand_geometry.py',ROOT/'SETTLING_REGRASP_PROTOCOL.json',Path(a.thresholds)]
    hashes={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in files}
    (out/f'seal_{a.target}.json').write_text(json.dumps(hashes,indent=2))
    ros=str(ROOT/'ros_env');site=f'{ros}/lib/python312/site-packages:{ros}/lib/python3.12/site-packages'
    base={**os.environ,'FR3_ARENA_TARGET':a.target,'CALIBRATION_MU':'.7','CALIBRATION_PORT':str(a.port),'ROS_DOMAIN_ID':str(a.domain),'ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost','OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y','OPENBLAS_NUM_THREADS':'1','AMENT_PREFIX_PATH':ros,'PATH':f'{ros}/bin:'+os.environ['PATH'],'PYTHONPATH':site}
    moveit_log=(out/f'moveit_{a.target}.log').open('w')
    moveit=subprocess.Popen([str(ROOT/'ros_env/bin/ros2'),'launch',str(ROOT/'src/moveit.launch.py')],cwd=ROOT,env=base,stdout=moveit_log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        time.sleep(8)
        for seed in PROTOCOL['formal_scene_seeds'][a.target]:
            complete=True
            for repeat in range(3):
                for mode in 'AB':
                    path=out/f'{a.target}_seed{seed}_r{repeat}_{mode}.json'
                    complete &= path.exists() and json.loads(path.read_text()).get('category')!='SYSTEM_ERROR'
            if complete:continue
            env={**base,'CALIBRATION_SCENE_SEED':str(seed),'CUDA_VISIBLE_DEVICES':str(a.gpu)}
            log=(out/f'sim_{a.target}_{seed}.log').open('w')
            sim=subprocess.Popen([PY,'-u','calibration/sim_calibration.py','--gpu','0','--port',str(a.port)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                wait_http(a.port,sim);time.sleep(2)
                cmd=[str(ROOT/'ros_env/bin/python'),'-u','calibration/stability_backend.py','--target',a.target,'--scene-seed',str(seed),'--repeats','3','--output',str(out),'--thresholds',str(Path(a.thresholds).resolve())]
                with (out/f'run_{a.target}_{seed}.log').open('a') as runlog:subprocess.run(cmd,cwd=ROOT,env={**env,'PYTHONPATH':f'src:calibration:{site}'},stdout=runlog,stderr=subprocess.STDOUT,check=True)
            finally:stop(sim);log.close()
            for path,digest in hashes.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest,path
            print('SEED_COMPLETE',a.target,seed,flush=True)
    finally:stop(moveit);moveit_log.close()
    print('TARGET_COMPLETE',a.target,flush=True)

if __name__=='__main__':main()
