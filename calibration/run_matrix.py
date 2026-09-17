"""One isolated simulator per asset and friction block; no hot material changes."""
import argparse,os,subprocess,time,urllib.request,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--target',required=True);p.add_argument('--gpu',type=int,required=True);p.add_argument('--port',type=int,required=True);a=p.parse_args()
PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'
base=ROOT/'results/hand_calibration';out=base/'formal'/a.target;out.mkdir(parents=True,exist_ok=True)
seal=json.loads((base/'controller_frozen.json').read_text())
for path,h in seal.items():assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest()==h,path
for mu in [.7,.3,1.,.5]:
    if (out/f'{a.target}_mu{mu}_COMPLETE.json').exists():continue
    env=os.environ.copy();env.update(FR3_ARENA_TARGET=a.target,CALIBRATION_MU=str(mu),CALIBRATION_PORT=str(a.port),OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y',OPENBLAS_NUM_THREADS='1')
    simlog=(base/f'cold_{a.target}_mu{mu}.log').open('w')
    sim=subprocess.Popen([PY,'-u','calibration/sim_calibration.py','--gpu',str(a.gpu),'--port',str(a.port)],cwd=ROOT,env=env,stdout=simlog,stderr=subprocess.STDOUT)
    print('START',a.target,mu,sim.pid,flush=True)
    try:
        deadline=time.monotonic()+600
        while True:
            if sim.poll() is not None:raise RuntimeError(f'Simulator exited {sim.returncode}')
            try:
                urllib.request.urlopen(f'http://127.0.0.1:{a.port}/calibration',timeout=2).read();break
            except Exception:
                if time.monotonic()>deadline:raise TimeoutError('Simulator startup')
                time.sleep(2)
        print('READY',a.target,mu,flush=True)
        with (base/f'formal_{a.target}.log').open('a') as f:
            result=subprocess.run([PY,'-u','calibration/run_trials.py','--target',a.target,'--mu',str(mu),'--output',str(out)],cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
        if result.returncode:raise RuntimeError(f'Trial runner stopped, see formal_{a.target}.log')
        print('BLOCK_COMPLETE',a.target,mu,flush=True)
    finally:
        sim.terminate()
        try:sim.wait(timeout=30)
        except subprocess.TimeoutExpired:sim.kill();sim.wait()
        simlog.close()
    time.sleep(3)
(base/f'{a.target}_ALL_COMPLETE.json').write_text(json.dumps(dict(target=a.target,n=63)))
print('ALL_COMPLETE',a.target,flush=True)
