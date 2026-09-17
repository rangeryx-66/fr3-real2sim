"""Single original success control before freezing the combined implementation."""
import os,sys,time,signal,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];os.chdir(ROOT)
import plant
out=ROOT/'results/arena_repair_diagnostics/combined_preflight';out.mkdir(parents=True,exist_ok=True)
env={**os.environ,'FR3_ARENA_TARGET':'mustard','FR3_ARENA_PROTOCOL':str(ROOT/'ARENA_REPAIR_PROTOCOL.json'),'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y','ROS_DOMAIN_ID':'83','ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost'}
with open(out/'sim.log','w') as log:
    sim=subprocess.Popen(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','src/sim_arena_repair.py','--clutter'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+300
        while time.monotonic()<deadline:
            if sim.poll() is not None:raise RuntimeError('simulator exited')
            try:
                if plant.state().get('clutter',{}).get('seed')==1000:break
            except Exception:pass
            time.sleep(1)
        else:raise TimeoutError('startup')
        subprocess.run([sys.executable,'-u','src/arena_repair_backend.py','--output',str(out),'--seeds','1000','--mesh-gate','--dynamic-attachment','--lift-segment-m','.01'],env=env,check=True)
    finally:
        if sim.poll() is None:
            os.killpg(sim.pid,signal.SIGTERM)
            try:sim.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
