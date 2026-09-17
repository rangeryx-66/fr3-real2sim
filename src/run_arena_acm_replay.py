"""Six predeclared same-pose ACM diagnostics; no candidate reselection."""
import os, sys, time, signal, subprocess, json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
os.chdir(ROOT)
import plant
out=ROOT/'results/arena_repair_diagnostics/acm_replay'
out.mkdir(parents=True,exist_ok=True)
for name,seeds in [('raisin',[1006]),('hidden_tuna',list(range(1010,1015)))]:
    if all((out/f'B_seed_{seed:04d}.json').exists() for seed in seeds):continue
    env={**os.environ,'FR3_ARENA_TARGET':name,'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y','ROS_DOMAIN_ID':'83','ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost'}
    with open(out/f'sim_{name}.log','w') as log:
        sim=subprocess.Popen(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','src/sim_arena.py','--clutter'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+300
            while time.monotonic()<deadline:
                if sim.poll() is not None:raise RuntimeError('simulator exited')
                try:
                    if plant.state().get('clutter',{}).get('seed') in ([1005] if name=='raisin' else [1010]):break
                except Exception:pass
                time.sleep(1)
            else:raise TimeoutError('simulator startup')
            subprocess.run([sys.executable,'-u','src/arena_repair_backend.py','--output',str(out),'--seeds',','.join(map(str,seeds)),'--replay'],env=env,check=True)
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGTERM)
                try:sim.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
(out/'COMPLETE.json').write_text(json.dumps({'episodes':6,'same_original_selected_pose':True}))
