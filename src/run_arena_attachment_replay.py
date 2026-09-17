"""Isolate post-micro attachment refresh on the two declared carried collisions."""
import os,sys,time,signal,subprocess,json,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];os.chdir(ROOT)
import plant
parser=argparse.ArgumentParser();parser.add_argument('--segment-m',type=float,default=0.);args=parser.parse_args()
out=ROOT/('results/arena_repair_diagnostics/attachment_segmented' if args.segment_m else 'results/arena_repair_diagnostics/attachment_replay');out.mkdir(parents=True,exist_ok=True)
for name,seed,default in [('sugar',1025,1025),('mug',1039,1035)]:
    if (out/f'B_seed_{seed:04d}.json').exists():continue
    env={**os.environ,'FR3_ARENA_TARGET':name,'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y','ROS_DOMAIN_ID':'83','ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost'}
    with open(out/f'sim_{name}.log','w') as log:
        sim=subprocess.Popen(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','src/sim_arena.py','--clutter'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+300
            while time.monotonic()<deadline:
                if sim.poll() is not None:raise RuntimeError('simulator exited')
                try:
                    if plant.state().get('clutter',{}).get('seed')==default:break
                except Exception:pass
                time.sleep(1)
            else:raise TimeoutError('simulator startup')
            subprocess.run([sys.executable,'-u','src/arena_repair_backend.py','--output',str(out),'--seeds',str(seed),'--replay','--dynamic-attachment','--legacy-acm-diagnostic','--lift-segment-m',str(args.segment_m)],env=env,check=True)
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGTERM)
                try:sim.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
(out/'COMPLETE.json').write_text(json.dumps({'episodes':2,'same_original_selected_pose':True,'acm':'legacy for isolated attachment ablation'}))
