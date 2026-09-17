"""Frozen original-40 and held-out-32 runs. Never retune or hide failed episodes."""
import os,sys,json,time,subprocess,hashlib,signal
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];os.chdir(ROOT)
import plant
protocol=json.loads((ROOT/'ARENA_REPAIR_PROTOCOL.json').read_text())
config=json.loads((ROOT/'ARENA_REPAIR_CONFIG.json').read_text())
audit_dir=ROOT/'results/arena_repair_frozen';audit_dir.mkdir(parents=True,exist_ok=True)
files=list((ROOT/'src').glob('*.py'))+[ROOT/'ARENA_REPAIR_PROTOCOL.json',ROOT/'ARENA_REPAIR_CONFIG.json',ROOT/'assets/arena_complex/inventory.json']+list((ROOT/'assets/arena_complex').glob('*_mesh.npz'))+list((ROOT/'config').glob('*.urdf'))
hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
seal=audit_dir/'sealed_files.json'
if seal.exists():assert json.loads(seal.read_text())==hashes,'sealed sources changed'
else:seal.write_text(json.dumps(hashes,indent=2))
def verify():
    for name,digest in hashes.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
for cohort,seeds in [('original40',protocol['development_seeds']),('heldout32',protocol['heldout_seeds'])]:
    out=ROOT/f'results/arena_repair_{cohort}';out.mkdir(parents=True,exist_ok=True)
    for name in protocol['classes']:
        selected=[e['seed'] for e in protocol['episodes'] if e['target']==name and e['seed'] in seeds]
        if all((out/f'B_seed_{seed:04d}.json').exists() for seed in selected):continue
        verify()
        default=next(e['seed'] for e in protocol['episodes'] if e['target']==name)
        env={**os.environ,'FR3_ARENA_TARGET':name,'FR3_ARENA_PROTOCOL':str(ROOT/'ARENA_REPAIR_PROTOCOL.json'),'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y','ROS_DOMAIN_ID':'83','ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost'}
        with open(out/f'sim_{name}.log','w') as log:
            sim=subprocess.Popen(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','src/sim_arena_repair.py','--clutter','--gpu',str(config['simulation_gpu'])],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                deadline=time.monotonic()+300
                while time.monotonic()<deadline:
                    if sim.poll() is not None:raise RuntimeError(f'{name}: simulator exited')
                    try:
                        if plant.state().get('clutter',{}).get('seed')==default:break
                    except Exception:pass
                    time.sleep(1)
                else:raise TimeoutError('simulator startup')
                print('FROZEN_CLASS',cohort,name,selected,flush=True)
                subprocess.run([sys.executable,'-u','src/arena_repair_backend.py','--output',str(out),'--seeds',','.join(map(str,selected)),'--mesh-gate','--dynamic-attachment','--lift-segment-m',str(config['lift_segment_m'])],env=env,check=True)
            finally:
                if sim.poll() is None:
                    os.killpg(sim.pid,signal.SIGTERM)
                    try:sim.wait(timeout=20)
                    except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
        verify()
        (out/'progress.json').write_text(json.dumps(dict(last_class=name,episodes=len(list(out.glob('B_seed*.json')))),indent=2))
    results=[json.loads((out/f'B_seed_{seed:04d}.json').read_text()) for seed in seeds]
    assert all(r['category']!='SYSTEM_ERROR' for r in results)
    (out/'COMPLETE.json').write_text(json.dumps(dict(episodes=len(results),successes=sum(r['success'] for r in results),configuration_unchanged=True),indent=2))
