"""Run each sealed scene once, with one simulator process per target asset."""
import os,sys,json,time,subprocess,hashlib,signal
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];os.chdir(ROOT)
protocol=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())
out=ROOT/'results/arena_complex40';out.mkdir(exist_ok=True,parents=True)
files=list((ROOT/'src').glob('*.py'))+[ROOT/'ARENA_COMPLEX_PROTOCOL.json',ROOT/'assets/arena_complex/inventory.json']+list((ROOT/'assets/arena_complex').glob('*_mesh.npz'))
hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
audit=out/'sealed_files.json'
if audit.exists():
    previous=json.loads(audit.read_text());assert all(hashes[k]==v for k,v in previous.items()),'sealed source changed'
else:audit.write_text(json.dumps(hashes,indent=2))
import plant
for name in protocol['classes']:
    seeds=[e['seed'] for e in protocol['episodes'] if e['target']==name]
    if all((out/f'B_seed_{s:04d}.json').exists() for s in seeds):continue
    env={**os.environ,'FR3_ARENA_TARGET':name,'OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y','ROS_DOMAIN_ID':'83','ROS_LOCALHOST_ONLY':'1','NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost'}
    log=open(ROOT/f'logs/arena_sim_{name}.log','w')
    sim=subprocess.Popen(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','src/sim_arena.py','--clutter'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+300
        while time.monotonic()<deadline:
            if sim.poll() is not None:raise RuntimeError(f'{name} simulator exited {sim.returncode}')
            try:
                s=plant.state()
                if s.get('clutter',{}).get('seed')==seeds[0]:break
            except Exception:pass
            time.sleep(1)
        else:raise TimeoutError('sim startup')
        print('FORMAL_CLASS',name,seeds,flush=True)
        subprocess.run([sys.executable,'-u','src/arena_backend.py','--output',str(out),'--seeds',','.join(map(str,seeds))],env=env,check=True)
    finally:
        os.killpg(sim.pid,signal.SIGTERM)
        try:sim.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
        log.close()
    for k,v in hashes.items():assert hashlib.sha256((ROOT/k).read_bytes()).hexdigest()==v,k
    (out/'progress.json').write_text(json.dumps(dict(completed_class=name,episodes=len(list(out.glob('B_seed*.json')))),indent=2))
(out/'COMPLETE.json').write_text(json.dumps(dict(episodes=40,configuration_unchanged=True),indent=2))
