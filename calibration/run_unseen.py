"""Cold-start each paired episode. Verify the sealed code before and after each."""
import argparse,hashlib,json,os,signal,subprocess,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'

def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()

def wait_http(port,process):
    for _ in range(300):
        if process.poll() is not None:raise RuntimeError(f'simulator exited {process.returncode}')
        try:
            data=json.loads(urllib.request.urlopen(f'http://127.0.0.1:{port}/calibration',timeout=2).read())
            if 'calibration' in data:return
        except Exception:pass
        time.sleep(1)
    raise TimeoutError('simulator startup')

def verify(seal):
    for relative,digest in seal.items():
        path=ROOT/relative
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise RuntimeError('FROZEN_FILE_CHANGED: '+str(path))

def main():
    p=argparse.ArgumentParser();p.add_argument('--gpu',type=int,default=3);p.add_argument('--port',type=int,default=18903);p.add_argument('--domain',type=int,default=93);p.add_argument('--output',required=True)
    p.add_argument('--target');p.add_argument('--preflight',action='store_true');p.add_argument('--unsealed-development',action='store_true');p.add_argument('--no-clutter',action='store_true');p.add_argument('--family',action='store_true');p.add_argument('--paired-family',action='store_true');p.add_argument('--protocol');p.add_argument('--asset-directory');p.add_argument('--seal');p.add_argument('--seed',type=int);p.add_argument('--mode',choices=['GT','ANYGRASP']);p.add_argument('--gt-path');p.add_argument('--partition',type=int,default=0);p.add_argument('--partitions',type=int,default=1);a=p.parse_args()
    out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=True)
    proto=Path(a.protocol).resolve() if a.protocol else ROOT/('ARENA_COMPLEX_PROTOCOL.json' if a.preflight else 'UNSEEN_PROTOCOL.json')
    protocol=json.loads(proto.read_text())
    if a.preflight:
        protocol['episodes']=[{**e,'objects':e['objects'][:4]} for e in protocol['episodes']]
        proto=out/'development_protocol.json';proto.write_text(json.dumps(protocol,indent=2))
    episodes=[e for e in protocol['episodes'] if (a.target is None or e['target']==a.target) and (a.seed is None or e['seed']==a.seed)]
    episodes=[e for i,e in enumerate(episodes) if i%a.partitions==a.partition]
    seal_path=Path(a.seal).resolve() if a.seal else ROOT/'UNSEEN_SEAL.json'
    seal={} if (a.preflight or a.unsealed_development) else json.loads(seal_path.read_text())
    verify(seal)
    ros=str(ROOT/'ros_env');site=f'{ros}/lib/python312/site-packages:{ros}/lib/python3.12/site-packages'
    base={**os.environ,'CALIBRATION_MU':'.7','CALIBRATION_PORT':str(a.port),'ROS_DOMAIN_ID':str(a.domain),'ROS_LOCALHOST_ONLY':'1','UNSEEN_GPU':str(a.gpu),
          'NO_PROXY':'127.0.0.1,localhost','no_proxy':'127.0.0.1,localhost','OMNI_KIT_ACCEPT_EULA':'YES','ACCEPT_EULA':'Y','OPENBLAS_NUM_THREADS':'1',
          'AMENT_PREFIX_PATH':ros,'PATH':f'{ros}/bin:'+os.environ['PATH'],'PYTHONPATH':site,'UNSEEN_PROTOCOL':str(proto),
          'FR3_ASSET_DIRECTORY':str(Path(a.asset_directory).resolve()) if a.asset_directory else str(ROOT/'assets'/('arena_complex' if a.preflight else 'unseen_v1'))}
    base.pop('CUDA_VISIBLE_DEVICES',None)
    if a.no_clutter:base['UNSEEN_NO_CLUTTER']='1'
    else:base.pop('UNSEEN_NO_CLUTTER',None)
    (out/f'worker_{a.partition}_protocol.json').write_text(json.dumps(dict(arguments=vars(a),code_seal=seal,episodes=episodes),indent=2))
    with (out/f'moveit_{a.partition}.log').open('w') as moveit_log:
        moveit=subprocess.Popen([str(ROOT/'ros_env/bin/ros2'),'launch',str(ROOT/'src/moveit.launch.py')],cwd=ROOT,env=base,stdout=moveit_log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            time.sleep(8)
            for episode in episodes:
                target=episode['target'];seed=episode['seed']
                if a.paired_family:
                    variants=[('ANYGRASP',False,'baseline'),('ANYGRASP',True,'family')]
                    if episode.get('repeat',0)%2:variants.reverse()
                else:
                    order=[a.mode] if a.mode else episode.get('order',['GT','ANYGRASP'])
                    variants=[(mode,a.family,None) for mode in order]
                shared_anygrasp=None;shared_reference=None
                for mode,use_family,variant in variants:
                    verify(seal);label=f'{target}_seed{seed}_{mode}'
                    run_out=out/variant if variant else out;run_out.mkdir(parents=True,exist_ok=True)
                    path=run_out/(label+'.json')
                    if path.exists():
                        old=json.loads(path.read_text())
                        if old.get('category')=='SYSTEM_ERROR':raise RuntimeError('Previous failed run requires batch audit: '+label)
                        if a.paired_family and shared_anygrasp is None:
                            candidate=run_out/(label+'_anygrasp.json')
                            if candidate.exists():shared_anygrasp=candidate;shared_reference=path
                            elif old.get('anygrasp_replay'):
                                shared_anygrasp=Path(old['anygrasp_replay']['source_json']);shared_reference=Path(old['anygrasp_replay']['source_result'])
                        continue
                    env={**base,'FR3_ARENA_TARGET':target,'CALIBRATION_SCENE_SEED':str(seed),
                         'UNSEEN_SCENE_SNAPSHOT':str(run_out/(label+'_scene.usdc'))}
                    with (run_out/(label+'.sim.log')).open('w') as log:
                        sim=subprocess.Popen([PY,'-u','calibration/sim_unseen.py','--gpu',str(a.gpu),'--port',str(a.port)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                        try:
                            wait_http(a.port,sim)
                            cmd=[str(ROOT/'ros_env/bin/python'),'-u','calibration/unseen_backend.py','--seed',str(seed),'--mode',mode,'--output',str(run_out)]
                            if a.gt_path:cmd.extend(['--gt-path',a.gt_path])
                            if use_family:cmd.append('--family')
                            if a.paired_family and shared_anygrasp is not None:
                                cmd.extend(['--replay-anygrasp',str(shared_anygrasp),'--replay-reference',str(shared_reference)])
                            with (run_out/(label+'.run.log')).open('w') as runlog:
                                subprocess.run(cmd,cwd=ROOT,env={**env,'PYTHONPATH':f'src:calibration:{site}'},stdout=runlog,stderr=subprocess.STDOUT,check=True,timeout=3600)
                            if a.paired_family and shared_anygrasp is None:
                                shared_anygrasp=run_out/(label+'_anygrasp.json');shared_reference=path
                        finally:stop(sim)
                    verify(seal);print('EPISODE_COMPLETE',label,flush=True)
        finally:stop(moveit)
    print('WORKER_COMPLETE',a.partition,flush=True)
if __name__=='__main__':main()
