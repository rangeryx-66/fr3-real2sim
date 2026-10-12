"""One registered offline study, four worker processes, equal per-mode ceilings."""
from pathlib import Path
import json,os,subprocess,time,hashlib
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/cross_robot_20261012';PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
config=json.loads((O/'ROBOT_MODELS.json').read_text());old=json.loads((ROOT/'results/door_transfer4_20261012/BATCH_SOURCE_FREEZE.json').read_text());paths=list(old['hashes'])+[str(ROOT/'scripts/cross_robot_adapter.py'),str(ROOT/'scripts/cross_robot_offline.py')]+[c['urdf'] for c in config.values()];frozen={p:sha(p) for p in paths};(O/'STUDY_SOURCE_FREEZE.json').write_text(json.dumps({'hashes':frozen,'reference':'81ab4f5','source_provider':'official pinned models; fixed original PiPER hand','equal_station_and_own_plan_ceiling_s':300,'joint_fixed_values_rad':[-.6,0,.6],'extra_joint_fixed':'matched full7 initial joint3 value','no_physical_attempts_in_offline_study':True},indent=2))
with (O/'OFFLINE_BATCH_RESERVED.json').open('x') as f:json.dump({'robots':list(config),'assets':['45134','45443','45176','45415'],'cases':16,'no_replacement':True},f)
queue=[(robot,asset) for asset in ['45134','45443','45176','45415'] for robot in config];env=dict(os.environ,PYTHONPATH=str(ROOT)+':'+str(ROOT/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1');active={};done=[]
while queue or active:
 while queue and len(active)<4:
  r,a=queue.pop(0);log=(O/(r+'_'+a+'.log')).open('w');p=subprocess.Popen([PY,'-u',str(ROOT/'scripts/cross_robot_offline.py'),'--robot',r,'--asset',a],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT);active[(r,a)]=(p,log,time.time());print('OFFLINE_START',r,a,p.pid,flush=True)
 for (r,a),(p,log,start) in list(active.items()):
  if p.poll() is not None:log.close();done.append({'robot':r,'asset':a,'pid':p.pid,'returncode':p.returncode,'wall_s':time.time()-start});del active[(r,a)];print('OFFLINE_END',r,a,p.returncode,flush=True)
 (O/'OFFLINE_PROGRESS.json').write_text(json.dumps({'active':[{'robot':r,'asset':a,'pid':v[0].pid} for (r,a),v in active.items()],'completed':done},indent=2));time.sleep(1)
changes=[p for p,h in frozen.items() if sha(p)!=h];result={'cases':done,'source_changes':changes,'new_physical_episodes':0,'no_retry':True};(O/'OFFLINE_BATCH_COMPLETE.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
