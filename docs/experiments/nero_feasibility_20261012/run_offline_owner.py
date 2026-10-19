from pathlib import Path
import os,subprocess,json,time,hashlib
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/nero_feasibility_20261012';PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python';env=dict(os.environ,PYTHONPATH=str(R)+':'+str(R/'scripts'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1');jobs=[]
for a in ['45134','45443','45176','45415']:
 log=(O/('plan_'+a+'.log')).open('w');p=subprocess.Popen([PY,'-u',str(R/'scripts/plan_nero_doors.py'),'--asset',a],cwd=R,env=env,stdout=log,stderr=subprocess.STDOUT);jobs.append((a,p,log));print(a,p.pid,flush=True)
while any(p.poll() is None for a,p,l in jobs):
 (O/'PLANNING_PROGRESS.json').write_text(json.dumps({a:{'pid':p.pid,'exit':p.poll()} for a,p,l in jobs},indent=2));time.sleep(1)
for a,p,l in jobs:l.close()
(O/'PLANNING_COMPLETE.json').write_text(json.dumps({a:p.returncode for a,p,l in jobs},indent=2))
