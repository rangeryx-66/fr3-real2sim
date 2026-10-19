"""One-command bounded conditional dynamics and independent full-task validation."""
import sys,json,hashlib,copy,argparse,os,subprocess,time,signal,queue,concurrent.futures
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
import numpy as np
from interactive_twin_conditional.candidates import build
from interactive_twin.twin import write_twins
from interactive_twin.visual_import import compatible_urdf

def read(p):return json.loads(Path(p).read_text())
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2,allow_nan=False))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def absolute(p):return Path(p) if Path(p).is_absolute() else ROOT/p

class Experiment:
 def __init__(self,config):
  self.c=config;self.out=absolute(config['output']);self.old=absolute(config['refinement_output']);self.out.mkdir(parents=True,exist_ok=True)
  p=self.out/'frozen_config.json'
  if p.exists() and read(p)!=config:raise ValueError('FROZEN_CONFIG_CHANGED')
  if not p.exists():write(p,config)
  self.pool=queue.Queue()
  for g in config['gpus']:self.pool.put(g)
 def prepare(self,eid='dev_7320_00'):
  out=self.out/eid;old=self.old/eid;ref=old/'reference'
  original=read(ref/'job_private.json');scene=read(ref/'initial_scene_private.json');W=np.eye(4);W[:3,:3]=scene['asset_rotation'];W[:3,3]=scene['asset_xyz']
  snap=out/'stable_grasp_snapshot.json'
  if not snap.exists():
   rows=read(ref/'observations.json');commands=read(ref/'command_tape.json');i=next(i for i,r in enumerate(rows) if r['phase']=='COMPLIANT_SETTLE')-1
   r=rows[i];cmd=commands[i];prev=rows[i-1]
   write(snap,{'q_rad':r['q'],'qdot_rad_s':((np.asarray(r['q'])-prev['q'])*240).tolist(),'T_ee':r['T_tcp'],'arm_position_command':cmd['arm_position'],'arm_velocity_command':cmd['arm_velocity'],'finger_position_command':cmd['finger_position'],'finger_effort_n':cmd['finger_effort'],'source_time_s':r['t'],'source_observations_sha256':sha(ref/'observations.json'),'provenance':{'joint_truth_used':False,'moving_trajectory_used':False,'part_pose_source':'initial prepared visual assembly; stable pre-probe grasp, no axis/angle observation','initial_part_pose_estimated_unchanged':True,'source_phase':r['phase'],'contact_forces_not_restored':True}})
  space=out/'structure_candidates.json'
  if not space.exists():write(space,build(read(old/'structure_selection.json'),read(old/'refinement_once/measured_actions.json'),self.c['structure_policy']))
  structures=[]
  support=read(old/'structured_memory.json')['supporting_observations']
  for r in read(space)['candidates']:
   if not r['accepted']:continue
   sid=r['structure_id'];dest=out/'models'/sid
   if not (dest/'twin_versions.json').exists():write_twins(original['asset_root'],dest,r['estimate'],W,initial_physics_prior=self.c['wrong_prior'],ee_poses=support)
   asset=Path(read(dest/'twin_versions.json')['versions']['T1']['asset_root']);m=read(asset/'manifest.json');compatible_urdf(asset/'urdf'/f"{m['asset_id']}.urdf",asset/'visual_compatibility')
   structures.append({'id':sid,'asset':str(asset),'estimate':str(dest/'estimated_articulation.json')})
  # Source estimate remains the previously safe controller memory for every plant.
  job={**original,'output':str(out/'reference/original'),'conditional_snapshot':str(snap),'mode':'physics_reference','role':'reference','physics_protocol':self.c['physics_protocol'],'continue_manipulation':False,'deadline_shanghai':self.c['deadline_shanghai'],'wall_clock_budget_s':self.c['native_wall_clock_budget_s']}
  job.pop('replay_commands',None)
  data={'source_job':job,'structures':structures,'source_hashes':{str(old/'structure_selection.json'):sha(old/'structure_selection.json'),str(snap):sha(snap)},'physics_candidate_budget':len(structures)*9,'source_commit':self.c['base_commit']}
  prepared=out/'prepared.json'
  if prepared.exists() and read(prepared)['source_hashes']!=data['source_hashes']:raise RuntimeError('FROZEN_SOURCE_CHANGED')
  write(prepared,data)
  return data
 def native(self,job):
  out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
  if (out/'report.json').exists():
   previous=read(out/'job_private.json')
   if {k:v for k,v in previous.items() if k!='gpu'}!={k:v for k,v in job.items() if k!='gpu'}:raise RuntimeError('CACHED_JOB_CHANGED:'+str(out))
   return read(out/'report.json')
  gpu=self.pool.get();job=dict(job,gpu=gpu)
  try:
   if time.time()>=datetime.fromisoformat(job['deadline_shanghai']).timestamp():raise RuntimeError('DEADLINE')
   write(out/'job_private.json',job);write(out/'code_identity.json',{'entry_sha256':sha(ROOT/'scripts/run_interactive_twin_conditional_episode.py'),'config_sha256':sha(self.out/'frozen_config.json')})
   env=dict(os.environ)
   for k in ('PYTHONPATH','CUDA_VISIBLE_DEVICES','http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','ALL_PROXY','all_proxy'):env.pop(k,None)
   env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
   cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'scripts/run_interactive_twin_conditional_episode.py'),'--job',str(out/'job_private.json')]
   start=time.time()
   with (out/'process.log').open('w') as f:
    p=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);write(out/'process.json',{'pid':p.pid,'start':start,'command':cmd})
    try:p.wait(timeout=min(job['wall_clock_budget_s'],datetime.fromisoformat(job['deadline_shanghai']).timestamp()-start))
    except subprocess.TimeoutExpired:
     os.killpg(p.pid,signal.SIGINT)
     try:p.wait(timeout=30)
     except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
   write(out/'process.json',{'pid':p.pid,'start':start,'end':time.time(),'exit_code':p.returncode})
   if not (out/'report.json').exists():write(out/'report.json',{'status':'NATIVE_PROCESS_FAILED','exit_code':p.returncode})
   result=read(out/'report.json');print(out.relative_to(self.out),result['status'],flush=True);return result
  finally:self.pool.put(gpu)
 def run(self,stage,eid='dev_7320_00'):
  data=self.prepare(eid)
  if stage=='prepare':return
  if stage=='initialize':return self.native(data['source_job'])
  from interactive_twin_conditional.workflow import full
  return full(self,eid,data)

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',default='configs/interactive_twin_conditional.yaml');p.add_argument('--stage',choices=['prepare','initialize','full','summarize'],default='full');p.add_argument('--episode',default='dev_7320_00');a=p.parse_args()
 e=Experiment(read(absolute(a.config)))
 from interactive_twin_conditional.reporting import finalize
 if a.stage=='summarize':finalize(e)
 else:
  try:e.run(a.stage,a.episode)
  finally:
   if a.stage=='full':finalize(e)
if __name__=='__main__':main()
