"""Continue both saved command episodes in independent live Isaac processes."""
import argparse,json,time,os,sys,subprocess,signal
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from concurrent.futures import ThreadPoolExecutor
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/wrist_reconstruction_extended.json');p.add_argument('--output',type=Path,required=True);p.add_argument('--source-root',type=Path,required=True);p.add_argument('--object',choices=['7320','45746']);p.add_argument('--checkpoint',type=Path);p.add_argument('--stop-t',type=float);a=p.parse_args()
 out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);c=json.loads(a.config.read_text());clock=out/'run_clock.json'
 d=json.loads(clock.read_text()) if clock.exists() else {'start_wall_s':time.time(),'finish_early':True}
 d.setdefault('capture_deadline_wall_s',d['start_wall_s']+c['capture']['wall_s']);clock.write_text(json.dumps(d,indent=2));deadline=datetime.fromtimestamp(d['capture_deadline_wall_s'],ZoneInfo('Asia/Shanghai')).isoformat()
 sources={'7320':a.source_root/'cartesian_retreat_stop_7320','45746':a.source_root/'load_stop_5mm_45746'}
 if a.object:sources={a.object:sources[a.object]}
 (out/'frozen_config.json').write_text(json.dumps(c,indent=2));records={}
 def run(obj):
  from wrist_reconstruction.actor_owner import claim
  owner=claim(obj,out/('inputs_'+obj)/'continuation_job.json',ROOT)
  from wrist_reconstruction.continuation import prepare,make_job
  src=sources[obj].resolve();prepared=out/('inputs_'+obj);target=out/('capture_'+obj)
  if not (prepared/'issued_robot_commands.json').exists():prepare(src,prepared,stop_t=a.stop_t)
  job=make_job(src,prepared,target,c,deadline,checkpoint_path=a.checkpoint);path=prepared/'continuation_job.json'
  prior=out/'preserved_failed_regrasp_candidates.json'
  if prior.exists():
   job['prior_failed_regrasp_candidates']=json.loads(prior.read_text()).get(obj,[])
   path.write_text(json.dumps(job,indent=2))
  records[obj]={'status':'RUNNING','source':str(src),'actual_prior_progress':'reference only; never applied as object state','job':str(path)}
  (out/(obj+'_component.json')).write_text(json.dumps(records[obj],indent=2))
  env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONNOUSERSITE='1');env.pop('PYTHONPATH',None);env['PATH']='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+env['PATH']
  with (out/(obj+'.log')).open('a') as log:
   proc=subprocess.Popen(['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'scripts/run_wrist_reconstruction_episode.py'),'--job',str(path)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   records[obj]['pid']=proc.pid;(out/(obj+'_component.json')).write_text(json.dumps(records[obj],indent=2))
   try:code=proc.wait(timeout=max(1,d['capture_deadline_wall_s']-time.time()))
   except subprocess.TimeoutExpired:
    os.killpg(proc.pid,signal.SIGINT)
    try:code=proc.wait(timeout=30)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);code=proc.wait()
  owner.close()
  records[obj].update(status='STOPPED',returncode=code);report=target/'report.json'
  if report.exists():records[obj]['physical_status']=json.loads(report.read_text())['status']
  (out/(obj+'_component.json')).write_text(json.dumps(records[obj],indent=2))
 with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(run,sources))
 # Never start a reconstruction job from progress labels alone.
 from wrist_reconstruction.readiness import both_ready,summarize
 ready,audit=both_ready(out,c);(out/'backend_data_gate.json').write_text(json.dumps({'ready':ready,'objects':audit},indent=2))
 (out/'RUN_STATUS.md').write_text('# Continuation processes stopped\n\n'+json.dumps(records,ensure_ascii=False,indent=2)+'\n'+ '\n'.join(summarize(out)))
if __name__=='__main__':main()
