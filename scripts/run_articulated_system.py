"""Resumable collection/backend components; one failure never cancels the other."""
import argparse,json,os,sys,time,subprocess,signal,traceback,hashlib
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def main():
 p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/articulated_system.json');p.add_argument('--stage',choices=['full','collect','prepare','reconstruct','twin','preview','validate','report'],default='full');p.add_argument('--output',type=Path,default=ROOT/'results/articulated_system_20261005');p.add_argument('--object',choices=['7320','45746']);p.add_argument('--backend-python',type=Path,default=ROOT/'environments/artgs/bin/python');p.add_argument('--isaac-python',type=Path,default=Path('/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'));p.add_argument('--pair',choices=['small','large'],help='Run one state pair without duplicating an incomplete capture');p.add_argument('--capture-root',type=Path,help='Use an existing durable extension capture, without rerunning robot');p.add_argument('--interaction-type-prior',action='store_true',help='Official ArtGS type-only option from measured EE identification');p.add_argument('--gpu',type=int,default=1);a=p.parse_args()
 c=json.loads(a.config.read_text());out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);frozen=out/'frozen_system_config.json'
 if frozen.exists() and json.loads(frozen.read_text())!=c:raise RuntimeError('RESUME_CONFIGURATION_CHANGED')
 frozen.write_text(json.dumps(c,indent=2));ledgerpath=out/'component_status.json';ledger=json.loads(ledgerpath.read_text()) if ledgerpath.exists() else {'started_wall_s':time.time(),'basis':c['basis_commit'],'components':{}}
 deadline=min(datetime.fromisoformat(c['deadline_shanghai']).timestamp(),ledger['started_wall_s']+c['total_wall_budget_s'])
 def save():ledgerpath.write_text(json.dumps(ledger,indent=2))
 def run(key,command,budget,env=None):
  row=ledger['components'].get(key,{})
  if row.get('status')=='COMPLETE':return True
  remain=min(budget,deadline-time.time())
  if remain<=0:ledger['components'][key]={'status':'CUTOFF','resume_command':command};save();return False
  log=out/(key.replace('/','_')+'.log');row={'status':'RUNNING','command':command,'log':str(log),'started_wall_s':time.time(),'budget_s':remain};ledger['components'][key]=row;save()
  e=dict(os.environ,PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1');e.pop('PYTHONPATH',None);e.update(env or {})
  with log.open('a') as stream:
   child=subprocess.Popen(command,cwd=ROOT,env=e,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
   try:code=child.wait(timeout=remain)
   except subprocess.TimeoutExpired:
    os.killpg(child.pid,signal.SIGINT)
    try:child.wait(timeout=30)
    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    code=-1
   except BaseException:
    # A parent deadline also reaches this nested runner; its stage child has
    # its own process group and must be explicitly stopped, never orphaned.
    if child.poll() is None:
     os.killpg(child.pid,signal.SIGINT)
     try:child.wait(timeout=30)
     except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    row.update(status='INTERRUPTED',returncode=child.returncode,ended_wall_s=time.time());save();raise
  row.update(status='COMPLETE' if code==0 else 'FAILED' if code!=-1 else 'BUDGET_EXHAUSTED',returncode=code,ended_wall_s=time.time());save();return code==0
 objects=[o for o in c['objects'] if not a.object or o['id']==a.object]
 # Existing observations are durable inputs: run backend first, independently
 # of extension collection or safe-release recovery.
 for o in objects:
  root=a.capture_root.resolve() if a.capture_root else ROOT/o['existing_capture']
  suffix='_multiview' if a.capture_root else ''
  doc=json.loads((root/'multistate_capture.json').read_text())
  if doc['object_id']!=o['id']:raise RuntimeError('CAPTURE_ASSET_MISMATCH')
  if a.capture_root and len(doc['states'])<3:raise RuntimeError('INSUFFICIENT_CAPTURE_STATES')
  if a.stage in ['prepare','reconstruct','full']:
   for label,end in [('small',min(o['small_state'],len(doc['states'])-1)),('large',len(doc['states'])-1 if a.capture_root else o['large_state'])]:
    if a.pair and label!=a.pair:continue
    src=out/'artgs/data/capture/sensor'/f"{o['id']}_{label}{suffix}"
    key=f"backend/{o['id']}/{label}{suffix}"+("/interaction_prior" if a.interaction_type_prior else "")
    if (src/'input_provenance.json').exists():
     saved_input=json.loads((src/'input_provenance.json').read_text())
     if Path(saved_input['capture_root']).resolve()!=root.resolve() or saved_input['states']!=[0,end]:raise RuntimeError('FROZEN_INPUT_PAIR_CHANGED: choose a new output directory')
    if not (src/'input_provenance.json').exists():
     command=[str(a.backend_python),'-c','from articulated_system.artgs import prepare;prepare(*__import__("sys").argv[1:3],0,int(__import__("sys").argv[3]))',str(root),str(src),str(end)]
     if not run('prepare/'+o['id']+'/'+label+suffix,command,600):continue
    if a.stage!='prepare':
     existing=Path(str(src).replace('/data/','/outputs/'))/('backend_status_interaction_prior.json' if a.interaction_type_prior else 'backend_status.json')
     if existing.exists():
      r=json.loads(existing.read_text())
      if r.get('state')=='COMPLETE' and r.get('actual_iterations')=={'coarse':c['backend']['coarse_iterations'],'predict':c['backend']['predict_iterations'],'joint':c['backend']['joint_iterations']}:
       ledger['components'][key]={'status':'COMPLETE','reused_actual_output':str(existing),'no_reconstruction_rerun':True};save()
     command=[str(a.backend_python),str(ROOT/'scripts/run_artgs_backend.py'),'--source',str(src),'--coarse-iterations',str(c['backend']['coarse_iterations']),'--predict-iterations',str(c['backend']['predict_iterations']),'--joint-iterations',str(c['backend']['joint_iterations']),'--wall-s',str(c['backend']['wall_s_per_pair'])]
     if a.interaction_type_prior:command.append('--type-from-interaction')
     run(key,command,c['backend']['wall_s_per_pair'],{'CUDA_VISIBLE_DEVICES':str(a.gpu)})
  if a.stage in ['collect','full'] and not a.capture_root and ledger['components'].get('collect/'+o['id'],{}).get('status')!='COMPLETE':
   original=json.loads((ROOT/o['job']).read_text());job=original
   if 'skill' not in job:
    # Closed-state initialization is the original frozen successful job.
    job.update(asset_root=str(ROOT/'results/semantic_interaction/asset'),plan=str(ROOT/'results/semantic_interaction/nominal_minimal/plan.json'),candidate=0)
    job['skill']={'joint_type':'revolute','goal':'open_for_multistate_capture','targets':[0,5,10,15,20],'capture_interval':2.5,'minimum_capture_separation':.5,'hold_s':2.,'segment_m':.001,'segment_timeout_s':5.}
   attempt=out/(o['id']+'_capture_'+datetime.now().strftime('%Y%m%dT%H%M%S'))
   job.update(output=str(attempt),gpu=a.gpu,deadline_shanghai=c['deadline_shanghai'],mobile_platform=True,system_capture=c['capture'],wall_clock_budget_s=c['capture']['wall_s'])
   job['skill'].update(maximum_segments=c['capture']['maximum_segments'],maximum_sim_s=c['capture']['maximum_sim_s'],maximum_path_m=c['capture']['task_path_m'],warning_margin_rad=c['capture']['warning_margin_rad'])
   jp=out/(o['id']+'_capture_job.json');jp.write_text(json.dumps(job,indent=2))
   key='collect/'+o['id']
   ok=run(key,[str(a.isaac_python),str(ROOT/'scripts/run_articulated_system_episode.py'),'--job',str(jp)],c['capture']['wall_s'])
   ledger['components'][key]['capture_directory']=str(attempt)
   if not ok and not (attempt/'report.json').exists():
    attempt.mkdir(parents=True,exist_ok=True)
    (attempt/'infrastructure_failure.json').write_text(json.dumps({
     'status':'COLLECTION_PROCESS_EXIT_WITHOUT_PHYSICAL_REPORT',
     'exit_code':ledger['components'][key].get('returncode'),
     'log':ledger['components'][key].get('log'),
     'physical_safety_failure_confirmed':False,
     'recovery_success_not_inferred_from_saved_images':True,
    },indent=2))
   if (attempt/'report.json').exists():
    ledger['components'][key]['physical_report']=json.loads((attempt/'report.json').read_text())
    if (attempt/'multistate_capture.json').exists() and json.loads((attempt/'multistate_capture.json').read_text())['states']:
     run('physical_evaluation/'+o['id'],[str(a.backend_python),str(ROOT/'scripts/evaluate_articulated_collection.py'),'--capture',str(attempt)],600)
    run('effort_extension/'+o['id'],[str(a.backend_python),'-c','from articulated_system.effort import profile;profile(__import__("sys").argv[1])',str(attempt)],600)
   save()
   if (attempt/'multistate_capture.json').exists() and len(json.loads((attempt/'multistate_capture.json').read_text())['states'])>=3:
    # Real newly acquired views go through the backend in THIS run, separately
    # from pre-existing inputs. Failure of recovery does not discard captures.
    run('extension/'+o['id'],[sys.executable,str(Path(__file__).resolve()),'--config',str(a.config.resolve()),'--output',str(out/'extension'/o['id']),'--object',o['id'],'--stage','reconstruct','--capture-root',str(attempt),'--gpu',str(a.gpu),'--backend-python',str(a.backend_python),'--isaac-python',str(a.isaac_python)],c['backend']['wall_s_per_pair']*2)
  if a.stage in ['twin','preview','reconstruct','full']:
   if (root/'effort_segments.json').exists() and not (root/'effort_profile.json').exists():
    ledger['components'].pop('effort/'+o['id']+suffix,None)
   run('effort/'+o['id']+suffix,[str(a.backend_python),'-c','from articulated_system.effort import profile;profile(__import__("sys").argv[1])',str(root)],600)
   if (root/'report.json').exists() and (root/'evaluation_private/object_trajectory.json').exists():
    run('effort_object_evaluation/'+o['id']+suffix,[str(a.backend_python),'-c','from articulated_system.effort import profile;profile(__import__("sys").argv[1],object_motion_evaluation=True)',str(root)],600)
   for label in ['small','large']:
    if a.pair and label!=a.pair:continue
    src=out/'artgs/data/capture/sensor'/f"{o['id']}_{label}{suffix}";recon=out/'artgs/outputs/capture/sensor'/f"{o['id']}_{label}{suffix}"/('reconstruction_interaction_prior' if a.interaction_type_prior else 'reconstruction');twin=out/'twins'/o['id']/(label+suffix+('_interaction_prior' if a.interaction_type_prior else ''))
    if not (recon/'motion_inferred.json').exists():continue
    if not run('twin/'+o['id']+'/'+label+suffix+('_interaction_prior' if a.interaction_type_prior else ''),[str(a.backend_python),'-c','from articulated_system.twin import write;write(*__import__("sys").argv[1:])',str(recon),str(src),str(root),str(twin)],600):continue
    # A backend can finish while collection is still running. Resume must
    # attach later real observations/effort rather than freeze an unavailable stub.
    attachment_bytes=(root/'multistate_capture.json').read_bytes()
    if (root/'effort_profile.json').exists():attachment_bytes+=(root/'effort_profile.json').read_bytes()
    if (root/'effort_profile_object_evaluation.json').exists():attachment_bytes+=(root/'effort_profile_object_evaluation.json').read_bytes()
    attachment_version=hashlib.sha256(attachment_bytes).hexdigest()[:12]
    run('attach/'+o['id']+'/'+twin.name+'/'+attachment_version,[str(a.backend_python),'-c','from articulated_system.twin import attach_observations;attach_observations(*__import__("sys").argv[1:])',str(root),str(twin)],60)
    if a.stage in ['preview','reconstruct','full']:run('preview/'+o['id']+'/'+label+suffix+('_interaction_prior' if a.interaction_type_prior else ''),[str(a.isaac_python),str(ROOT/'scripts/preview_reconstructed_twin.py'),'--twin',str(twin),'--gpu',str(a.gpu)],600)
  if a.stage in ['validate','reconstruct','full']:
   for label in ['small','large']:
    if a.pair and label!=a.pair:continue
    name=f"{o['id']}_{label}{suffix}";src=out/'artgs/data/capture/sensor'/name
    recon=out/'artgs/outputs/capture/sensor'/name/('reconstruction_interaction_prior' if a.interaction_type_prior else 'reconstruction')
    if not (recon/'motion_inferred.json').exists():continue
    result=out/'value_check'/(name+('_interaction_prior' if a.interaction_type_prior else ''))
    run('validate/'+result.name,[str(a.backend_python),str(ROOT/'scripts/evaluate_artgs_state.py'),'--input',str(src),'--capture',str(root),'--state','1','--model-name','artgs_interaction_prior' if a.interaction_type_prior else 'artgs','--output',str(result)],600,{'CUDA_VISIBLE_DEVICES':str(a.gpu)})
 report(out,objects,ledger,c)


def report(out,objects,ledger,config):
 lines=['# Articulated system integration','',f"Baseline: {config['basis_commit']}. Deadline: {config['deadline_shanghai']}",'','Collection and reconstruction are independent. COMPLETE means that component ran; physical success uses its report, never process exit code.','', '| Component | Status | Log |','|---|---|---|']
 for key,row in ledger['components'].items():lines.append(f"| {key} | {row['status']} | {row.get('log','')} |")
 lines+=['','## Interpretation','ArtGS performs scene-specific optimization, not pretrained general-purpose inference. Official example checkpoints are demonstration-only. Existing two-view captures are retained; new views hold the object at one state by pausing physics during render. Reconstructed-twin joint-driven videos are import checks, not robot manipulation.','Effort is a normal+friction simulation measurement only when the verified buffers are available; otherwise command proxy. No effort profile is silently mapped to URDF friction.','No claims of full completion are made for failed or missing components.']
 (out/'COMPONENT_REPORT.md').write_text('\n'.join(lines)+'\n')
 from report_articulated_system import report as actual_report
 actual_report(out)
if __name__=='__main__':main()
