"""Independent bounded OLD/WRIST-CLEAN/ORACLE capture/backend ledger."""
import argparse,copy,json,os,sys,time,subprocess,hashlib,signal,threading,fcntl
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from datetime import datetime,timedelta
from zoneinfo import ZoneInfo
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/wrist_reconstruction_v2.json');p.add_argument('--output',type=Path,default=ROOT/'results/wrist_mobile_20261006/run_v2');p.add_argument('--stage',choices=['capture','audit-old','coarse','backend','full','report'],default='full');p.add_argument('--object',choices=['7320','45746']);p.add_argument('--gpu',type=int,default=0);p.add_argument('--mode',choices=['periodic','maximum-range'],default='periodic');p.add_argument('--resume',action='store_true');p.add_argument('--observation-source',type=Path,action='append',default=[]);a=p.parse_args()
    if a.mode=='maximum-range' and a.config==ROOT/'configs/wrist_reconstruction_v2.json':a.config=ROOT/'configs/wrist_reconstruction_max_range.json'
    c=json.loads(a.config.read_text());out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);frozen=out/'frozen_config.json'
    if frozen.exists() and json.loads(frozen.read_text())!=c:raise RuntimeError('FROZEN_CONFIGURATION_CHANGED')
    frozen.write_text(json.dumps(c,indent=2));ledgerpath=out/'components.json';ledger=json.loads(ledgerpath.read_text()) if ledgerpath.exists() else {'start_wall_s':time.time(),'components':{}}
    maximum_mode=c.get('maximum_range',{}).get('enabled',False)
    if maximum_mode and ledgerpath.exists() and not a.resume:raise RuntimeError('USE_RESUME_FOR_EXISTING_RUN; no automatic physical reset')
    inherited_clock=out/'run_clock.json'
    inherited=json.loads(inherited_clock.read_text()) if inherited_clock.exists() else {}
    absolute_capture_ceiling=inherited.get('capture_deadline_wall_s',datetime.fromisoformat(c['deadline_shanghai']).timestamp() if c.get('deadline_shanghai') else ledger['start_wall_s']+c['capture']['wall_s'])
    capture_cutoff=ledger.setdefault('capture_cutoff_wall_s',absolute_capture_ceiling)
    if capture_cutoff>absolute_capture_ceiling:raise RuntimeError('CAPTURE_CLOCK_CANNOT_EXTEND')
    cutoff=capture_cutoff if maximum_mode else min(datetime.fromisoformat(c['deadline_shanghai']).timestamp(),ledger['start_wall_s']+c['total_wall_budget_s'])
    if maximum_mode:c['deadline_shanghai']=datetime.fromtimestamp(capture_cutoff,ZoneInfo('Asia/Shanghai')).isoformat()
    ledger_lock=threading.RLock()
    saved_component_fingerprints={k:json.dumps(v,sort_keys=True) for k,v in ledger['components'].items()}
    def save():
        with ledger_lock, (out/'components.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            disk=json.loads(ledgerpath.read_text()) if ledgerpath.exists() else {'components':{}}
            disk.update({k:v for k,v in ledger.items() if k!='components'})
            for k,v in ledger['components'].items():
                fingerprint=json.dumps(v,sort_keys=True)
                if fingerprint!=saved_component_fingerprints.get(k):disk['components'][k]=v
                saved_component_fingerprints[k]=fingerprint
            temporary=ledgerpath.with_suffix('.tmp');temporary.write_text(json.dumps(disk,indent=2));temporary.replace(ledgerpath)
            fcntl.flock(lock,fcntl.LOCK_UN)
    def run(key,cmd,budget,env=None):
        old=ledger['components'].get(key,{})
        if old.get('status')=='COMPLETE':return
        if maximum_mode and key.startswith('capture/') and old.get('status') in ('RUNNING','FAILED','INCOMPLETE_NO_EPISODE_REPORT'):
            pid=old.get('pid')
            if pid:
                try:
                    os.kill(pid,0)
                    commandline=Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
                    if cmd[-1] not in commandline:raise RuntimeError('RESUME_PID_NOT_OWNED_BY_JOB')
                    while time.time()<cutoff:
                        try:
                            os.kill(pid,0)
                            stat=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()[0]
                            if stat=='Z':break
                        except (ProcessLookupError,FileNotFoundError):break
                        time.sleep(2)
                    else:os.kill(pid,signal.SIGINT)
                    reportpath=out/('capture_'+key.split('/')[-1])/'report.json'
                    old.update(status='COMPLETE' if reportpath.exists() else 'INCOMPLETE_NO_EPISODE_REPORT',ended_wall_s=time.time(),resumed_live_actor=True);save();return
                except ProcessLookupError:pass
            if (out/('capture_'+key.split('/')[-1])/'max_range_checkpoint.json').exists():
                row=dict(old,status='COLD_RESUME_REQUIRES_VERIFIED_ACTION_REPLAY');ledger['components'][key]=row;save();return
        actor_lock=None
        if maximum_mode and key.startswith('capture/'):
            from wrist_reconstruction.actor_owner import claim
            actor_lock=claim(key.split('/')[-1],Path(cmd[-1]),ROOT)
        remaining=min(cutoff-time.time(),budget)
        if remaining<=0:ledger['components'][key]={'status':'CUTOFF','command':cmd};save();return
        log=out/(key.replace('/','_')+'.log');row={'status':'RUNNING','command':cmd,'log':str(log),'started_wall_s':time.time()};ledger['components'][key]=row;save()
        e=dict(os.environ,PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1');e.pop('PYTHONPATH',None);e.update(env or {})
        with log.open('a') as f:
            child=subprocess.Popen(cmd,cwd=ROOT,env=e,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            row['pid']=child.pid;save()
            try:code=child.wait(timeout=remaining)
            except BaseException:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                code=child.returncode
        if actor_lock is not None:actor_lock.close()
        row.update(status='COMPLETE' if code==0 else 'FAILED',returncode=code,ended_wall_s=time.time());save()
    objects=[o for o in c['objects'] if not a.object or o['id']==a.object]
    if a.stage in ('audit-old','full'):
        run('audit-old',[str(ROOT/'environments/artgs/bin/python'),str(ROOT/'scripts/audit_artgs_sensor_inputs.py'),'--root',str(ROOT/'results/articulated_system_20261005'),'--output',str(out/'old_input_audit.json')],600)
    if a.stage in ('capture','full'):
        if a.stage=='full' and c.get('overlap_backend',False):
            for o in objects:
                key='snapshot-watch/'+o['id'];path=out/(key.replace('/','_')+'.log')
                command=[str(ROOT/'environments/artgs/bin/python'),str(ROOT/'scripts/watch_wrist_backend_ready.py'),'--capture-output',str(out),'--config',str(a.config.resolve()),'--object',o['id'],'--gpu',str(c.get('backend_gpus',{}).get(o['id'],a.gpu))]
                with path.open('a') as log:watch=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                ledger['components'][key]={'status':'RUNNING','pid':watch.pid,'command':command,'log':str(path)};save()
        capture_jobs=[]
        for o in objects:
            job=json.loads((ROOT/o['job']).read_text());job=copy.deepcopy(job)
            capture_budget=max(0,capture_cutoff-time.time()) if maximum_mode else max(0,c['capture']['wall_s']-ledger.get('resumed_capture_wall_s',{}).get(o['id'],0))
            output=out/('capture_'+o['id']);job.update(output=str(output),episode_id='wrist_'+o['id'],gpu=c.get('capture_gpus',{}).get(o['id'],a.gpu),deadline_shanghai=c['deadline_shanghai'],wall_clock_budget_s=c['capture']['wall_s'],wrist_experiment=c,camera_calibration=str((ROOT/c['camera_calibration']).resolve()),object_prompt=o['prompt'],system_capture=c['capture'])
            job['wall_clock_budget_s']=capture_budget
            if o.get('resume_closed_capture'):
                sources=o['resume_closed_capture'];sources=[sources] if isinstance(sources,str) else sources
                job['resume_closed_capture']=[str(ROOT/s) for s in sources]
            for extra in a.observation_source:
                extra=extra.resolve();manifest=extra/'multistate_capture.json'
                if not manifest.exists():continue
                observation=json.loads(manifest.read_text())
                if str(observation.get('object_id'))!=o['id']:continue
                if observation.get('capture_mode')!='wrist_camera_capture':raise RuntimeError('OBSERVATION_SOURCE_NOT_PHYSICAL_WRIST_CAPTURE')
                sources=job.setdefault('resume_closed_capture',[])
                if isinstance(sources,str):sources=[sources];job['resume_closed_capture']=sources
                sources.append(str(extra))
            if o.get('operation_memory'):job['operation_memory']=str(ROOT/o['operation_memory'])
            job['skill'].update(targets=o['targets'],capture_interval=o.get('capture_interval',30. if job['skill']['joint_type']=='revolute' else .05),minimum_capture_separation=.5 if job['skill']['joint_type']=='revolute' else .002,maximum_segments=c['capture']['maximum_segments'],maximum_sim_s=c['capture']['maximum_sim_s'],maximum_path_m=c['capture']['task_path_m'],warning_margin_rad=c['capture']['warning_margin_rad'])
            jp=out/(o['id']+'_job.json');jp.write_text(json.dumps(job,indent=2))
            capture_jobs.append((o,output,jp,capture_budget))
        def capture_one(item):
            o,output,jp,capture_budget=item
            run('capture/'+o['id'],['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'scripts/run_wrist_reconstruction_episode.py'),'--job',str(jp)],capture_budget,{'PATH':'/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH']})
            row=ledger['components']['capture/'+o['id']];row['actual_capture_directory']=str(output)
            result=output/'report.json'
            if result.exists():
                actual=json.loads(result.read_text());row['physical_status']=actual.get('status');row['bilateral_hold_established']=actual.get('bilateral_hold_established',False);row['recorded_states']=actual.get('skill_capture_states',0)
            elif row['status']=='COMPLETE':
                # Kit may return exit code zero on SIGINT before the experiment
                # writes its report. Process exit is not physical completion.
                row.update(status='INCOMPLETE_NO_EPISODE_REPORT',physical_status='NOT_VERIFIED')
            save()
        with ThreadPoolExecutor(max_workers=min(2,max(1,len(capture_jobs)))) as pool:list(pool.map(capture_one,capture_jobs))
    if a.stage in ('coarse','backend','full'):
        if maximum_mode:
            from wrist_reconstruction.readiness import both_ready
            ready,detail=both_ready(out,c)
            ledger['both_object_backend_gate']={'ready':ready,'detail':detail};save()
            if not ready:report(out,ledger,c);return
            cutoff=ledger.setdefault('backend_started_wall_s',time.time())+c['maximum_range']['backend_budget_s'];save()
        for o in objects:
            if a.stage=='full' and c.get('overlap_backend',False):
                ledger['components'][f'backend/{o["id"]}']={'status':'VERIFIED_SNAPSHOT_WATCH','output':str(out/('early_backend_'+o['id']))};save();continue
            wrist_manifest=out/('capture_'+o['id'])/'multistate_capture.json'
            if not wrist_manifest.exists():
                ledger['components'][f'backend/{o["id"]}']={'status':'WAITING_FOR_WRIST_CLEAN'};save();continue
            observed=json.loads(wrist_manifest.read_text());clean=[s for s in observed['states'] if s.get('clean_wrist_capture')]
            family=observed['joint_family_requested'];required=c['backend']['minimum_pair_span'][family]
            if len(clean)<c['backend']['minimum_clean_states'] or max([s['estimated_articulation_state'] for s in clean] or [0])-min([s['estimated_articulation_state'] for s in clean] or [0])<required:
                ledger['components'][f'backend/{o["id"]}']={'status':'WAITING_FOR_REQUIRED_CLEAN_STATES_AND_LARGE_SPAN'};save();continue
            old=ROOT/'results/articulated_system_20261005'/('7320_recovery_v2' if o['id']=='7320' else '45746_recovery_v3_infrastructure_retry')
            for variant,root in [('OLD',old),('WRIST_CLEAN',out/('capture_'+o['id'])),('ORACLE_CAPTURE',out/('capture_'+o['id'])/'oracle_capture')]:
                manifest=root/'multistate_capture.json'
                if not manifest.exists():continue
                doc=json.loads(manifest.read_text());states=doc.get('states',[])
                if variant=='WRIST_CLEAN':states=[s for s in states if s.get('clean_wrist_capture',False)]
                if len(states)<2:
                    ledger['components'][f'backend/{o["id"]}/{variant}']={'status':'BLOCKED_INSUFFICIENT_DISTINCT_CLEAN_STATES'};save();continue
                start,end=states[0]['state_id'],states[-1]['state_id'];src=out/'artgs/data/capture/sensor'/f'{o["id"]}_{variant}_fullfov'
                if not (src/'input_provenance.json').exists():
                    run(f'prepare/{o["id"]}/{variant}',[str(ROOT/'environments/artgs/bin/python'),'-c','from articulated_system.artgs import prepare;import sys;prepare(sys.argv[1],sys.argv[2],int(sys.argv[3]),int(sys.argv[4]),size=640)',str(root),str(src),str(start),str(end)],600)
                if not (src/'input_provenance.json').exists():continue
                recon=Path(str(src).replace('/data/','/outputs/'));cmd=[str(ROOT/'environments/artgs/bin/python'),str(ROOT/'scripts/run_artgs_quality_backend.py'),'--source',str(src),'--coarse-iterations','10000','--predict-iterations','5000','--joint-iterations','20000','--wall-s',str(c['backend']['wall_s_per_pair'])]
                e={'CUDA_VISIBLE_DEVICES':str(a.gpu)};key=f'backend/{o["id"]}/{variant}'
                run(key+'/coarse',cmd+['--stage','coarse'],c['backend']['wall_s_per_pair'],e)
                if not (recon/'coarse_gs/point_cloud/iteration_10000/point_cloud.ply').exists():continue
                run(key+'/diagnostics',[str(ROOT/'environments/artgs/bin/python'),str(ROOT/'scripts/artgs_coarse_diagnostics.py'),'--source',str(src)],300,e)
                if a.stage=='coarse':continue
                review=recon/'coarse_review.json'
                if not review.exists() or not json.loads(review.read_text()).get('full_optimization_allowed',False):
                    ledger['components'][key+'/joint']={'status':'COARSE_REVIEW_REQUIRED','review_path':str(review)};save();continue
                run(key+'/predict',cmd+['--stage','predict'],c['backend']['wall_s_per_pair'],e)
                run(key+'/joint',cmd+['--stage','joint'],c['backend']['wall_s_per_pair'],e)
                reconstruction=recon/'reconstruction';twin=out/'twins'/o['id']/variant
                if (reconstruction/'motion_inferred.json').exists():
                    run(key+'/twin',[str(ROOT/'environments/artgs/bin/python'),'-c','from articulated_system.twin import write;import sys;write(*sys.argv[1:])',str(reconstruction),str(src),str(root),str(twin)],600)
                    if (twin/'reconstructed.urdf').exists():run(key+'/isaac_import',['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'scripts/preview_reconstructed_twin.py'),'--twin',str(twin),'--gpu',str(a.gpu)],600,{'PATH':'/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH']})
    if a.stage not in ('capture',):report(out,ledger,c)


def report(out,ledger,c):
    ledgerpath=out/'components.json'
    if ledgerpath.exists():ledger=json.loads(ledgerpath.read_text())
    lines=['# Wrist reconstruction experiment','',f"Basis: {c['basis_commit']}. Simulation nominal camera is NOT real hand-eye calibration.",'','| Component | Status |','|---|---|']
    for k,v in ledger['components'].items():lines.append(f"| {k} | {v.get('physical_status',v['status'])} |")
    lines+=['','## Capture results','']
    for p in sorted(out.glob('capture_*/multistate_capture.json')):
        d=json.loads(p.read_text());lines.append(f"{d['object_id']}: {d['status']}")
        for s in d['states']:lines.append(f"- state {s['estimated_articulation_state']}: {len(s['views'])} views; clean={s.get('clean_wrist_capture',False)}")
        def read(name,default):
            file=p.parent/name
            return json.loads(file.read_text()) if file.exists() else default
        mobile=read('mobile_wrist_planning.json',[]);retreat=read('retreat_alternatives.json',[]);regrasp=read('reposition_history.json',[])
        proposals=sum(e.get('operation')=='view' for e in mobile)
        bases=sum(sum(c.get('kind')=='mobile' and c.get('status') is not None for c in e.get('candidates',[])) for e in mobile)
        executed_retreat=sum(e.get('status')=='RETREAT_EXECUTED' for e in retreat)
        actual_repositions=read('mobile_progress.json',{}).get('repositions',0)
        attempts=sum(len(e.get('regrasp_attempts',[])) for e in regrasp)
        physical_regrasp=sum(e.get('regrasp_completed',False) for e in regrasp)
        partial=sum(len(json.loads(f.read_text()).get('views',[])) for f in p.parent.glob('states/*/views_checkpoint.json'))
        lines+=['', '| Camera goals attempted | Base candidates checked | Base routes executed | Retreat alternatives / executed | Regrasp attempts / completed | Checkpointed wrist views |',
                '|---|---|---|---|---|---|',
                f'| {proposals} | {bases} | {actual_repositions} | {len(retreat)} / {executed_retreat} | {attempts} / {physical_regrasp} | {partial} |', '']
        for failure in [e for e in regrasp if e.get('error')]:lines.append(f"- Recovery stop: {failure['error']}")
        stops=[e for e in mobile if e.get('operation')=='physical_path_stop']
        for stop in stops:lines.append(f"- Physical path stop: {stop['reason']}; recovery={stop.get('recovery','pending')}")
    if ledger.get('resume_provenance'):
        lines+=['','## Retained previous attempt','',json.dumps(ledger['resume_provenance'],ensure_ascii=False),
                'Physical actions are reexecuted; no contact impulse, object joint state or attachment is restored. The original total cutoff and cumulative per-object capture budgets are retained.']
    lines+=['','Missing large-span clean captures block a valid A/B/C backend verdict; no fallback is triggered by missing capture. Existing contact baselines remain unchanged.']
    if c.get('maximum_range',{}).get('enabled'):
        from wrist_reconstruction.readiness import summarize
        lines+=summarize(out)
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
