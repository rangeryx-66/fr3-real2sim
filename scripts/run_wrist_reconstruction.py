"""Independent bounded OLD/WRIST-CLEAN/ORACLE capture/backend ledger."""
import argparse,copy,json,os,sys,time,subprocess,hashlib,signal,threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/wrist_reconstruction_v2.json');p.add_argument('--output',type=Path,default=ROOT/'results/wrist_mobile_20261006/run_v2');p.add_argument('--stage',choices=['capture','audit-old','coarse','backend','full','report'],default='full');p.add_argument('--object',choices=['7320','45746']);p.add_argument('--gpu',type=int,default=0);a=p.parse_args()
    c=json.loads(a.config.read_text());out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);frozen=out/'frozen_config.json'
    if frozen.exists() and json.loads(frozen.read_text())!=c:raise RuntimeError('FROZEN_CONFIGURATION_CHANGED')
    frozen.write_text(json.dumps(c,indent=2));ledgerpath=out/'components.json';ledger=json.loads(ledgerpath.read_text()) if ledgerpath.exists() else {'start_wall_s':time.time(),'components':{}}
    cutoff=min(datetime.fromisoformat(c['deadline_shanghai']).timestamp(),ledger['start_wall_s']+c['total_wall_budget_s'])
    ledger_lock=threading.RLock()
    def save():
        with ledger_lock:
            temporary=ledgerpath.with_suffix('.tmp');temporary.write_text(json.dumps(ledger,indent=2));temporary.replace(ledgerpath)
    def run(key,cmd,budget,env=None):
        old=ledger['components'].get(key,{})
        if old.get('status')=='COMPLETE':return
        remaining=min(cutoff-time.time(),budget)
        if remaining<=0:ledger['components'][key]={'status':'CUTOFF','command':cmd};save();return
        log=out/(key.replace('/','_')+'.log');row={'status':'RUNNING','command':cmd,'log':str(log),'started_wall_s':time.time()};ledger['components'][key]=row;save()
        e=dict(os.environ,PYTHONNOUSERSITE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1');e.pop('PYTHONPATH',None);e.update(env or {})
        with log.open('a') as f:
            child=subprocess.Popen(cmd,cwd=ROOT,env=e,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=child.wait(timeout=remaining)
            except BaseException:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                code=child.returncode
        row.update(status='COMPLETE' if code==0 else 'FAILED',returncode=code,ended_wall_s=time.time());save()
    objects=[o for o in c['objects'] if not a.object or o['id']==a.object]
    if a.stage in ('audit-old','full'):
        run('audit-old',[str(ROOT/'environments/artgs/bin/python'),str(ROOT/'scripts/audit_artgs_sensor_inputs.py'),'--root',str(ROOT/'results/articulated_system_20261005'),'--output',str(out/'old_input_audit.json')],600)
    if a.stage in ('capture','full'):
        capture_jobs=[]
        for o in objects:
            job=json.loads((ROOT/o['job']).read_text());job=copy.deepcopy(job)
            output=out/('capture_'+o['id']);job.update(output=str(output),episode_id='wrist_'+o['id'],gpu=c.get('capture_gpus',{}).get(o['id'],a.gpu),deadline_shanghai=c['deadline_shanghai'],wall_clock_budget_s=c['capture']['wall_s'],wrist_experiment=c,camera_calibration=str(ROOT/c['camera_calibration']),object_prompt=o['prompt'],system_capture=c['capture'])
            job['skill'].update(targets=o['targets'],capture_interval=o.get('capture_interval',30. if job['skill']['joint_type']=='revolute' else .05),minimum_capture_separation=.5 if job['skill']['joint_type']=='revolute' else .002,maximum_segments=c['capture']['maximum_segments'],maximum_sim_s=c['capture']['maximum_sim_s'],maximum_path_m=c['capture']['task_path_m'],warning_margin_rad=c['capture']['warning_margin_rad'])
            jp=out/(o['id']+'_job.json');jp.write_text(json.dumps(job,indent=2))
            capture_jobs.append((o,output,jp))
        def capture_one(item):
            o,output,jp=item
            run('capture/'+o['id'],['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python',str(ROOT/'scripts/run_wrist_reconstruction_episode.py'),'--job',str(jp)],c['capture']['wall_s'],{'PATH':'/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static:'+os.environ['PATH']})
            row=ledger['components']['capture/'+o['id']];row['actual_capture_directory']=str(output)
            result=output/'report.json'
            if result.exists():
                actual=json.loads(result.read_text());row['physical_status']=actual.get('status');row['bilateral_hold_established']=actual.get('bilateral_hold_established',False);row['recorded_states']=actual.get('skill_capture_states',0)
            save()
        with ThreadPoolExecutor(max_workers=min(2,max(1,len(capture_jobs)))) as pool:list(pool.map(capture_one,capture_jobs))
    if a.stage in ('coarse','backend','full'):
        for o in objects:
            wrist_manifest=out/('capture_'+o['id'])/'multistate_capture.json'
            if not wrist_manifest.exists():
                ledger['components'][f'backend/{o["id"]}']={'status':'WAITING_FOR_WRIST_CLEAN'};save();continue
            observed=json.loads(wrist_manifest.read_text());clean=[s for s in observed['states'] if s.get('clean_wrist_capture')]
            family=observed['joint_family_requested'];required=c['backend']['minimum_pair_span'][family]
            if len(clean)<c['backend']['minimum_clean_states'] or max([s['estimated_articulation_state'] for s in clean] or [0])-min([s['estimated_articulation_state'] for s in clean] or [0])<required:
                ledger['components'][f'backend/{o["id"]}']={'status':'WAITING_FOR_THREE_CLEAN_STATES_AND_LARGE_SPAN'};save();continue
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
    report(out,ledger,c)


def report(out,ledger,c):
    lines=['# Wrist reconstruction experiment','',f"Basis: {c['basis_commit']}. Simulation nominal camera is NOT real hand-eye calibration.",'','| Component | Status |','|---|---|']
    for k,v in ledger['components'].items():lines.append(f"| {k} | {v.get('physical_status',v['status'])} |")
    lines+=['','## Capture results','']
    for p in sorted(out.glob('capture_*/multistate_capture.json')):
        d=json.loads(p.read_text());lines.append(f"{d['object_id']}: {d['status']}")
        for s in d['states']:lines.append(f"- state {s['estimated_articulation_state']}: {len(s['views'])} views; clean={s.get('clean_wrist_capture',False)}")
    lines+=['','Missing large-span clean captures block a valid A/B/C backend verdict; no fallback is triggered by missing capture. Existing contact baselines remain unchanged.']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
if __name__=='__main__':main()
