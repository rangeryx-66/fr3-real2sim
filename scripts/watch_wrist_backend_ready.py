"""Freeze verified observation snapshots and start ArtGS while capture continues.

This observer never controls the robot or initializes physical state.
"""
import argparse,json,time,subprocess,os,shutil,signal
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1]


def freeze(source,target,indices):
    doc=json.loads((source/'multistate_capture.json').read_text())
    selected=[doc['states'][i] for i in indices]
    target.mkdir(parents=True,exist_ok=True)
    for state in selected:
        for view in state['views']:
            src=source/view['directory'];dst=target/view['directory']
            shutil.copytree(src,dst,dirs_exist_ok=True)
    doc.update(states=selected,status='FROZEN_OBSERVATION_SNAPSHOT',snapshot_source=str(source),new_robot_execution=False)
    (target/'multistate_capture.json').write_text(json.dumps(doc,indent=2))
    for name in ['structured_memory.json','effort_sensor_capability.json','effort_summary.json','effort_segments.json']:
        if (source/name).exists():shutil.copy2(source/name,target/name)


def main():
    p=argparse.ArgumentParser();p.add_argument('--capture-output',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--object',required=True);p.add_argument('--gpu',type=int,required=True);a=p.parse_args()
    source=a.capture_output.resolve();config=a.config.resolve();c=json.loads(config.read_text())
    ledger=json.loads((source/'components.json').read_text());start=ledger['start_wall_s']
    cutoff=min(start+c['total_wall_budget_s'],datetime.fromisoformat(c['deadline_shanghai']).timestamp())
    out=source/('early_backend_'+a.object);out.mkdir(parents=True,exist_ok=True);status=out/'watch_status.json'
    def save(state,**fields):status.write_text(json.dumps(dict(state=state,source=str(source),deadline_wall_s=cutoff,**fields),indent=2))
    capture=source/('capture_'+a.object);save('WAITING_FOR_VERIFIED_DATA')
    while time.time()<cutoff:
        try:
            doc=json.loads((capture/'multistate_capture.json').read_text())
            good=[(i,s) for i,s in enumerate(doc['states']) if s.get('clean_wrist_capture') and len(s['views'])>=c['capture']['minimum_clean_views']]
            span=max([s['estimated_articulation_state'] for _,s in good] or [0])-min([s['estimated_articulation_state'] for _,s in good] or [0])
            if len(good)>=c['backend']['minimum_clean_states'] and span>=c['backend']['minimum_pair_span'][doc['joint_family_requested']]:
                first,last=good[0],good[-1];middle=min(good[1:-1],key=lambda x:abs(x[1]['estimated_articulation_state']-(first[1]['estimated_articulation_state']+last[1]['estimated_articulation_state'])/2))
                indices=[first[0],middle[0],last[0]];freeze(capture,out/('capture_'+a.object),indices)
                oracle=capture/'oracle_capture'
                if (oracle/'multistate_capture.json').exists():
                    od=json.loads((oracle/'multistate_capture.json').read_text());matched=[]
                    for i in indices:
                        value=doc['states'][i]['estimated_articulation_state'];matches=[j for j,s in enumerate(od['states']) if abs(s['estimated_articulation_state']-value)<1e-6]
                        if not matches:break
                        matched.append(matches[0])
                    if len(matched)==3:freeze(oracle,out/('capture_'+a.object)/'oracle_capture',matched)
                (out/'components.json').write_text(json.dumps({'start_wall_s':start,'components':{},'snapshot_provenance':{'original_capture':str(capture),'state_indices':indices,'new_physical_actions':False}},indent=2))
                command=[str(ROOT/'environments/artgs/bin/python'),str(ROOT/'scripts/run_wrist_reconstruction.py'),'--config',str(config),'--output',str(out),'--object',a.object,'--stage','backend','--gpu',str(a.gpu)]
                save('BACKEND_RUNNING',state_indices=indices,command=command)
                with (out/'backend_driver.log').open('a') as log:
                    process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    try:code=process.wait(timeout=max(1,cutoff-time.time()))
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGINT)
                        try:process.wait(timeout=30)
                        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
                        save('GLOBAL_BUDGET_EXHAUSTED');return
                save('BACKEND_STAGE_RETURNED',returncode=code,review_may_be_required=True);return
            if (capture/'report.json').exists():save('CAPTURE_TERMINAL_WITHOUT_REQUIRED_DATA',clean_states=len(good),span=span);return
        except (FileNotFoundError,json.JSONDecodeError):pass
        time.sleep(20)
    save('GLOBAL_BUDGET_EXHAUSTED')


if __name__=='__main__':main()
