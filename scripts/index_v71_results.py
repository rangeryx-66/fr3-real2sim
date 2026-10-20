"""Read-only blind progress index across original and continued V7 directories.

Never opens scene_private or HIDDEN_* records. No fitting or actor operations.
The original post-hoc evaluator is run only after every registered slot exits.
"""
import argparse,json,hashlib,datetime
from pathlib import Path


def index(root):
    root=Path(root).resolve();original=root/'results/cross_object_v7_20261019'
    continued=root/'results/cross_object_v7_v71_20261020'
    reg=json.loads((root/'docs/experiments/prismatic_preload_v71_20261020/BENCHMARK_ACTIVATION.json').read_text())
    rows=[]
    for e in reg['episodes']:
        identity=e['episode_id']
        p=original/'episodes'/identity if not e['execution_authorized'] else continued/'episodes'/('45606_0_initfix01' if identity=='45606_0' else identity)
        row=dict(episode_id=identity,asset_id=e['asset_id'],joint_type=e['joint_type'],configuration_index=e['configuration_index'],private_result=str(p),status='NOT_STARTED')
        if (p/'PROCESS.json').exists():row['status']='RUNNING'
        if (p/'PROCESS_COMPLETION.json').exists():
            done=json.loads((p/'PROCESS_COMPLETION.json').read_text());ep=json.loads((p/'EPISODE_RESULT.json').read_text()) if (p/'EPISODE_RESULT.json').exists() else {}
            trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text()) if (p/'AGENT_ACTION_TRACE.json').exists() else []
            attempts=[x['result'] for x in trace if x['command']['operation']=='attempt_grasp_and_probe']
            terminal=ep.get('safe_exit',trace[-1].get('result',{}).get('safe_exit',{}) if trace else {})
            extra=[attempts[i-1].get('failed_contact_candidate',False) for i in range(1,len(attempts)) if attempts[i].get('physical_grasp_started')]
            row.update(status=ep.get('status','NO_EPISODE_RESULT'),process=done,physical_acquisitions=sum(bool(a.get('physical_grasp_started'))for a in attempts),
                initial_acquired=bool(attempts and attempts[0].get('acquisition',{}).get('acquired')),
                failed_action_recovery_acquisitions=sum(extra),proactive_additional_acquisitions=len(extra)-sum(extra),
                final_safe_park=bool(terminal.get('safe')),attempts=[dict(family=a.get('family'),acquisition=a.get('acquisition'),error=a.get('error'),
                failed_contact_candidate=a.get('failed_contact_candidate'),safe_exit=a.get('safe_exit',{}).get('safe'),
                segments=[{k:s.get(k)for k in ['role','status','reason','gain_native','joint_coordinate','physical_transmission_valid']}for s in a.get('segments',[])])for a in attempts])
            # The action role belongs to recorder metadata, not the low-level
            # result dict. Use the saved segment contract rather than guessing.
            recorder=p/'telemetry/index.json'
            row['complete_validation']=False
            if recorder.exists():
                segments=json.loads(recorder.read_text()).get('segments',[])
                row['recorded_segments']=[dict(role=s['role'],target_q=s['target_q'],completed=s.get('completed'),
                    **{k:s.get('result',{}).get(k)for k in ['status','reason','gain_native']})for s in segments]
                row['complete_validation']=any(s['role'].startswith('VALIDATION') and s.get('result',{}).get('ok')for s in segments)
            f=p/'model/estimate.json'
            if f.exists():
                m=json.loads(f.read_text());s=m.get('selected') or {}
                row['online_model']=dict(sha256=hashlib.sha256(f.read_bytes()).hexdigest(),status=m.get('status'),parameters=s.get('parameters'),parameter_names=s.get('parameter_names'),conditional_std=s.get('conditional_std'),conditionally_supported=s.get('conditionally_supported'),heldout_motion=s.get('heldout_motion'),support_intervals_native=s.get('support_intervals_native'),hidden_truth_used=False)
            f=p/'STORAGE_SUMMARY.json'
            if f.exists():row['recorded_storage_bytes']=json.loads(f.read_text()).get('total_bytes')
        rows.append(row)
    out=dict(snapshot_UTC=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='blind pre-reveal progress snapshot; independent final evaluation is separate',
        registered_episodes=16,truth_still_withheld=True,engineering_results_not_pooled=True,original_six_outcomes_preserved=True,rows=rows)
    (continued/'BLIND_PROGRESS.json').write_text(json.dumps(out,indent=2));return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);args=p.parse_args();result=index(args.root)
    print(json.dumps({'closed_slots':sum('process'in r for r in result['rows']),'registered':16,'hidden_truth_read':False}))
