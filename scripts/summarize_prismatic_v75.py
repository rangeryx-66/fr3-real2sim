"""Compact post-episode metrics; never exports raw observations or measurements."""
import argparse, hashlib, json
from pathlib import Path


def summarize(path):
    p=Path(path)
    completion=json.loads((p/'PROCESS_COMPLETION.json').read_text())
    if not completion.get('actor_exited'):
        raise ValueError('Online actor must finish before post-episode summary')
    result=json.loads((p/'EPISODE_RESULT.json').read_text())
    index_path=p/'telemetry/index.json'
    index=json.loads(index_path.read_text())
    segments=[]
    for s in index.get('segments',[]):
        r=s.get('result',{})
        segments.append(dict(role=s['role'],status=r.get('status'),
            travel_m=r.get('gain_native'),end_coordinate_m=r.get('joint_coordinate'),
            physical_duration_s=(s['last_step']-s['first_step']+1)/240,
            requested_target_reached=r.get('requested_target_reached',False),
            transmission_valid=r.get('physical_transmission_valid'),
            reason=r.get('reason')))
    model_path=p/'model/estimate.json'
    model=json.loads(model_path.read_text())if model_path.exists()else{}
    selected=model.get('selected')or{}
    predictions=selected.get('heldout_motion',{})
    model_summary={k:selected.get(k)for k in ['parameters','conditional_std',
        'scaled_condition','support_intervals_native','conditionally_supported',
        'uncalibrated_total_physical_accuracy','FIT_rmse']}
    model_summary.update(heldout_motion=predictions,
        worker_wall_s=model.get('worker_wall_s'),
        model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest()if model_path.exists()else None,
        validation_used_for_fit_or_selection=model.get('validation_used_for_fit_or_selection'),
        tool_compensation_count=model.get('tool_compensation_count'))
    safe=result.get('safe_exit')or{}
    obs=json.loads((p/'AGENT_OBSERVATIONS.json').read_text())if(p/'AGENT_OBSERVATIONS.json').exists()else[]
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text())if(p/'AGENT_ACTION_TRACE.json').exists()else[]
    attempts=[x for x in trace if x.get('command',{}).get('operation')=='attempt_grasp_and_probe']
    categories={}
    for f in p.rglob('*'):
        if not f.is_file():continue
        rel=f.relative_to(p);top=rel.parts[0]
        category='RGBD_checkpoints'if top=='rgbd'else'numeric'if top in ('telemetry','contact_evidence')else'geometry'if top in ('agent_geometry','incremental_feedback','contact_feedback')else'source_and_metadata'
        categories[category]=categories.get(category,0)+f.stat().st_size
    return dict(episode=p.name,classification='engineering; not a benchmark rate',
        status=result.get('status'),safe_exit=bool(safe.get('safe')),
        verified_park=bool(safe.get('park',{}).get('reached')),
        physical_attempt_operations=len(attempts),observations=len(obs),
        requested_preloads_N=[x['command'].get('native_g1_preload_request_n')for x in attempts],
        segments=segments,online_model=model_summary,
        launcher_wall_s=completion['wall_s'],
        bytes_by_category=categories,total_bytes=sum(categories.values()),
        index_sha256=hashlib.sha256(index_path.read_bytes()).hexdigest(),
        raw_data_exported=False,hidden_truth_read=False)


if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('episodes',nargs='+',type=Path)
    a.add_argument('--output',required=True,type=Path);args=a.parse_args()
    rows=[summarize(p)for p in args.episodes]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(episodes=rows,units='SI'),indent=2)+'\n')
    print(json.dumps([dict(episode=r['episode'],status=r['status'],bytes=r['total_bytes'])for r in rows]))
