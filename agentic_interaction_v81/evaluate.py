"""Compact post-episode metrics, never an online action or fit input."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def evaluate(folder):
    import numpy as np
    from diagnose_probing_v3 import columns
    from update_probing_v7_model import load_segments
    from xarm_ft_v7_estimator import prediction,samples
    folder=Path(folder)
    if not (folder/'PROCESS_COMPLETION.json').exists():
        raise ValueError('ACTOR_MUST_FINISH_BEFORE_POST_EPISODE_EVALUATION')
    arrays,index=columns(folder);joint=index['metadata']['articulation']['joint_type']
    movement=[]
    for segment in index['segments']:
        ix=np.flatnonzero(arrays['segment']==segment['id'])
        q=arrays['joint_q'][ix];q=q[np.isfinite(q)]
        movement.append(dict(id=segment['id'],role=segment['role'],completed=segment['completed'],
            result=segment.get('result'),start=float(q[0]) if len(q) else None,
            end=float(q[-1]) if len(q) else None,maximum=float(q.max()) if len(q) else None,
            minimum=float(q.min()) if len(q) else None,records=len(ix)))
    trace=json.loads((folder/'AGENT_ACTION_TRACE.json').read_text())
    seals=[]
    for path in sorted((folder/'validation_model_seals').glob('*.json')):
        seal=json.loads(path.read_text());selected=seal.get('selected') or seal.get('online_model',{}).get('selected')
        if not selected:continue
        pieces,settings,_=load_segments(folder)
        held={}
        for piece in pieces:
            if not piece['segments'][0].startswith('VALIDATION'):continue
            intervals=selected['intervals']
            if intervals:samples(piece,selected['domain_native'],[intervals])
            held[piece['block_id']]=prediction(piece,selected)
        seals.append(dict(seal_file=path.name,model_sha256=seal.get('model_sha256'),
            parameters=selected['parameters'],conditional_std=selected['conditional_std'],
            heldout=held,validation_used_for_fit=False))
    sizes={}
    for p in folder.rglob('*'):
        if p.is_file():
            category=p.relative_to(folder).parts[0] if len(p.relative_to(folder).parts)>1 else 'root'
            sizes[category]=sizes.get(category,0)+p.stat().st_size
    return dict(joint_type=joint,units='rad' if joint=='revolute' else 'm',segments=movement,
        acquisition_attempts=sum(e['command']['operation']=='attempt_grasp_and_probe' for e in trace),
        observation_actions=sum(e['command']['operation'] in ('observe','move_and_observe','reobserve_and_replan') for e in trace),
        base_relocations=[e for e in trace if e['command']['operation']=='relocate_station'],
        prevalidation_sealed_models=seals,storage_bytes_by_category=sizes,total_storage_bytes=sum(sizes.values()),
        process=json.loads((folder/'PROCESS_COMPLETION.json').read_text()),
        hidden_truth_used=False,parameter_accuracy_requires_separate_posthoc_authored_reference=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();a.output.write_text(json.dumps(evaluate(a.episode),indent=2))
