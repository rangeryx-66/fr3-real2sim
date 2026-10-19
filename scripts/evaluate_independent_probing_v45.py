"""Post-exit V4.5 evaluation wrapper; existing V3.1 math/output stays immutable."""
import argparse,json,os
from pathlib import Path
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet


def evaluate(root):
    root=Path(root);episode=root/'episode_00'
    completion=json.loads((episode/'PROCESS_COMPLETION.json').read_text())
    pid=json.loads((episode/'PROCESS.json').read_text())['pid']
    if not completion['actor_exited'] or Path('/proc/%d'%pid).exists():raise RuntimeError('No truth access before actor and online decisions finish')
    # Reuse the existing qualified-interval, authored-reference, legacy-diagnostic
    # and free forward-prediction evaluator. No new fit or truth-based selection.
    from evaluate_independent_probing_v43 import evaluate as previous
    result=previous(root)
    result.update(experiment='V4.5',reference='bcf5dda',registration='b76148b',
                  episode_kind='one fresh blind research episode, no engineering fixture')
    complete=[q for q in result['quality'] if q['role'].startswith('VALIDATION') and q.get('quality',{}).get('action_ok') and q.get('blocks')]
    result['assessment']=dict(status='REQUIRES_CREDIBILITY_REVIEW' if complete else 'FAIL',complete_validation=bool(complete),
        note='PASS also requires physically credible identifiable parameters and held-out prediction; no automatic training-residual or angle-based PASS')
    atomic_json(root/'RESULT.json',result_packet(result))
    print(json.dumps(result_packet({k:result[k] for k in ['experiment','assessment','comparison','prevalidation_frozen_model','storage','completion']}),indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);evaluate(p.parse_args().root)
