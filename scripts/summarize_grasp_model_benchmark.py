"""Summarize completed paired R1/Dex1 model trials and collision diagnostics."""

import argparse
from collections import Counter
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROVIDERS = ('graspgenx', 'zerograsp', 'economicgrasp', 'graspness', 'anygrasp')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inference-dir', type=Path,
                        default=ROOT / 'results/grasp_model_execution')
    parser.add_argument('--execution-dir', type=Path,
                        default=ROOT / 'results/grasp_model_execution_trials')
    parser.add_argument('--pose-index', type=int, choices=range(0, 6), default=1,
                        help='1..5 selects one pose per asset; 0 summarizes all 40 scenes')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rows = json.loads((ROOT / 'results/r1a7_arena_ab/input_manifest.json').read_text())['rows']
    if args.pose_index:
        rows = [r for r in rows if (r['seed'] - 1000) % 5 + 1 == args.pose_index]
    details = []
    for row in rows:
        seed, obj = row['seed'], row['target']
        comparison = args.inference_dir / f'seed_{seed}/diagnostics/candidates.json'
        diag = json.loads(comparison.read_text()) if comparison.is_file() else None
        trial_path = args.execution_dir / obj
        trials = {}
        for provider in PROVIDERS:
            trial_index = ((seed - 1000) % 5 + 1) if args.pose_index == 0 else 1
            path = trial_path / provider / f'trial_{trial_index:02d}.json'
            trials[provider] = json.loads(path.read_text()) if path.is_file() else None
        item = {'seed': seed, 'target': obj, 'providers': {}}
        for provider in PROVIDERS:
            model = diag['models'].get(provider, {}) if diag else {}
            counts = model.get('counts', {})
            trial = trials[provider]
            grasp_path = (args.inference_dir / f'seed_{seed}/{provider}_grasps.json'
                          if provider != 'anygrasp' else Path(row['grasps']))
            selected = json.loads(grasp_path.read_text())['grasps'] if grasp_path.is_file() else []
            selected_collision = Counter(g.get('source_collision', 'FROZEN') for g in selected)
            item['providers'][provider] = {
                'inference_status': model.get('status', 'MISSING'),
                'native_candidates': counts.get('raw'),
                'strict_free': counts.get('collision_free'),
                'low_clearance': counts.get('low_clearance'),
                'mesh_collision': counts.get('collision'),
                'execution_candidates': len(selected),
                'selected_collision': dict(selected_collision),
                'executed': trial is not None,
                'success': bool(trial.get('success')) if trial else None,
                'category': trial.get('category') if trial else None,
                'path_valid': trial.get('candidate_counts', {}).get('path_valid') if trial else None,
                'planning_succeeded': bool(trial.get('planning_succeeded')) if trial else None,
                'lift_m': trial.get('lift_m') if trial else None,
                'selected_rank': trial.get('selected_rank') if trial else None,
            }
        details.append(item)
    totals = {}
    for provider in PROVIDERS:
        records = [r['providers'][provider] for r in details]
        executed = [r for r in records if r['executed']]
        totals[provider] = {
            'scenes_with_inference': sum(r['inference_status'] == 'OK' for r in records),
            'native_candidates': sum(r['native_candidates'] or 0 for r in records),
            'strict_free': sum(r['strict_free'] or 0 for r in records),
            'low_clearance': sum(r['low_clearance'] or 0 for r in records),
            'mesh_collision': sum(r['mesh_collision'] or 0 for r in records),
            'executed_scenes': len(executed),
            'successes': sum(r['success'] for r in executed),
            'success_rate': sum(r['success'] for r in executed) / len(executed) if executed else None,
            'end_to_end_success_rate': sum(r['success'] for r in executed) / len(rows),
            'planning_successes': sum(r['planning_succeeded'] for r in executed),
            'failure_categories': dict(Counter(r['category'] for r in executed if not r['success'])),
        }
    result = {'protocol': 'same eight assets, matched pose index, top100 native then Dex1 collision-aware top20, 8 adaptation variants per raw candidate',
              'pose_index': args.pose_index, 'expected_scenes': len(rows),
              'complete': all(totals[p]['executed_scenes'] == len(rows) for p in PROVIDERS),
              'totals': totals, 'scenes': details}
    out = args.output or args.execution_dir / 'benchmark_summary.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2))
    print(json.dumps({'complete': result['complete'], 'totals': totals}, indent=2))


if __name__ == '__main__':
    main()
