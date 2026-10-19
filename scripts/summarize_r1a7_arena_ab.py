"""Summarize completed paired Arena scenes, preserving partial-run denominators."""
import json
import math
import statistics as stats
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/r1a7_arena_ab'
manifest=json.loads((OUT/'input_manifest.json').read_text())

def read(path):return json.loads(path.read_text()) if path.exists() else None
def rate(rows,key):return dict(success=sum(bool(r.get(key)) for r in rows),total=len(rows))
def executed(result,stage):return any(item.get('stage')==stage for item in result.get('executions',[])) if result else False
def dist(values):
    return dict(n=len(values),minimum=min(values),median=stats.median(values),maximum=max(values),mean=stats.mean(values)) if values else dict(n=0)

rows=[]
for entry in manifest['rows']:
    seed=entry['seed'];target=entry['target']
    order=[r['seed'] for r in manifest['rows'] if r['target']==target].index(seed)+1
    base=OUT/target
    raw=read(base/'raw'/f'trial_{order:02d}.json')
    adapted=read(base/'adapted'/f'trial_{order:02d}.json')
    fr3=read(OUT/'fr3_baseline'/f'B_seed_{seed:04d}.json')
    if fr3 is None:raise FileNotFoundError(f'FR3 scene {seed} missing')
    if fr3['success'] and not (fr3.get('lift_ge_8cm') and fr3.get('stable_hold_ge_2s')):
        raise ValueError(f'FR3 scene {seed} marked success without lift/hold')
    for mode,result in (('raw',raw),('adapted',adapted)):
        if result is not None:
            ref=result.get('reference_input',{})
            if ref.get('cloud_sha256')!=entry['cloud_sha256'] or ref.get('grasps_sha256')!=entry['grasps_sha256']:
                raise ValueError(f'{mode} scene {seed} input hash mismatch')
            if result.get('scenario',{}).get('seed')!=seed or result.get('object',{}).get('id')!=target:
                raise ValueError(f'{mode} scene {seed} target/seed mismatch')
            if result.get('success'):
                samples=result.get('hold_samples',[])
                z0=result.get('initial_z')
                if (result.get('category')!='SUCCESS' or result.get('lift_m',0)<.08 or
                    len(samples)<2 or samples[-1]['t']-samples[0]['t']<2. or
                    z0 is None or min(item['z'] for item in samples)<z0+.07):
                    raise ValueError(f'{mode} scene {seed} marked success without stable physical lift')
    selected=None
    if adapted:
        selected=next((c for c in adapted.get('candidates',[]) if c['rank']==adapted.get('selected_rank')),None)
    feasible_adaptations=[dict(rank=c['rank'],translation_m=c['translation_m'],rotation_rad=c['rotation_rad'],
                               joint_margin_rad=c.get('joint_margin_rad'))
                          for c in adapted.get('candidates',[]) if c.get('translation_m') is not None] if adapted else []
    def candidate_statuses(result):
        return dict(Counter(c.get('status','UNKNOWN') for c in result.get('candidates',[]))) if result else {}
    def variant_failures(result):
        count=Counter()
        if result:
            for candidate in result.get('candidates',[]):count.update(candidate.get('failures',{}))
        return dict(count)
    rows.append(dict(seed=seed,target=target,position_index=order,
                     xy=entry['pose']['position'][:2],yaw_rad=2*math.atan2(entry['pose']['quaternion_wxyz'][3],entry['pose']['quaternion_wxyz'][0]),
                     grasp_count=entry['grasp_count'],target_points=entry['target_points'],
                     fr3_category=fr3['category'],fr3_success=bool(fr3['success']),
                     fr3_valid_candidates=sum(c['status']=='VALID' for c in fr3['candidates']),
                     fr3_selected_rank=fr3.get('selected_rank'),
                     fr3_pregrasp_executed=executed(fr3,'PREGRASP'),
                     fr3_approach_recorded=executed(fr3,'APPROACH'),
                     raw_category=raw.get('category') if raw else 'MISSING',raw_success=bool(raw and raw['success']),
                     raw_path_valid=raw.get('candidate_counts',{}).get('path_valid',0) if raw else None,
                     raw_planned=bool(raw and raw.get('planning_succeeded')),
                     raw_approach_recorded=executed(raw,'APPROACH'),
                     raw_candidate_statuses=candidate_statuses(raw),raw_variant_failures=variant_failures(raw),
                     adapted_category=adapted.get('category') if adapted else 'MISSING',
                     adapted_success=bool(adapted and adapted['success']),
                     adapted_path_valid=adapted.get('candidate_counts',{}).get('path_valid',0) if adapted else None,
                     adapted_planned=bool(adapted and adapted.get('planning_succeeded')),
                     adapted_approach_recorded=executed(adapted,'APPROACH'),
                     adapted_candidate_statuses=candidate_statuses(adapted),adapted_variant_failures=variant_failures(adapted),
                     contact_loss_events=sum(a.get('status')=='CONTACT_LOSS' or a.get('initial_contact_failure')=='CONTACT_LOSS'
                                             for a in adapted.get('candidate_attempts',[])) if adapted else None,
                     selected_rank=adapted.get('selected_rank') if adapted else None,
                     translation_m=selected.get('translation_m') if selected else None,
                     rotation_rad=selected.get('rotation_rad') if selected else None,
                     feasible_adaptations=feasible_adaptations,
                     lift_m=adapted.get('lift_m') if adapted else None))

paired=[r for r in rows if r['raw_category']!='MISSING' and r['adapted_category']!='MISSING']
groups={target:[r for r in rows if r['target']==target] for target in sorted({r['target'] for r in rows})}
by_object={}
for target,data in groups.items():
    raw_rows=[r for r in data if r['raw_category']!='MISSING']
    adapted_rows=[r for r in data if r['adapted_category']!='MISSING']
    by_object[target]=dict(fr3=rate(data,'fr3_success'),raw=rate(raw_rows,'raw_success'),
                           adapted=rate(adapted_rows,'adapted_success'),
                           grasp_candidates=sum(r['grasp_count'] for r in data),
                           raw_path_valid=sum(r['raw_path_valid'] for r in raw_rows),
                           adapted_path_valid=sum(r['adapted_path_valid'] for r in adapted_rows),
                           raw_pregrasp_planned=sum(r['raw_planned'] for r in raw_rows),
                           adapted_pregrasp_planned=sum(r['adapted_planned'] for r in adapted_rows),
                           adapted_approach_recorded=sum(r['adapted_approach_recorded'] for r in adapted_rows),
                           adapted_contact_loss_events=sum(r['contact_loss_events'] for r in adapted_rows),
                           adapted_final_categories=dict(Counter(r['adapted_category'] for r in adapted_rows)))
by_position={str(i):dict(fr3=rate([r for r in rows if r['position_index']==i],'fr3_success'),
                         raw=rate([r for r in rows if r['position_index']==i and r['raw_category']!='MISSING'],'raw_success'),
                         adapted=rate([r for r in rows if r['position_index']==i and r['adapted_category']!='MISSING'],'adapted_success'))
             for i in range(1,6)}
completed_raw=[r for r in rows if r['raw_category']!='MISSING']
completed_adapted=[r for r in rows if r['adapted_category']!='MISSING']
selected=[r for r in completed_adapted if r['translation_m'] is not None]
all_feasible=[v for r in completed_adapted for v in r['feasible_adaptations']]
def combined(rows,key):
    count=Counter()
    for row in rows:count.update(row[key])
    return dict(count)
summary=dict(planned_scenes=len(rows),paired_complete=len(paired),
             fr3_full=rate(rows,'fr3_success'),fr3_paired=rate(paired,'fr3_success'),
             raw=rate(completed_raw,'raw_success'),adapted=rate(completed_adapted,'adapted_success'),
             raw_paired=rate(paired,'raw_success'),adapted_paired=rate(paired,'adapted_success'),
             by_object=by_object,by_position=by_position,
             path_valid_coverage=dict(fr3=sum(r['fr3_valid_candidates'] for r in paired),
                                      raw=sum(r['raw_path_valid'] for r in paired),
                                      adapted=sum(r['adapted_path_valid'] for r in paired),
                                      candidates=sum(r['grasp_count'] for r in paired)),
             r1_pregrasp_planning_paired=dict(raw=sum(r['raw_planned'] for r in paired),
                                              adapted=sum(r['adapted_planned'] for r in paired),
                                              paired_scenes=len(paired)),
             fr3_pregrasp_execution_paired=sum(r['fr3_pregrasp_executed'] for r in paired),
             approach_trajectory_recorded=dict(fr3=sum(r['fr3_approach_recorded'] for r in paired),
                                               raw=sum(r['raw_approach_recorded'] for r in paired),
                                               adapted=sum(r['adapted_approach_recorded'] for r in paired),
                                               paired_scenes=len(paired)),
             raw_planning_success=sum(r['raw_planned'] for r in completed_raw),
             adapted_planning_success=sum(r['adapted_planned'] for r in completed_adapted),
             adapted_contact_loss_events=sum(r['contact_loss_events'] for r in completed_adapted),
             raw_failures=dict(Counter(r['raw_category'] for r in completed_raw if not r['raw_success'])),
             adapted_failures=dict(Counter(r['adapted_category'] for r in completed_adapted if not r['adapted_success'])),
             fr3_selected_ranks=dict(Counter(str(r['fr3_selected_rank']) for r in paired if r['fr3_selected_rank'] is not None)),
             raw_candidate_statuses=combined(completed_raw,'raw_candidate_statuses'),
             adapted_candidate_statuses=combined(completed_adapted,'adapted_candidate_statuses'),
             raw_variant_failures=combined(completed_raw,'raw_variant_failures'),
             adapted_variant_failures=combined(completed_adapted,'adapted_variant_failures'),
             selected_ranks=dict(Counter(str(r['selected_rank']) for r in selected)),
             successful_selected_ranks=dict(Counter(str(r['selected_rank']) for r in selected if r['adapted_success'])),
             selected_translation_m=dist([r['translation_m'] for r in selected]),
             selected_rotation_rad=dist([r['rotation_rad'] for r in selected]),
             selected_offsets_22_5_to_27_5mm=sum(.0225<=r['translation_m']<=.0275 for r in selected),
             successful_translation_m=dist([r['translation_m'] for r in selected if r['adapted_success']]),
             successful_rotation_rad=dist([r['rotation_rad'] for r in selected if r['adapted_success']]),
             feasible_adaptation_count=len(all_feasible),
             feasible_translation_m=dist([v['translation_m'] for v in all_feasible]),
             feasible_rotation_rad=dist([v['rotation_rad'] for v in all_feasible]),
             feasible_joint_margin_rad=dist([v['joint_margin_rad'] for v in all_feasible if v['joint_margin_rad'] is not None]),
             feasible_margin_gt_0_08=sum(v['joint_margin_rad']>.08 for v in all_feasible if v['joint_margin_rad'] is not None),
             feasible_ranks=dict(Counter(str(v['rank']) for v in all_feasible)),
             feasible_offsets_22_5_to_27_5mm=sum(.0225<=v['translation_m']<=.0275 for v in all_feasible),
             rows=rows)
(OUT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=2))
