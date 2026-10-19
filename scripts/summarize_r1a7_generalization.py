"""Aggregate paired rigid-object R1 generalization trials without rerunning them."""
import json
import statistics as stats
from collections import Counter,defaultdict
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]/'results/r1a7_generalization'
OBJECTS=json.loads((Path(__file__).resolve().parents[1]/'config/r1a7_generalization_objects.json').read_text())

def read(path):return json.loads(path.read_text()) if path.exists() else None
def ratio(rows,field):return dict(success=sum(bool(r.get(field)) for r in rows),total=len(rows))
def distribution(values):
    return dict(n=len(values),minimum=min(values),median=stats.median(values),maximum=max(values),mean=stats.mean(values)) if values else dict(n=0)

rows=[]
for obj in OBJECTS:
    object_id=obj['id']
    scenarios=read(ROOT/'scenarios'/f'{object_id}.json') or []
    for i,scenario in enumerate(scenarios,1):
        capture=read(ROOT/object_id/'capture'/f'trial_{i:02d}.json') or {}
        raw=read(ROOT/object_id/'raw'/f'trial_{i:02d}.json') or {}
        adapted=read(ROOT/object_id/'adapted'/f'trial_{i:02d}.json') or {}
        grasps=read(ROOT/object_id/'capture'/f'trial_{i:02d}_grasps.json') or {}
        widths=[g['width'] for g in grasps.get('grasps',[])]
        selected=next((c for c in adapted.get('candidates',[]) if c.get('status')=='PLANNED'),{})
        visible=capture.get('capture',{}).get('object_points',capture.get('object_points',0))>=30
        row=dict(object_id=object_id,shape=obj['shape'],height_m=obj.get('height',obj.get('size',[0,0,0])[2]),
                 placement=i,xy=scenario['xy'],yaw_rad=scenario['yaw'],position_band=scenario['position_band'],
                 capture_category=capture.get('category','MISSING'),target_visible=visible,
                 anygrasp_candidates=len(grasps.get('grasps',[])),
                 target_region_candidates=len(grasps.get('grasps',[])) if visible else 0,
                 anygrasp_best_score=max((g['score'] for g in grasps.get('grasps',[])),default=None),
                 anygrasp_width_range_m=[min(widths),max(widths)] if widths else None,
                 object_points=capture.get('capture',{}).get('object_points'),
                 raw_category=raw.get('category','MISSING'),raw_success=raw.get('success',False),
                 raw_path_valid=raw.get('candidate_counts',{}).get('path_valid',0),
                 adapted_category=adapted.get('category','MISSING'),adapted_success=adapted.get('success',False),
                 adapted_path_valid=adapted.get('candidate_counts',{}).get('path_valid',0),
                 planned=bool(selected),selected_rank=adapted.get('selected_rank'),
                 selected_raw_width_m=selected.get('raw_grasp',{}).get('width'),
                 selected_raw_score=selected.get('raw_grasp',{}).get('score'),
                 translation_m=selected.get('translation_m'),rotation_rad=selected.get('rotation_rad'),
                 joint_margin_rad=selected.get('joint_margin_rad'),lift_m=adapted.get('lift_m'),
                 candidate_failures=dict(Counter(c.get('status','UNKNOWN') for c in adapted.get('candidates',[]))))
        rows.append(row)

complete=[r for r in rows if r['raw_category']!='MISSING' and r['adapted_category']!='MISSING']
objects={obj['id']:dict(raw=ratio([r for r in complete if r['object_id']==obj['id']],'raw_success'),
                        adapted=ratio([r for r in complete if r['object_id']==obj['id']],'adapted_success')) for obj in OBJECTS}
positions={band:dict(raw=ratio([r for r in complete if r['position_band']==band],'raw_success'),
                     adapted=ratio([r for r in complete if r['position_band']==band],'adapted_success'))
           for band in ('near','center','far')}
offsets=[r['translation_m'] for r in complete if r['translation_m'] is not None]
rotations=[r['rotation_rad'] for r in complete if r['rotation_rad'] is not None]
summary=dict(total_planned=len(rows),total_complete=len(complete),
             raw=ratio(complete,'raw_success'),adapted=ratio(complete,'adapted_success'),
             target_visible_scenes=sum(r['target_visible'] for r in complete),
             visible_only=dict(raw=ratio([r for r in complete if r['target_visible']],'raw_success'),
                               adapted=ratio([r for r in complete if r['target_visible']],'adapted_success')),
             by_object=objects,by_position_band=positions,
             executable_candidates=dict(raw=sum(r['raw_path_valid'] for r in complete),
                                        adapted=sum(r['adapted_path_valid'] for r in complete),
                                        anygrasp_in_target_region=sum(r['target_region_candidates'] for r in complete),
                                        anygrasp_raw_output=sum(r['anygrasp_candidates'] for r in complete)),
             final_failures=dict(Counter(r['adapted_category'] for r in complete if not r['adapted_success'])),
             candidate_failures=dict(sum((Counter(r['candidate_failures']) for r in complete),Counter())),
             selected_translation_m=distribution(offsets),selected_rotation_rad=distribution(rotations),
             selected_offsets_22_5_to_27_5mm=sum(.0225<=v<=.0275 for v in offsets),
             selected_offsets_by_object={obj['id']:distribution([r['translation_m'] for r in complete if r['object_id']==obj['id'] and r['translation_m'] is not None]) for obj in OBJECTS},
             target_points_by_outcome={status:distribution([r['object_points'] for r in complete if r['adapted_success']==status and r['object_points'] is not None]) for status in (False,True)},
             grasp_candidates_by_outcome={status:distribution([r['target_region_candidates'] for r in complete if r['adapted_success']==status]) for status in (False,True)},
             rows=rows)
(ROOT/'analysis.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k!='rows'},indent=2))
