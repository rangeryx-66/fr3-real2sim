"""Evaluator-only IK/planning diagnostic from a source-mesh handle mask.

This never substitutes for SAM3 in the full demo and never commands motion.
"""
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
import rclpy
from articulated_demo.backend import ArticulatedBackend,Failure
from grasp_compare.adapters import read_candidates
from grasp_compare.scene import load_scene

source=ROOT/'results/articulated_47686_oracle_diagnostic'
scene=load_scene(source/'oracle_scene.npz')
all_candidates=read_candidates(source/'graspgenx_native.json','graspgenx',scene)
by_rank={c.rank:c for c in all_candidates}
common=json.loads((source/'common_filter/candidates.json').read_text())
results={'kind':'evaluator_only_oracle_mask; not SAM3, not execution',
         'collision_counts':common['models']['graspgenx']['counts'],'candidates':[]}
output=source/'moveit_preflight.json'
rclpy.init();node=ArticulatedBackend('/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1/urdf/47686.urdf')
try:
    node.scene();results['fk_tcp_check']=node.check_fk()
    for item in common['candidates']['graspgenx']:
        candidate=by_rank[item['rank']]
        row={'rank':candidate.rank,'score':candidate.score,
             'collision_status':item['checks']['dex1_scene_collision']['status']}
        try:
            plan=node.candidate_plan(candidate)
            row.update(status='PATH_VALID',grasp_margin_rad=plan['grasp_margin_rad'],
                       path_margin_rad=plan['path_margin_rad'])
        except Failure as error:
            row.update(status=error.category,detail=str(error))
        results['candidates'].append(row)
        output.write_text(json.dumps(results,indent=2))
        print(row,flush=True)
finally:
    results['path_valid']=sum(x['status']=='PATH_VALID' for x in results['candidates'])
    output.write_text(json.dumps(results,indent=2))
    node.destroy_node();rclpy.shutdown()
