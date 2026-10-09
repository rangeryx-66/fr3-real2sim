"""Offline shadow reachability/accessibility of already frozen visual estimates."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from interaction_identification.contact_probe import robot_only_model
from interactive_twin_recovery.mobile import physical_scene_class
PhysicalScene=physical_scene_class(ROOT)
from rgbd_grasp_source import plan_visual
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args()
export=json.loads((a.run/'cooked_initial.json').read_text());allowed=[e['path'] for e in export['shapes'] if 'handlepiece' in e['path'].replace('_','').lower()]
model=robot_only_model(ROOT/'config/piper.urdf');scene=PhysicalScene(export,model,allowed);frame=a.run/'rgbd/initial';visual=json.loads((frame/'visual_handle.json').read_text());camera=json.loads((frame/'camera.json').read_text())
result=plan_visual(visual,model,scene,scene.moving_reference,camera['base'],a.run/'visual_shadow_plan.json');print(json.dumps({'found':bool(result['trial_candidates']),'candidate_counts':len(result['rows']),'statuses':[r['status'] for r in result['rows']]}))
