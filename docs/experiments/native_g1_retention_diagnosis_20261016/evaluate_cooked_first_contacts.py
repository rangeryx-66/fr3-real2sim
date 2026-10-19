"""Independent native-cooked fingers + handle evaluator. No ranking changes."""
import argparse,copy,json
from pathlib import Path
import numpy as np
import trimesh
from xarm7_native_planning import NativeModel,PAD_FRAMES
from piper_mobile_demo.owned_scene import meshes
from evaluate_native_g1_multiview_predictions import first
from handle_contact_quality import handle_surfaces
from revolute10_fixed import authored_handle_colliders

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--success',type=Path,required=True);p.add_argument('--failed',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    read=lambda x:json.loads(x.read_text())
    j=read(a.failed/'job.json');model=NativeModel(None,j['asset_root'],read(Path(j['source'])/'report.json'));c=read(a.failed/'episode/cooked_initial.json')
    evaluator=copy.copy(model);evaluator.robot_hulls=dict(model.robot_hulls)
    for body,name in zip(['left_finger','right_finger'],['gripper_link1','gripper_link2']):
        s=next(s for s in c['shapes'] if s.get('rigid_body_path','').split('/')[-1]==body)
        m=trimesh.util.concatenate(meshes(s));m.apply_transform(np.linalg.inv(PAD_FRAMES[body])@np.linalg.inv(s['rigid_body_world_transform'])@s['world_transform']);evaluator.robot_hulls[name]=m
    selection=j['grasp_geometry_candidates'][0];_,native=handle_surfaces(model,c,authored_handle_colliders(c,selection),selection)
    result={}
    for label,root in [('success',a.success),('failed',a.failed)]:
        plan=read(root/'episode/visual_plan.json');corr=read(root/'episode/visual_correction.json')
        result[label]={k:first(evaluator,np.asarray(T),plan['base'],native,True) for k,T in [('selected',plan['T']),('refined',corr['T_after'])]}
    a.output.write_text(json.dumps(dict(evaluation_only=True,scorer_or_executable_modified=False,poses_fixed=True,predictions=result),indent=2));print('COOKED_FINGER_FIRST_CONTACT_DONE')
