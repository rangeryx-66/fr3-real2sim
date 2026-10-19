"""Separate evaluator: GT cannot generate, score, or correct executable grasps."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import trimesh,fcl
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from xarm7_native_planning import NativeModel,LOGICAL
from piper_mobile_demo.owned_scene import Shape

def first(model,T,base,meshes,convex=False):
    solids=[Shape(m,np.eye(4),convex,str(i),'evaluation') for i,m in enumerate(meshes)]
    robots={n:Shape(model.robot_hulls[n],np.eye(4),True,n,n) for n in LOGICAL}
    def hit(n,w):
        P=model.poses(model.home,base,width=float(w));D=T@np.linalg.inv(P['tcp_link']);robots[n].place(D@P[n])
        for s in solids:
            result=fcl.CollisionResult();fcl.collide(robots[n].object,s.object,fcl.CollisionRequest(num_max_contacts=1,enable_contact=True),result)
            if result.contacts:return True
        return False
    result={}
    widths=np.linspace(.084,.001641,41)
    for n in LOGICAL:
        value=None
        for i,w in enumerate(widths):
            if hit(n,w):
                low=float(w);high=float(widths[max(0,i-1)])
                for _ in range(8):
                    mid=(low+high)/2
                    if hit(n,mid):low=mid
                    else:high=mid
                value=dict(first_aperture_mm=low*1000,bracket_mm=[low*1000,high*1000],coarse_aperture_mm=float(w*1000));break
        result[n]=value
    return result

def main(a):
    old=ROOT/'results/native_g1_visual_reliability_20261015'/a.asset
    job=json.loads((old/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text())
    model=NativeModel(None,job['asset_root'],source);rows=[]
    from handle_contact_quality import handle_surfaces
    from revolute10_fixed import authored_handle_colliders
    # GT is read ONLY here after all candidate choices have been persisted.
    export=json.loads((old/'physical_01/cooked_initial.json').read_text())
    selection=job['grasp_geometry_candidates'][0]
    gt,native=handle_surfaces(model,export,authored_handle_colliders(export,selection),selection)
    for mode in ['single','multi']:
        d=a.root/('offline/'+a.asset+'_'+mode)
        if not (d/'visual_plan.json').exists():
            rows.append(dict(mode=mode,planning=False));continue
        candidate=json.loads((d/'visual_plan.json').read_text());T=np.asarray(candidate['T']);base=candidate['base'];geo=d/'geometry'
        observed=trimesh.load(geo/'observed_patches.ply',process=False)
        predictions=dict(observed=first(model,T,base,[observed]))
        for i,scale in enumerate([.75,1.,1.25]):
            assumed=trimesh.load(geo/('uncertain_rear_%d.ply'%i),process=False)
            predictions[str(scale)]=first(model,T,base,[observed,assumed])
        predictions['GT_visual_evaluation_only']=first(model,T,base,[gt])
        predictions['GT_native_evaluation_only']=first(model,T,base,native,True)
        rows.append(dict(mode=mode,planning=True,candidate_pose_source='observation only; frozen before this evaluator',section=candidate['section_fraction'],insertion_m=candidate['insertion_m'],roll=candidate['roll_deg'],pitch=candidate['pitch_deg'],planned_deg=float(np.rad2deg(candidate['planned_reached_state'])),min_margin_rad=candidate['minimum_approach_arc_margin_rad'],contact_scores={k:v for k,v in candidate['contact_geometry_audit']['robust_contact'].items() if k!='aperture_trials'},predictions=predictions,geometry=json.loads((geo/'observed_geometry.json').read_text())))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(dict(asset=a.asset,GT_execution_fallback=False,aperture_refinement='41 coarse brackets + 8 offline bisections, approximately 0.008mm; controller unchanged',rows=rows),indent=2))
    print('PREDICTIONS_DONE',a.asset,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--asset',required=True);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
