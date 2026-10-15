"""Offline view feasibility. Archived RGB-D is diagnostic/proposal input only.

Live execution regenerates subsequent views from fresh wrist observations.
No saved grasp pose or GT handle mesh is used to solve a camera pose.
"""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from native_g1_wrist_views import *
from xarm7_native_planning import NativeModel,scene_factory
from cross_robot_adapter import RobotPinkIK
from xarm7_native_gripper import configuration

def visibility_diagnostic(model,base,route,points,c,X):
    from native_g1_wrist_input import self_mask
    q=np.asarray(route['q_path'][-1]);poses=model.poses(q,base,width=.084);C=poses['tcp_link']@X
    Q=(points-C[:3,3])@C[:3,:3];K=np.asarray(c['K']);uv=Q@K.T;uv=np.rint(uv[:,:2]/np.maximum(Q[:,2,None],1e-9)).astype(int)
    w,h=c['resolution_wh'];ok=(Q[:,2]>.025)&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    depth=np.full((h,w),np.inf);xy=uv[ok];z=Q[ok,2];np.minimum.at(depth,(xy[:,1],xy[:,0]),z)
    mask=self_mask(depth,K,C,model,poses)
    return dict(archived_observed_points_in_FOV_fraction=float(ok.mean()),robot_occluded_fraction_of_in_FOV_points=float(mask[xy[:,1],xy[:,0]].mean()) if len(xy) else None,source='archived observed points + known robot geometry; diagnostic only, not pose selection')

def main(a):
    a.output.mkdir(parents=True,exist_ok=True);c,X=calibration();results={}
    for aid in ['45600','45606']:
        begun=time.perf_counter();old=ROOT/'results/native_g1_multiview_skill_20261015'/aid
        job=json.loads((old/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text())
        m=NativeModel(None,job['asset_root'],source);b=source['robot_base_pose'];solver=RobotPinkIK(configuration())
        export=json.loads((Path(job['output'])/'cooked_initial.json').read_text())
        # Collider identities associate the moving rigid body for collision
        # checks only. Their vertices never generate a view/grasp target.
        allowed=json.loads((Path(job['output'])/'visual_plan.json').read_text())['allowed_authored_colliders']
        scene=scene_factory(ROOT,export,m,allowed)
        observed=np.load(Path(job['output'])/'multiview_geometry/points_world.npy')
        data=json.loads((Path(job['output'])/'multiview_geometry/observed_geometry.json').read_text())
        survey,logs=select_route(m,solver,scene,b,m.home,survey_candidates(m,c),c,X,observed)
        views=[]
        if survey:
            survey['visibility_diagnostic']=visibility_diagnostic(m,b,survey,observed,c,X)
            start=np.asarray(survey['q_path'][-1])
            for sign in [-1,1]:
                proposals=[p for p in observed_candidates(observed,data['normal_world'],c) if np.sign(p['azimuth_deg'])==sign]
                route,audit=select_route(m,solver,scene,b,start,proposals,c,X,observed)
                if route:route['visibility_diagnostic']=visibility_diagnostic(m,b,route,observed,c,X)
                views.append(dict(selected=route,attempts=audit))
                if route:start=np.asarray(route['q_path'][-1])
        result=dict(asset=aid,survey=survey,survey_attempts=logs,additional_views=views,wall_s=time.perf_counter()-begun,
                    no_physics=True,GT_handle_mesh_or_saved_grasp_used=False,archived_RGBD_used_for_offline_view_proposals_only=True,
                    live_execution_uses_fresh_wrist_points=True,base=b,calibration=c)
        results[aid]=result;(a.output/(aid+'.json')).write_text(json.dumps(result,indent=2))
        print('WRIST_OFFLINE',aid,'survey',bool(survey),'views',sum(v['selected'] is not None for v in views),'wall',result['wall_s'],flush=True)
    (a.output/'OFFLINE_REACHABILITY.json').write_text(json.dumps(results,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);main(p.parse_args())
