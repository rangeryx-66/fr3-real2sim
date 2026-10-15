"""Separate offline GT/coverage/contact evaluator; never imported by runner."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from visualize_native_g1_gt_observed import sample,distance,stats,project,pca,angle
from evaluate_native_g1_multiview_predictions import first
from xarm7_native_planning import NativeModel
from handle_contact_quality import handle_surfaces
from revolute10_fixed import authored_handle_colliders

def evaluate(run,output):
    output.mkdir(parents=True,exist_ok=True);start=time.perf_counter()
    job=json.loads((run.parent/'job.json').read_text())
    source=json.loads((Path(job['source'])/'report.json').read_text());model=NativeModel(None,job['asset_root'],source)
    observation=json.loads((run/'WRIST_OBSERVATION_RESULT.json').read_text()) if (run/'WRIST_OBSERVATION_RESULT.json').exists() else dict(wall_s=None,geometry=None)
    folders=[Path(x) for x in observation['views']] if observation.get('views') else sorted(p.parent for p in (run/'rgbd').glob('*/camera.json'))
    # All executable poses/choices exist before opening any handle GT surface.
    selection=job['grasp_geometry_candidates'][0];export=json.loads((run/'cooked_initial.json').read_text())
    gt,native=handle_surfaces(model,export,authored_handle_colliders(export,selection),selection)
    G=sample(gt,30000);axis=pca(G);rows=[];visible=[]
    for folder in folders:
        meta=json.loads((folder/'camera.json').read_text());d=np.load(folder/'depth_m.npy');K=np.asarray(meta['K']);C=np.asarray(meta['T_world_camera_optical'])
        xy,valid,dz=project(G,K,C,d);visible.append(valid&(abs(dz)<=.002))
        mask=np.load(folder/'robot_self_exclusion.npy');inside=(xy[:,0]>=0)&(xy[:,0]<d.shape[1])&(xy[:,1]>=0)&(xy[:,1]<d.shape[0]);self_hidden=np.zeros(len(G),bool);self_hidden[inside]=mask[xy[inside,1],xy[inside,0]]
        row=dict(view=folder.name,sim_s=meta['t_sim'],position_world_m=C[:3,3].tolist(),robot_q=meta['robot_q'],GT_surface_depth_visible_fraction=float(visible[-1].mean()),GT_surface_robot_silhouette_fraction=float(self_hidden.mean()),self_mask_image_fraction=meta['robot_self_mask_fraction'],FK_camera_translation_error_mm=meta['FK_camera_translation_error_m']*1000,FK_camera_rotation_error_deg=meta['FK_camera_rotation_error_deg'],rigid_camera_translation_error_mm=meta['rigid_camera_translation_error_m']*1000)
        detection=folder/'visual_detection_attempts.json'
        if detection.exists():
            row['detection_attempts']=[{k:a.get(k) for k in ['kind','raw_max','confidence_retained_scores','mask_filter','accepted','wall_s']} for a in json.loads(detection.read_text())['attempts']]
        rows.append(row)
    old=ROOT/'results/native_g1_multiview_skill_20261015/45600/physical_01'
    scene_visible=[]
    for folder in [old/'rgbd/initial']+sorted((old/'rgbd').glob('multi_*')):
        if not (folder/'camera.json').exists():continue
        meta=json.loads((folder/'camera.json').read_text());depth=np.load(folder/'depth_m.npy')
        _,valid,dz=project(G,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),depth)
        scene_visible.append(valid&(abs(dz)<=.002))
    union=np.any(visible,axis=0);clouds={}
    for label,path in [('wrist_single',run/'wrist_geometry/single/points_world.npy'),('wrist_multi',run/'wrist_geometry/points_world.npy'),('scene_multi_baseline',old/'multiview_geometry/points_world.npy')]:
        # metric.save_view names points_world.npy; retain explicit lookup.
        if not path.exists():
            options=list(path.parent.glob('*.npy'));options=[p for p in options if 'point' in p.name]
            if not options:continue
            path=options[0]
        P=np.load(path);covered=cKDTree(P).query(G)[0]<=.002
        clouds[label]=dict(points=len(P),observed_to_GT=stats(distance(gt,P)),full_GT_coverage_2mm=float(covered.mean()),wrist_visible_GT_coverage_2mm=float(covered[union].mean()) if union.any() else None,principal_axis_error_deg=angle(pca(P),axis),extent_along_GT_axis_mm=float(np.ptp(P@axis)*1000))
    predictions={}
    if (run/'visual_plan.json').exists():
        plan=json.loads((run/'visual_plan.json').read_text());T=np.asarray(plan['T']);base=plan['base'];geo=run/'wrist_geometry'
        measured=trimesh.load(geo/'observed_patches.ply',process=False)
        predictions['observed_only']=first(model,T,base,[measured])
        for i,scale in enumerate([.75,1.,1.25]):
            rear=trimesh.load(geo/('uncertain_rear_%d.ply'%i),process=False);predictions[str(scale)]=first(model,T,base,[measured,rear])
        predictions['GT_visual_offline_only']=first(model,T,base,[gt]);predictions['GT_native_offline_only']=first(model,T,base,native,True)
    cameras=np.array([r['position_world_m'] for r in rows]);pair=[]
    for i in range(len(folders)):
        for j in range(i):
            Ci=np.asarray(json.loads((folders[i]/'camera.json').read_text())['T_world_camera_optical']);Cj=np.asarray(json.loads((folders[j]/'camera.json').read_text())['T_world_camera_optical'])
            pair.append(dict(views=[folders[j].name,folders[i].name],baseline_mm=float(np.linalg.norm(Ci[:3,3]-Cj[:3,3])*1000),optical_axis_angle_deg=float(np.rad2deg(np.arccos(np.clip(Ci[:3,2]@Cj[:3,2],-1,1))))))
    result=dict(asset='45600',run=str(run),views=rows,pairs=pair,clouds=clouds,predicted_first_contacts=predictions,wrist_union_GT_depth_visible_fraction=float(union.mean()),scene_baseline_union_GT_depth_visible_fraction=float(np.any(scene_visible,axis=0).mean()) if scene_visible else None,scene_baseline_views_evaluated=len(scene_visible),observation_wall_s=observation['wall_s'],evaluation_wall_s=time.perf_counter()-start,GT_is_offline_evaluation_only=True,missing_backside_is_not_sensor_error=True)
    (output/'GEOMETRY_COMPARISON.json').write_text(json.dumps(result,indent=2))
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    axes[0].plot(cameras[:,1],cameras[:,2],'o-')
    for i,r in enumerate(rows):axes[0].annotate(str(i),cameras[i,1:3])
    axes[0].set(xlabel='World y (m)',ylabel='World z (m)',title='Actually reached wrist camera positions');axes[0].axis('equal')
    if 'wrist_multi' in clouds:
        labels=list(clouds);axes[1].bar(range(len(labels)),[100*clouds[k]['full_GT_coverage_2mm'] for k in labels]);axes[1].set_xticks(range(len(labels)),[k.replace('_','\n') for k in labels]);axes[1].set(ylabel='GT surface covered within 2 mm (%)',title='Offline evaluation; hidden regions remain unknown',ylim=(0,100))
    else:
        axes[1].bar(range(len(rows)),[100*r['GT_surface_depth_visible_fraction'] for r in rows]);axes[1].set_xticks(range(len(rows)),[str(i) for i in range(len(rows))]);axes[1].set(xlabel='Reached view index',ylabel='GT surface depth-visible (%)',title='Raw wrist visibility only; no accepted handle geometry',ylim=(0,100))
    fig.tight_layout();fig.savefig(output/'coverage_and_views.png',dpi=160);plt.close(fig)
    print('WRIST_EVALUATED',json.dumps(dict(views=len(rows),clouds=clouds)),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();evaluate(a.run,a.output)
