"""Separate offline GT evaluator for the observation-only geometry study.

No Isaac execution, candidate generation or executable pose correction.
GT handle/collider access is confined to this evaluator, never the RGB-D module.
"""
from pathlib import Path
import argparse,json,hashlib,time
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from native_g1_metric_rgbd_geometry import preserve,projected_seed,save_view,fuse,uncertain_profiles
from visualize_native_g1_gt_observed import sample,distance,stats,pca,angle,project,scan_closing,section
from xarm7_native_planning import NativeModel
from handle_contact_quality import handle_surfaces
from revolute10_fixed import authored_handle_colliders


def first(model,T,base,parts,convex=False):
    key='hypothesis' if convex else 'GT_visual'
    return scan_closing(model,T,base,{key:parts})[1][key]['first']


def brief(pred):
    return {n:None if pred.get(n) is None else pred[n]['aperture_mm'] for n in ['gripper_link1','gripper_link2']}


def main(a):
    a.output.mkdir(parents=True,exist_ok=True);rows=[]
    old_metrics={r['asset_id']:r for r in json.loads(a.reference_metrics.read_text())}
    contacts=json.loads((a.archived/'CONTACT_MISMATCH.json').read_text())
    for asset in ['45671','45600','45606']:
        started=time.perf_counter();physical=a.archived/asset/'physical_01';study=a.study/asset
        folder=next(physical.glob('rgbd/initial*/observed_geometry.json')).parent
        # Observation-only reconstruction is completed before GT is opened.
        saved=preserve(folder);save_view(saved,study/'repaired_single');views=[saved];capture_issues=[]
        for directory in sorted(study.glob('view_*')):
            try:
                seed=projected_seed(directory,saved['points']);v=preserve(directory,seed=seed,plane=saved['plane'])
                save_view(v,directory/'geometry');views.append(v)
            except Exception as e:capture_issues.append(dict(view=directory.name,error=repr(e)))
        points,patch=fuse(views);np.save(study/'fused_observed_points_world.npy',points);patch.export(study/'fused_observed_patches.ply')
        frame=np.asarray(saved['audit']['frame_world']);origin=np.asarray(saved['audit']['origin_world_m'])
        hypotheses=uncertain_profiles(points,frame,origin)
        (study/'MODEL_BOUNDARY.json').write_text(json.dumps(dict(GT_inputs=False,views=[v['folder'] for v in views],observed='calibrated depth points and local neighbor patches',unknown='no watertight backside from RGB-D; three rear priors remain hypothetical',hypotheses=[h['scale'] for h in hypotheses],view_issues=capture_issues),indent=2))
        # GT evaluation starts here. It cannot feed back into reconstruction.
        job=json.loads((a.archived/asset/'job.json').read_text());model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
        export=json.loads((physical/'cooked_initial.json').read_text());selection=job['grasp_geometry_candidates'][0]
        gt,native=handle_surfaces(model,export,authored_handle_colliders(export,selection),selection)
        G=sample(gt,30000);L=pca(G)
        if L@frame[:,0]<0:L=-L
        N=frame[:,2]-L*(frame[:,2]@L);N/=np.linalg.norm(N);W=np.cross(N,L);F=np.column_stack((L,W,N));center=G.mean(0)
        convert=lambda x:(np.asarray(x)-center)@F*1000
        visible=[];depth_flags=[]
        for v in views:
            depth=np.load(Path(v['folder'])/'depth_m.npy');m=v['meta'];_,valid,dz=project(G,np.asarray(m['K']),np.asarray(m['T_world_camera_optical']),depth)
            visible.append(valid&(abs(dz)<=.002))
            depth_flags.append((valid,dz))
        union=np.any(visible,axis=0);old=np.load(folder/'handle_points_world.npy')
        in_any=np.any([v for v,z in depth_flags],axis=0)
        behind_all=in_any&np.all([~v|(z>.002) for v,z in depth_flags],axis=0)
        clouds={'frozen_retained_single':old,'preserved_single':saved['points'],'fused_observed':points};cloud_metrics={}
        for label,P in clouds.items():
            near=cKDTree(P).query(G)[0]<=.002
            cloud_metrics[label]=dict(points=len(P),point_to_GT_visual=stats(distance(gt,P)),
                full_extent_LW_mm=np.ptp(convert(P),axis=0)[:2].tolist(),axis_error_deg=angle(pca(P),L),
                coverage_original_visible=float(near[visible[0]].mean()),coverage_multiview_visible=float(near[union].mean()),
                coverage_full_GT_surface=float(near.mean()))
        plan=json.loads((physical/'visual_plan.json').read_text());T=np.asarray(json.loads((physical/'visual_correction.json').read_text())['T_after']);base=plan['base']
        predictions={label:brief(first(model,T,base,[mesh])) for label,mesh in [('preserved_single_observed',saved['mesh']),('fused_observed',patch)]}
        uncertain=[]
        for h in hypotheses:
            pred=brief(first(model,T,base,h['parts'],True));S=sample(trimesh.util.concatenate(h['parts']),8000)
            supported=cKDTree(points).query(S)[0]<=.002
            uncertain.append(dict(scale=h['scale'],prediction_mm=pred,assumed_surface_fraction=float((~supported).mean()),
                observed_supported_surface_distance=stats(distance(gt,S[supported])) if supported.any() else {},
                inferred_surface_distance=stats(distance(gt,S[~supported])) if (~supported).any() else {}))
        # Bounded existing-candidate sensitivity. No new pose is constructed.
        candidates=json.loads((physical/'visual_planning/closing_geometry_audit.json').read_text());D=np.asarray(json.loads((physical/'visual_correction.json').read_text())['D_visual'])
        chosen=[dict(T=T.tolist(),label='executed_visual_pose')];seen=[]
        for c in candidates:
            key=tuple(c.get(k) for k in ['section','insertion_m','roll','pitch'])
            if key in seen:continue
            seen.append(key);chosen.append(dict(T=(D@np.asarray(c['T'])).tolist(),label=str(key)))
            if len(chosen)>=6:break
        sensitivity=[]
        for c in chosen:
            estimates=[dict(scale=h['scale'],first_mm=brief(first(model,np.asarray(c['T']),base,h['parts'],True))) for h in hypotheses]
            sensitivity.append(dict(candidate=c['label'],source='existing archived candidate; recorded visual D only',hypotheses=estimates))
        actual=old_metrics[asset]['actual_first_contact'];oldfirst={n:float(v['width_m']*1000) for n,v in old_metrics[asset]['stored_first_contact_predictions'].items()}
        row=dict(asset_id=asset,GT_offline_only=True,views=len(views),view_issues=capture_issues,
            captured_camera_angles=[v['meta'].get('azimuth_deg','archived') for v in views],clouds=cloud_metrics,
            GT_visible_fraction_single=float(visible[0].mean()),GT_visible_fraction_multiview=float(union.mean()),
            GT_depth_occluded_in_every_available_view_fraction=float(behind_all.mean()),
            GT_outside_all_valid_depth_fraction=float((~in_any).mean()),
            GT_depth_front_inconsistent_without_visible_match_fraction=float((in_any&~union&~behind_all).mean()),
            GT_extent_LWN_in_plot_frame_mm=np.ptp(convert(gt.vertices),axis=0).tolist(),
            plot_frame_definition='L=GT surface PCA for evaluation; N=observed panel normal orthogonalized to L; W=N cross L',
            unobserved_GT_fraction_after_fusion=float(1-cloud_metrics['fused_observed']['coverage_full_GT_surface']),
            predictions_mm=dict(frozen_loft=oldfirst,**predictions),uncertain_completions=uncertain,
            archived_actual_first_mm={n:None if c is None else c['measured_aperture_mm'] for n,c in actual.items()},
            candidate_sensitivity=sensitivity,source_boundary='RGB-D module accepted no GT; fixed poses used only here; open patches have unknown backsides, missing intersection is not a free-space guarantee',
            wall_s=time.perf_counter()-started)
        rows.append(row);(a.output/'METRICS.json').write_text(json.dumps(rows,indent=2))
        fig,axes=plt.subplots(1,3,figsize=(16,4.5));g=convert(G)
        for ax,(label,P) in zip(axes,clouds.items()):
            q=convert(P);ax.scatter(g[:,0],g[:,1],s=.3,c='#b7b7b7');ax.scatter(q[:,0],q[:,1],s=.6,c='#157bb1');ax.set(title=label.replace('_',' '),xlabel='GT-frame L (mm)',ylabel='GT-frame W (mm)')
        fig.suptitle(asset+' | observed extent; GT gray / depth points blue; GT evaluation only');fig.tight_layout();fig.savefig(a.output/(asset+'_extent.png'),dpi=170);plt.close(fig)
        # GT cross-section versus local observed points and uncertain envelopes.
        poses=model.poses(model.home,base,width=plan['contact_geometry_audit']['robust_contact']['chosen_expected_aperture_m']);X=T@np.linalg.inv(poses['tcp_link']);centers=[(X@poses[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in ['gripper_link1','gripper_link2']];at=np.mean(centers,axis=0)
        fig,axes=plt.subplots(1,3,figsize=(16,5))
        for ax,(label,P) in zip(axes,clouds.items()):
            section(ax,gt,at,L,convert,'#777777','GT visual',linewidth=2)
            for i,m in enumerate(native):section(ax,m,at,L,convert,'#29904b','native collider' if i==0 else None,linestyle='--')
            near=abs((P-at)@L)<.008;q=convert(P[near]);ax.scatter(q[:,1],q[:,2],s=3,c='#147cb1',label='observed +/-8mm L slab')
            if label=='fused_observed':
                for h in hypotheses:
                    for i,m in enumerate(h['parts']):section(ax,m,at,L,convert,'#de8523',str(h['scale'])+'x rear prior' if i==0 else None,alpha=.5)
                ax.plot([],[],color='#de8523',label='assumed rear: 0.75/1/1.25x')
            ax.set(title=label.replace('_',' '),xlabel='W (mm)',ylabel='N (mm)');ax.set_aspect('equal',adjustable='box');ax.legend(fontsize=6)
        fig.suptitle(asset+' | contact cross-section: points observed; orange closures assumed');fig.tight_layout();fig.savefig(a.output/(asset+'_contact.png'),dpi=170);plt.close(fig)
        fig,axes=plt.subplots(1,2,figsize=(12,4));colors=['#777777','#4299c5','#1566a5'];labels=['frozen','preserved single','fused observed']
        for i,(label,P) in enumerate(clouds.items()):
            d=np.sort(distance(gt,P)*1000);axes[0].plot(d,np.linspace(0,1,len(d)),color=colors[i],label=labels[i])
        axes[0].set(xlabel='point-to-GT visual distance (mm)',ylabel='empirical CDF',xlim=(0,4));axes[0].legend()
        records=[dict(scale='frozen',first_mm=oldfirst),dict(scale='single observed',first_mm=predictions['preserved_single_observed']),dict(scale='fused observed',first_mm=predictions['fused_observed'])]+[dict(scale=str(h['scale'])+'x assumed',first_mm=h['prediction_mm']) for h in uncertain]
        for j,n in enumerate(['gripper_link1','gripper_link2']):
            v=[np.nan if r['first_mm'].get(n) is None else r['first_mm'][n] for r in records];axes[1].plot(range(len(v)),v,'o-',label='left' if j==0 else 'right')
            actual_value=row['archived_actual_first_mm'].get(n)
            if actual_value is not None:axes[1].axhline(actual_value,color='#246aa6' if j==0 else '#c2378b',linestyle='--',label='archived actual '+('L' if j==0 else 'R'))
        axes[1].set_xticks(range(len(records)),[r['scale'] for r in records],rotation=25,ha='right');axes[1].set(ylabel='first intersection aperture (mm)',title='Identical frozen visual grasp; 2.059mm sweep');axes[1].legend(fontsize=7);fig.suptitle(asset+' | no new grasp or physics trial');fig.tight_layout();fig.savefig(a.output/(asset+'_metrics.png'),dpi=170);plt.close(fig)
        print('EVALUATED',asset,'views',len(views),'seconds',round(time.perf_counter()-started,1),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--archived',type=Path,required=True);p.add_argument('--study',type=Path,required=True);p.add_argument('--reference-metrics',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
