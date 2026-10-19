"""Bounded handle-local search; every survivor still requires actual closure."""
import argparse,json,sys,time
from pathlib import Path
from collections import Counter
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from scipy.stats import qmc
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model
from piper_mobile_demo.owned_scene import OwnedFingerScene


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['source','asset-root','reference-plan','reference-trial','native-export','association','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--ownership',type=Path,default=ROOT/'config/piper_contact_ownership.json');p.add_argument('--samples',type=int,default=160);p.add_argument('--handle-regions',type=int,default=0);p.add_argument('--max-planned-trials',type=int,default=96);p.add_argument('--variant-prefix',default='local');p.add_argument('--resume',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    source=json.loads((a.source/'report.json').read_text());old=json.loads(a.reference_plan.read_text());seeds=[r for r in old['rows'] if r.get('q_grasp')]
    # Fixed base comes from the previously executed trial, never searched here.
    base=np.asarray(seeds[0]['base']);assert all(np.array_equal(base,r['base']) for r in seeds)
    model=Model(ROOT/'config/piper.urdf',a.asset_root,source)
    targets=np.load(a.reference_plan.parent/'target_points.npy');np.save(a.output/'target_points.npy',targets);tree=cKDTree(targets)
    allowed=json.loads(a.association.read_text())['allowed_pad_targets'];owner=OwnedFingerScene(ROOT,a.native_export,a.ownership,allowed,model.manifest['moving_link'])
    # Reference pad centre is computed from the official pad surface in TCP.
    poses=model.poses(model.home,base,width=.04);inv=np.linalg.inv(poses['tcp_link']);centres=[]
    for n in ['gripper_link1','gripper_link2']:
        pts=np.asarray(owner.manifest['fingers'][n]['pad_vertices']);centres.append((inv@poses[n]@np.r_[pts.mean(0),1])[:3])
    pad_centre=np.mean(centres,axis=0)
    pivot=json.loads(a.reference_trial.read_text())['selected'];T0=np.asarray(pivot['T']);point=(T0@np.r_[pad_centre,1])[:3]
    # A tiny surface patch often has its first PCA axis across the handle.
    # Size the local neighborhood from the unchanged official pad geometry,
    # rather than a fixed point count or an object-specific dimension.
    pad_vertices=np.asarray(owner.manifest['fingers']['gripper_link1']['pad_vertices'])
    frame_radius=2*np.linalg.norm(np.ptp(pad_vertices,axis=0))
    indices=tree.query_ball_point(point,frame_radius)
    if len(indices)<64:_,indices=tree.query(point,k=min(512,len(targets)))
    local=targets[indices];anchor=np.median(local,axis=0);_,singular,V=np.linalg.svd(local-anchor,full_matrices=False);axis=V[0]
    if axis@T0[:3,0]<0:axis=-axis
    normal=T0[:3,2]-axis*(axis@T0[:3,2]);normal/=np.linalg.norm(normal);frame=np.column_stack((axis,np.cross(normal,axis),normal))
    offsets=[(0,0,0),(0,0,-.003),(0,0,.003),(0,0,-.006),(0,0,.006),(0,-.003,0),(0,.003,0),(-.006,0,0),(.006,0,0)]
    settings=[(np.array(x),np.zeros(3)) for x in offsets]
    for i in range(3):
        for deg in [-12,-6,6,12]:
            rot=np.zeros(3);rot[i]=deg
            for offset in offsets[:5]:settings.append((np.asarray(offset),rot))
    settings.extend(((u[:3]*2-1)*.008,(u[3:]*2-1)*12) for u in qmc.Halton(6,scramble=False).random(a.samples))
    regions=[(0,point,frame)]
    if a.handle_regions:
        regions=[];coordinates=(targets-anchor)@axis
        for i,quantile in enumerate(np.linspace(.05,.95,a.handle_regions)):
            coordinate=np.quantile(coordinates,quantile)
            section=targets[abs(coordinates-coordinate)<np.ptp(pad_vertices[:,0])/2]
            if len(section)<64:continue
            center=np.median(section,axis=0);rp=center+point-anchor
            neighbors=targets[tree.query_ball_point(rp,frame_radius)]
            if len(neighbors)<64:continue
            _,_,v=np.linalg.svd(neighbors-np.median(neighbors,axis=0),full_matrices=False);tangent=v[0]
            if tangent@axis<0:tangent=-tangent
            n=T0[:3,2]-tangent*(tangent@T0[:3,2]);n/=np.linalg.norm(n)
            regions.append((i,rp,np.column_stack((tangent,np.cross(n,tangent),n))))
    settings=[(i,rp,rf,offset,rpy) for i,rp,rf in regions for offset,rpy in settings]
    report={'mode':'fixed','base':base.tolist(),'source':str(a.source),'asset_root':str(a.asset_root),'ownership_sha256':__import__('hashlib').sha256(a.ownership.read_bytes()).hexdigest(),'native_export':str(a.native_export),'search_frame':frame.tolist(),'frame_neighborhood_radius_m':frame_radius,'frame_point_count':len(local),'frame_singular_values':singular.tolist(),'pad_centre_tcp':pad_centre.tolist(),'bounds':{'translation_handle_m':.008,'rpy_handle_deg':12},'raw_perception_changed':False,'closure_rule':'predicted width is diagnostic only; actual PhysX closure mandatory','rows':[],'best':None}
    if a.resume:
        previous=json.loads((a.output/'report.json').read_text())
        assert previous['base']==base.tolist() and previous['ownership_sha256']==report['ownership_sha256'] and previous['native_export']==str(a.native_export)
        assert np.allclose(previous['search_frame'],frame,atol=1e-12), 'search frame changed; use a new output directory'
        report=previous;settings=[]
    seen=set()
    report['handle_regions']=[{'region':i,'pad_anchor_world':rp.tolist(),'frame':rf.tolist()} for i,rp,rf in regions]
    for index,(region,rp,rf,offset,rpy) in enumerate(settings):
        T=T0.copy();T[:3,:3]=rf@Rotation.from_euler('xyz',rpy,degrees=True).as_matrix()@frame.T@T0[:3,:3];T[:3,3]=rp+rf@offset-T[:3,:3]@pad_centre
        sig=tuple(np.round(T[:3].ravel(),8))
        if sig in seen:continue
        seen.add(sig);row={'variant':f'{a.variant_prefix}_{index:04d}','region':region,'T':T.tolist(),'base':base.tolist(),'raw_rank':pivot['raw_rank'],'offset_handle_m':offset.tolist(),'rpy_handle_deg':rpy.tolist(),'pad_anchor_world':rp.tolist(),'raw_translation_delta_m':(T[:3,3]-T0[:3,3]).tolist(),'raw_rotation_delta_deg':float(np.rad2deg(Rotation.from_matrix(T[:3,:3]@T0[:3,:3].T).magnitude()))}
        q=model.ik(T,base,pivot['q_grasp'],starts=5)
        if q is None:row['status']='NO_IK'
        else:
            safe,why=owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False)
            if not safe:row['status']=why
            else:
                pre=T.copy();pre[:3,3]-=.04*T[:3,2];qp=model.ik(pre,base,q,starts=3);approach=[];current=qp
                if qp is None:row['status']='NO_PREGRASP_IK'
                else:
                    for f in np.linspace(0,1,21):
                        target=T.copy();target[:3,3]=(1-f)*pre[:3,3]+f*T[:3,3];current=model.ik(target,base,current,starts=1)
                        if current is None:row['status']='NO_APPROACH_IK';break
                        safe,why=owner.check_robot(model,current,base,finger_q=[.05,-.05],allow_pad=False)
                        if not safe:row['status']='APPROACH_'+why;break
                        approach.append(current.tolist())
                    else:
                        row.update(status='APPROACH_PLANNED_PENDING_REAL_CLOSURE',q_pre=qp.tolist(),q_grasp=current.tolist(),approach=approach,minimum_joint_margin_rad=min(model.margin(np.asarray(x)) for x in approach))
                        # Do not interpret this aperture prediction as closure.
                        target_vertices=np.vstack([x.vertices@owner.moving_reference[:3,:3].T+owner.moving_reference[:3,3] for x in owner.raw_scene if x.path in allowed]);local_target=(target_vertices-T[:3,3])@T[:3,:3]
                        width=float(np.ptp(local_target[:,1]));ok,reason=owner.check_robot(model,current,base,finger_q=[width/2,-width/2])
                        row.update(predicted_width_m=width,prediction_only_geometry_safe=ok,prediction_only_reason=reason)
        report['rows'].append(row);report['counts']=dict(Counter(x['status'] for x in report['rows']));(a.output/'report.json').write_text(json.dumps(report,indent=2));print(index,row['status'],flush=True)
    for row in report['rows']:
        if 'q_grasp' in row:
            # Predicted aperture is diagnostic only, including in ranking.
            row['score']=np.linalg.norm(row['offset_handle_m'])*20+np.linalg.norm(row['rpy_handle_deg'])*.002+.001/max(.05,row['minimum_joint_margin_rad'])
    report['closure_rule']='predicted width is diagnostic only; actual PhysX closure mandatory'
    eligible=sorted([r for r in report['rows'] if r['status']=='APPROACH_PLANNED_PENDING_REAL_CLOSURE'],key=lambda r:r['score'])
    if a.handle_regions:
        groups=[[r for r in eligible if r.get('region')==i] for i,_,_ in regions]
        eligible=[group[k] for k in range(max((len(g) for g in groups),default=0)) for group in groups if k<len(group)]
    # Plan and collision-check the home->pregrasp route only for leading poses.
    reference_path=pivot['preplan'];reference_valid=all(owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False)[0] for x,y in zip(reference_path,reference_path[1:]) for q in np.linspace(x,y,max(2,int(np.ceil(np.max(np.abs(np.asarray(y)-x))/.025))+1)))
    trials=[];report['planning_complete']=False
    for row in eligible[:a.max_planned_trials]:
        plan=row.get('preplan')
        if plan is None:
            local=model.joint_plan(np.asarray(reference_path[-1]),np.asarray(row['q_pre']),base) if reference_valid else None
            plan=reference_path+local[1:] if local is not None else model.joint_plan(model.home,np.asarray(row['q_pre']),base)
        if plan is None:row['status']='NO_PREGRASP_PLAN';continue
        shortcut=[plan[0]];i=0
        while i<len(plan)-1:
            for j in range(len(plan)-1,i,-1):
                if all(owner.check_robot(model,q,base,finger_q=[.05,-.05],allow_pad=False)[0] for q in np.linspace(plan[i],plan[j],max(2,int(np.ceil(np.max(np.abs(np.asarray(plan[j])-plan[i]))/.025))+1))):break
            shortcut.append(plan[j]);i=j
        row['preplan']=shortcut;trials.append(row)
        report['trial_candidates']=trials;report['best']=trials[0];(a.output/'report.json').write_text(json.dumps(report,indent=2))
    report['trial_candidates']=trials;report['best']=trials[0] if trials else None;report['planning_complete']=True;(a.output/'report.json').write_text(json.dumps(report,indent=2));print('actual closure queue',len(trials),flush=True)

if __name__=='__main__':main()
