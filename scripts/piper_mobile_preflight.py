"""Same-scene PiPER fixed-base test and generic stop/reposition/lock recovery."""
import argparse,json,sys,time
from datetime import datetime,timedelta
from zoneinfo import ZoneInfo
from pathlib import Path
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.model import Model


def variants(folder,report,maximum,model):
    capture=np.load(folder/'capture.npz');mask=np.load(folder/'handle_mask.npy')
    d=capture['depth_m'];K=capture['K'];v,u=np.where(mask&(d>0)&np.isfinite(d));z=d[v,u]
    p=np.column_stack(((u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z))
    C=capture['T_B_C'];points=p@C[:3,:3].T+C[:3,3];tree=cKDTree(points)
    model.target_points=points;model.target_tree=tree
    pad=model.pads['gripper_link1'];inset=-float(pad[:,1].mean())
    output=[]
    for source in report['candidates'][:6]:
        T=np.asarray(source['T_B_TCP']);_,indices=tree.query(T[:3,3],k=min(512,len(points)))
        local=points[np.atleast_1d(indices)];center=(np.quantile(local,.1,axis=0)+np.quantile(local,.9,axis=0))/2
        _,_,V=np.linalg.svd(local-center,full_matrices=False);axis=V[0]
        configurations=[(0,twist,depth,pitch,symmetry) for pitch in [0,-20,20] for symmetry in [0,180] for twist in [0,-15,15] for depth in [0,-.005,.005]]
        configurations += [(slide,0,0,0,symmetry) for slide in [-.006,.006] for symmetry in [0,180]]
        for slide,twist,depth,pitch,symmetry in configurations:
                    out=T.copy();out[:3,:3]=Rotation.from_rotvec(axis*np.deg2rad(twist)).as_matrix()@T[:3,:3]
                    out[:3,:3]=out[:3,:3]@Rotation.from_euler('yz',[pitch,symmetry],degrees=True).as_matrix()
                    # TCP is the distal plane; inset is derived from official mesh.
                    out[:3,3]=center+slide*axis+(inset+depth)*out[:3,2]
                    if any(np.linalg.norm(old['T']-out)<1e-5 for old in output):continue
                    output.append({'raw_rank':source['rank'],'slide_m':slide,'twist_deg':twist,'pitch_deg':pitch,'parallel_jaw_symmetry_deg':symmetry,'depth_delta_m':depth,'local_anchor':center.tolist(),'T':out})
    return output if len(output)<=maximum else [output[i] for i in np.linspace(0,len(output)-1,maximum,dtype=int)]


def path(model,item,base,goal=22,starts=5):
    T=item['T'];q=model.ik(T,base,seed=np.asarray(item['seed_q']) if 'seed_q' in item else None,starts=starts)
    row={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in item.items()};row['base']=list(map(float,base))
    if q is None:return {**row,'status':'NO_IK'}
    valid,reason,_=model.check(q,base)
    if not valid:return {**row,'status':reason}
    pre=T.copy();pre[:3,3]-=.04*T[:3,2];qp=model.ik(pre,base,q,starts=3)
    if qp is None:return {**row,'status':'NO_PREGRASP_IK'}
    approach=[];current=qp
    for a in np.linspace(0,1,21):
        W=T.copy();W[:3,3]=(1-a)*pre[:3,3]+a*T[:3,3];current=model.ik(W,base,current,starts=1)
        if current is None:return {**row,'status':'NO_APPROACH_IK'}
        ok,why,_=model.check(current,base)
        if not ok:return {**row,'status':'APPROACH_'+why}
        approach.append(current.tolist())
    preplan=model.joint_plan(model.home,qp,base)
    if preplan is None:return {**row,'status':'NO_PREGRASP_PLAN'}
    row.update(q_pre=qp.tolist(),preplan=preplan,q_grasp=current.tolist(),approach=approach,arc=[],status='GRASP_PATH_PLANNED')
    minimum=model.margin(current);clearance=float('inf')
    for degree in np.arange(.5,goal+.01,.5):
        angle=np.deg2rad(degree);target=model.door_target(T,angle);end=model.ik(target,base,current,starts=3)
        if end is None:row.update(status='ARC_NO_IK',failed_angle_deg=float(degree));break
        if np.max(np.abs(end-current))>.4:row.update(status='IK_BRANCH_JUMP',failed_angle_deg=float(degree));break
        for fraction in np.linspace(0,1,6):
            qr=current+fraction*(end-current);qa=np.deg2rad(degree-.5+fraction*.5)
            ok,why,d=model.check(qr,base,qa,clearance=True)
            if not ok:row.update(status='ARC_'+why,failed_angle_deg=float(degree));break
            minimum=min(minimum,model.margin(qr));clearance=min(clearance,d)
        if not ok:break
        row['arc'].append({'angle_deg':float(degree),'q':end.tolist(),'margin_rad':model.margin(end)})
        current=end
    else:row['status']='FULL_PATH_PLANNED'
    row.update(max_planned_angle_deg=row['arc'][-1]['angle_deg'] if row['arc'] else 0.,minimum_joint_margin_rad=minimum,minimum_collision_clearance_m=clearance if np.isfinite(clearance) else None)
    return row


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--asset-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--recover-from',type=Path);p.add_argument('--refine-from',type=Path);p.add_argument('--center-from',type=Path);p.add_argument('--max-candidates',type=int,default=162);p.add_argument('--goal-deg',type=float,default=22);p.add_argument('--deadline-shanghai')
    a=p.parse_args();zone=ZoneInfo('Asia/Shanghai');now=datetime.now(zone);deadline=datetime.fromisoformat(a.deadline_shanghai) if a.deadline_shanghai else now.replace(hour=5,minute=0,second=0,microsecond=0)
    if deadline.tzinfo is None:deadline=deadline.replace(tzinfo=zone)
    if not a.deadline_shanghai and deadline<=now:deadline+=timedelta(days=1)
    if now>=deadline:raise RuntimeError('experiment deadline passed')
    a.output.mkdir(parents=True,exist_ok=True);source=json.loads((a.source/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',a.asset_root,source);model.deadline_timestamp=deadline.timestamp()
    items=variants(a.source,source,a.max_candidates,model);np.save(a.output/'target_points.npy',model.target_points);base=np.asarray(source['robot_base_pose']);bases=[base];recovery=False
    if a.refine_from:
        previous=json.loads(a.refine_from.read_text());seeds=[r for r in previous['rows'] if r['status']=='FULL_PATH_PLANNED'];items=[]
        _,_,V=np.linalg.svd(model.target_points-model.target_points.mean(0),full_matrices=False);axis=V[0];pad=model.pads['gripper_link1'];stride=float(pad[1,0]-pad[0,0]);inset=-float(pad[:,1].mean());depth_step=float(pad[1,1]-pad[0,1])/3
        for seed in seeds:
            T=np.array(seed['T']);centre=T[:3,3]-inset*T[:3,2]
            for slide in [0,-.5,.5,-1,1,-1.5,1.5]:
                query=centre+slide*stride*axis;distance,indices=model.target_tree.query(query,k=min(512,len(model.target_points)))
                if distance[0]>.012:continue
                local=model.target_points[indices];anchor=(np.quantile(local,.1,axis=0)+np.quantile(local,.9,axis=0))/2
                for depth in [0,-1,1]:
                    W=T.copy();W[:3,3]=anchor+(inset+depth*depth_step)*W[:3,2]
                    items.append({'raw_rank':seed['raw_rank'],'T':W,'seed_q':seed['q_grasp'],'local_anchor':anchor.tolist(),'slide_m':slide*stride,'depth_delta_m':depth*depth_step,'refinement':'observed region PCA slide; step sizes derived from official pad geometry','pitch_deg':seed['pitch_deg'],'parallel_jaw_symmetry_deg':seed['parallel_jaw_symmetry_deg']})
    if a.center_from:
        previous=json.loads(a.center_from.read_text());seeds=sorted([r for r in previous['rows'] if r['status']=='FULL_PATH_PLANNED'],key=lambda r:r['minimum_joint_margin_rad'],reverse=True)[:max(1,a.max_candidates//5)];items=[]
        pad=model.pads['gripper_link1'];step=float(pad[1,1]-pad[0,1])/2
        for seed in seeds:
            for shift in [0,-1,1,-2,2]:
                W=np.asarray(seed['T']).copy();W[:3,3]+=shift*step*W[:3,1]
                items.append({'raw_rank':seed['raw_rank'],'T':W,'seed_q':seed['q_grasp'],'closing_center_shift_m':shift*step,'refinement':'symmetric jaw centering search; step derived from official distal pad length','pitch_deg':seed['pitch_deg'],'parallel_jaw_symmetry_deg':seed['parallel_jaw_symmetry_deg']})
    if a.recover_from:
        fixed=json.loads(a.recover_from.read_text());reason=fixed['best']['status'] if fixed.get('best') else 'NO_IK'
        if (fixed.get('best') or {}).get('status')=='FULL_PATH_PLANNED':raise RuntimeError('fixed base feasible; recovery is not authorized by a kinematic failure')
        if reason in ('BAD_CONTACT','CONTACT_LOSS','NON_PAD_CONTACT'):raise RuntimeError('contact failure alone must not trigger base recovery')
        recovery=True
        bases=[np.array([base[0]+dx,base[1]+dy,base[2],base[3]+yaw]) for dy in [-.25,-.15,0,.05,.08] for dx in [.15,0,-.15,.25,-.25] for yaw in [0,-30,30,-60,60] if -0.76<base[2]<0 and base[1]+dy<=-.45-.01]
    report={'schema':'piper_mobile_door/v1','mode':'mobile_recovery' if recovery else 'fixed','source':str(a.source),'asset_root':str(a.asset_root),'model':model.fingerprint(),'unchanged_asset_and_contact_parameters':True,'rows':[],'best':None}
    for index,B in enumerate(bases):
        # Recovery ranks whole paths, not grasp counts. Coarse stage uses 9 poses.
        candidates=items if not recovery else items[::max(1,len(items)//9)][:9]
        for item in candidates:
            if datetime.now(zone)>=deadline:
                report['status']='CUTOFF_05_00';(a.output/'report.json').write_text(json.dumps(report,indent=2));return
            row=path(model,item,B,a.goal_deg,starts=3 if recovery else 5);report['rows'].append(row)
            if 'q_grasp' in row:
                def score(r):return (r['max_planned_angle_deg'],r['minimum_joint_margin_rad'],r.get('minimum_collision_clearance_m') or 0)
                if report['best'] is None or score(row)>score(report['best']):report['best']=row
            report['counts']=dict(Counter(r['status'] for r in report['rows']));(a.output/'report.json').write_text(json.dumps(report,indent=2))
        print(index,len(bases),'base',B.tolist(),'counts',report['counts'],'best',None if report['best'] is None else [report['best']['max_planned_angle_deg'],report['best']['minimum_joint_margin_rad']],flush=True)
    if report['best'] is None:report['status']='NO_SAFE_GRASP_PATH'
    else:report['status']=report['best']['status']
    (a.output/'report.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
