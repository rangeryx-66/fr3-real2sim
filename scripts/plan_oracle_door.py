"""Bounded development planner: handle oracle ROI, shared grasp/base/branch search.

Original colliders and physical guards are unchanged. Closing-contact estimates
rank candidates; they do not add runtime assertions or certify a physical grasp.
"""
import argparse,copy,json,time
from pathlib import Path
import numpy as np
import fcl
from scipy.spatial.transform import Rotation
from revolute10_fixed import (ROOT,Model,PinkIK,authored_handle_colliders,
    physical_scene_class,at_base,candidate_bases,calibration,camera_clearance)
from piper_mobile_demo.model import obj
from piper_mobile_demo.owned_scene import intersects


def closing_quality(model,sc,T,base,width):
    """Audit the native solids throughout closure using the existing pad region."""
    audit=[];first={};root_contacts=0;other=None
    for w in np.linspace(.1,max(0.,width-.002),25):
        P=model.poses(model.home,base,width=float(w));D=T@np.linalg.inv(P['tcp_link'])
        P={n:D@X for n,X in P.items()}
        for r in sc.robot:
            if r.body not in ('gripper_link1','gripper_link2'):continue
            r.place(P[r.body])
            for s in sc.scene:
                s.place(sc.moving_reference if s.body==sc.moving_link else np.eye(4))
                if not intersects(r,s):continue
                if s.path not in sc.allowed:other=s.path;continue
                result=fcl.CollisionResult();fcl.collide(r.object,s.object,fcl.CollisionRequest(num_max_contacts=32,enable_contact=True),result)
                for c in result.contacts:
                    local=(np.linalg.inv(P[r.body])@np.r_[c.pos,1])[:3]
                    bounds=sc.jaw_bounds[r.body]
                    distal=bool(local[1]>=model.pads[r.body][0,1] and local[2]<=(bounds[0,2]+bounds[1,2])/2)
                    item={'finger':r.body,'width_m':float(w),'local_contact_m':local.tolist(),'existing_pad_region':distal}
                    audit.append(item)
                    if r.body not in first:first[r.body]=item
                    if not distal:root_contacts+=1
        if len(first)==2:break # first physical engagement, not penetration at zero aperture
    pad=sum(int(v['existing_pad_region']) for v in first.values())
    return {'first_contacts':first,'pad_fingers':pad,'root_contact_count':root_contacts,
            'non_handle_closing_collision':other,'samples':audit,
            'rank':float(pad)-float(root_contacts>0)*2-float(other is not None)*2}


def plan(job,output,wall_s=600):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);started=time.time()
    source=json.loads((Path(job['source'])/'report.json').read_text())
    model=Model(ROOT/'config/piper.urdf',job['asset_root'],source);solver=PinkIK(model.urdf,model.home);model.ik=solver.solve
    export=json.loads(Path(job['collision_export']).read_text());initial=source['robot_base_pose'];cal=calibration(job['camera_calibration'])
    selection=job['grasp_geometry_candidates'][0];allowed=authored_handle_colliders(export,selection)
    sc0=physical_scene_class(ROOT)(export,model,allowed);goal=float(job['benchmark_goal_rad'])
    joint=model.manifest['joint_name'];link=model.manifest['moving_link'];M0=model.asset_T@model.asset.root_to_link(link,{joint:0.})
    def delta(s):return model.asset_T@model.asset.root_to_link(link,{joint:float(s)})@np.linalg.inv(M0)
    width=min(.1,float(selection['dimensions_m'][1]));P=model.poses(model.home,initial,width=width)
    offset=np.mean([(np.linalg.inv(P['tcp_link'])@P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in ('gripper_link1','gripper_link2')],axis=0)
    normal=model.asset_T[:3,:3]@np.array(selection['outward_normal_root']);axis=model.asset_T[:3,:3]@np.array(selection['axis_root']);normal-=axis*(normal@axis);normal/=np.linalg.norm(normal)
    depth=float(np.diff(model.pads['gripper_link1'][:,1])[0])/4
    geometries=[]
    for section in sorted(selection['sections'],key=lambda s:abs(s['fraction'])):
        anchor=(model.asset_T@np.r_[section['anchor_root_m'],1])[:3]
        E=np.eye(4);E[:3,:3]=np.column_stack((axis,np.cross(-normal,axis),-normal));E[:3,3]=anchor-E[:3,:3]@offset
        for insertion in (0.,-depth,depth):
            for roll in (0.,180.):
                T=E.copy();T[:3,:3]=E[:3,:3]@Rotation.from_euler('z',roll,degrees=True).as_matrix();T[:3,3]=anchor+normal*insertion-T[:3,:3]@offset
                quality=closing_quality(model,sc0,T,initial,width)
                geometries.append({'T':T,'section':section['fraction'],'insertion_m':insertion,'roll':roll,'contact':quality})
    geometries.sort(key=lambda g:g['contact']['rank'],reverse=True)
    (output/'closing_geometry_audit.json').write_text(json.dumps([{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in g.items()} for g in geometries],indent=2))
    anchor=(model.asset_T@np.r_[selection['sections'][0]['anchor_root_m'],1])[:3]
    bases=[initial]+candidate_bases(initial,{'anchor_world_m':anchor.tolist(),'outward_normal_world':normal.tolist()},job['wrist_experiment']['mobile_scan'])
    audit=[];ranked=[]
    def arc(T,base,sc,seed,spacing=3):
        path=[];reason=None
        for s in np.linspace(0,goal,int(np.ceil(np.rad2deg(goal)/spacing))+1):
            q=model.ik(delta(s)@T,base,seed)
            if q is None:reason='NO_CONTINUOUS_PINK_IK';break
            ok,reason=sc.check(model.poses(q,base,width=width),delta(s)@sc.moving_reference,True)
            if ok:ok,reason=camera_clearance(sc,model.poses(q,base,width=width),cal)
            if not ok:break
            path.append({'state':float(s),'q':q.tolist(),'T_tcp':(delta(s)@T).tolist(),'margin_rad':model.margin(q)});seed=q
        return path,reason
    # Station-major order covers multiple contact sections at each station.
    for bi,base in enumerate(bases):
        if time.time()-started>wall_s*.72:break
        sc=physical_scene_class(ROOT)(at_base(export,initial,base),model,allowed)
        for gi,g in enumerate(geometries):
            if time.time()-started>wall_s*.72:break
            q=model.ik(g['T'],base,starts=4)
            if q is None:continue
            branches=[q];flip=q.copy();flip[3]+=np.pi if q[3]<0 else -np.pi;flip[4]*=-1;flip[5]+=np.pi if q[5]<0 else -np.pi
            qf=model.ik(g['T'],base,flip)
            if qf is not None:branches.append(qf)
            for branch,seed in enumerate(branches):
                path,reason=arc(g['T'],base,sc,seed)
                e={'base':base,'geometry_index':gi,'branch':branch,'reached_rad':path[-1]['state'] if path else 0.,'min_arc_margin_rad':min((p['margin_rad'] for p in path),default=0.),'blocker':reason,'contact_rank':g['contact']['rank']};audit.append(e)
                if path:ranked.append((e,path,g))
        if bi%5==0:print('SEARCH',model.manifest['asset_id'],bi,len(audit),time.time()-started,flush=True)
    # Contact quality first, then achieved interval and worst margin. No larger
    # safety margin is imposed; the original 0.05 rad bound remains the only one.
    ranked.sort(key=lambda r:(r[0]['contact_rank'],r[0]['reached_rad'],r[0]['min_arc_margin_rad']),reverse=True)
    best=None
    for e,path,g in ranked[:18]:
        if time.time()-started>wall_s:break
        base=e['base'];T=g['T'];sc=physical_scene_class(ROOT)(at_base(export,initial,base),model,allowed)
        fine,reason=arc(T,base,sc,np.array(path[0]['q']),spacing=1)
        if not fine:continue
        pre=T.copy();pre[:3,3]-=.04*T[:3,2];seed=model.ik(pre,base,np.array(fine[0]['q']),starts=2)
        if seed is None:continue
        view=copy.copy(model)
        def free_check(q,b):
            P=model.poses(q,b,width=.1);ok,why=sc.check(P,sc.moving_reference,False)
            if ok:ok,why=camera_clearance(sc,P,cal)
            return ok,why,None
        view.check=free_check;free=Model.joint_plan(view,model.home,seed,base,iterations=600)
        if free is None:continue
        approach=[]
        for f in np.linspace(0,1,21):
            X=pre.copy();X[:3,3]=(1-f)*pre[:3,3]+f*T[:3,3];seed=model.ik(X,base,seed)
            if seed is None or not free_check(seed,base)[0]:break
            approach.append(seed.tolist())
        if len(approach)!=21:continue
        min_margin=min([p['margin_rad'] for p in fine]+[model.margin(np.array(q)) for q in approach+free])
        candidate={'classification':'ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT','asset_id':model.manifest['asset_id'],'base':base,'T_grasp':T.tolist(),'T':T.tolist(),'preplan':free,'approach':approach,'path':fine,'goal_state':goal,'planned_reached_state':fine[-1]['state'],'planned_blocker':reason,'end_retreat':None,'grasp_geometry_selection':selection,'allowed_authored_colliders':allowed,'contact_geometry_audit':g['contact'],'section_fraction':g['section'],'insertion_m':g['insertion_m'],'minimum_approach_arc_margin_rad':min_margin,'base_fixed_after_grasp':True,'saved_GT_grasp_template_used':False}
        key=(g['contact']['rank'],fine[-1]['state'],min_margin)
        if best is None or key>best[0]:best=(key,candidate)
        print('APPROACH_OK',model.manifest['asset_id'],key,base,flush=True)
    (output/'candidate_audit.json').write_text(json.dumps(audit,indent=2))
    if best:(output/'candidate.json').write_text(json.dumps(best[1],indent=2))
    summary={'asset_id':model.manifest['asset_id'],'oracle_handle':selection['mesh'],'found':best is not None,'wall_s':time.time()-started,'checked_arc_branches':len(audit),'target_rad':goal,'planned_reached_rad':best[1]['planned_reached_state'] if best else None,'minimum_approach_arc_margin_rad':best[1]['minimum_approach_arc_margin_rad'] if best else None,'blocker':None if best else 'BOUNDED_ORACLE_HANDLE_APPROACH_SEARCH_EXHAUSTED','new_execution_gates':False}
    (output/'planning_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
    return best[1] if best else None

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wall-s',type=float,default=600);a=p.parse_args();plan(json.loads(a.job.read_text()),a.output,a.wall_s)
