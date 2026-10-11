"""Benchmark adapters only: authored collider identity and fixed-base Pink plans.

No collision/physics editing, object-specific controllers or grasp templates.
The registered set and shared search budget are fixed before physical outcomes.
"""
import argparse, copy, json, re, sys, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from piper_pink_fixed import PinkIK
from piper_mobile_demo.model import Model
from articulated_demo.kinematics import transform
from interactive_twin_recovery.mobile import physical_scene_class, at_base, candidate_bases
from wrist_reconstruction.geometry import calibration
from wrist_reconstruction.planner import camera_clearance


def authored_handle_colliders(export, selection):
    """Match the importer-preserved authored mesh name, never swap geometry."""
    stem = re.sub(r"[^a-zA-Z0-9]", "", selection['mesh'])
    token = re.compile(r'(?:^|_)' + re.escape(stem) + r'(?:_|$)', re.I)
    return [e['path'] for e in export['shapes']
            if '/articulated_pose/' in e['path'] and token.search(e['path'])
            and e.get('rigid_body_path', '').split('/')[-1] == selection['handle_link']]


def plan(job, output, wall_s=300):
    started = time.time(); output = Path(output); output.mkdir(exist_ok=True)
    source = json.loads((Path(job['source'])/'report.json').read_text())
    model = Model(ROOT/'config/piper.urdf', job['asset_root'], source)
    solver = PinkIK(model.urdf, model.home); model.ik = solver.solve
    export = json.loads(Path(job['collision_export']).read_text())
    initial = source['robot_base_pose']; cal = calibration(job['camera_calibration'])
    meta = model.manifest; joint = meta['joint_name']; link = meta['moving_link']
    goal = float(meta['source_joint_limits_rad']['upper'])
    M0 = model.asset_T @ model.asset.root_to_link(link, {joint: 0.})
    audit = []; best = None; checked = 0

    def delta(s):
        return model.asset_T @ model.asset.root_to_link(link, {joint: float(s)}) @ np.linalg.inv(M0)

    for selection in job['grasp_geometry_candidates']:
        allowed = authored_handle_colliders(export, selection)
        if not allowed:
            audit.append({'mesh': selection['mesh'], 'blocker': 'NO_AUTHORED_COLLIDER_ASSOCIATION'}); continue
        sc0 = physical_scene_class(ROOT)(export, model, allowed)
        width = min(.1, float(selection['dimensions_m'][1]))
        P = model.poses(model.home, initial, width=width)
        offset = np.mean([(np.linalg.inv(P['tcp_link'])@P[n]@np.r_[model.pads[n].mean(0),1])[:3]
                          for n in ('gripper_link1','gripper_link2')], axis=0)
        normal = model.asset_T[:3,:3] @ np.array(selection['outward_normal_root'])
        axis = model.asset_T[:3,:3] @ np.array(selection['axis_root'])
        normal -= axis * np.dot(normal,axis); normal /= np.linalg.norm(normal)
        for section in sorted(selection['sections'], key=lambda s: abs(s['fraction'])):
            anchor = (model.asset_T@np.r_[section['anchor_root_m'],1])[:3]
            E0 = np.eye(4); E0[:3,:3] = np.column_stack((axis,np.cross(-normal,axis),-normal))
            E0[:3,3] = anchor - E0[:3,:3]@offset
            pool = candidate_bases(initial, {'anchor_world_m': anchor.tolist(),
                                            'outward_normal_world': normal.tolist()},
                                   job['wrist_experiment']['mobile_scan'])
            for base in [initial]+pool:
                if time.time()-started > wall_s: break
                sc = physical_scene_class(ROOT)(at_base(export, initial, base),model,allowed)
                for roll in (0.,180.):
                    if time.time()-started > wall_s: break
                    T = E0.copy(); T[:3,:3] = T[:3,:3] @ Rotation.from_euler('z',roll,degrees=True).as_matrix()
                    q = model.ik(T,base,starts=4); checked += 1
                    if q is None:
                        audit.append({'mesh':selection['mesh'],'base':base,'roll':roll,'blocker':'NO_INITIAL_PINK_IK'});continue
                    # Same wrist alternatives as the previous fixed-base experiment.
                    branches = [q]
                    flip=q.copy();flip[3]+=np.pi if q[3]<0 else -np.pi;flip[4]*=-1;flip[5]+=np.pi if q[5]<0 else -np.pi
                    qflip=model.ik(T,base,flip)
                    if qflip is not None:branches.append(qflip)
                    for seed in branches:
                        path=[];reason=None
                        for s in np.linspace(0,goal,max(2,int(np.ceil(goal/np.deg2rad(3)))+1)):
                            X=delta(s)@T;seed=model.ik(X,base,seed)
                            if seed is None:reason='NO_CONTINUOUS_PINK_IK';break
                            poses=model.poses(seed,base,width=width)
                            ok,reason=sc.check(poses,delta(s)@sc.moving_reference,True)
                            if ok:ok,reason=camera_clearance(sc,poses,cal)
                            if not ok:break
                            path.append({'state':float(s),'q':seed.tolist(),'T_tcp':X.tolist(),'margin_rad':model.margin(seed)})
                        audit.append({'mesh':selection['mesh'],'section':section['fraction'],'base':base,'roll':roll,
                                      'reached_rad':path[-1]['state'] if path else 0.,'blocker':reason})
                        # Candidate selection, not a future-trajectory execution gate.
                        if not path or (best and path[-1]['state']<=best['planned_reached_state']):continue
                        pre=T.copy();pre[:3,3]-=.04*T[:3,2]
                        qpre=model.ik(pre,base,np.array(path[0]['q']),starts=2)
                        if qpre is None:continue
                        view=copy.copy(model)
                        def free_check(qq,bb):
                            poses=model.poses(qq,bb,width=.1);ok,why=sc.check(poses,sc.moving_reference,False)
                            if ok:ok,why=camera_clearance(sc,poses,cal)
                            return ok,why,None
                        view.check=free_check
                        free=Model.joint_plan(view,model.home,qpre,base,iterations=600)
                        if free is None:continue
                        approach=[];seed=qpre
                        for fraction in np.linspace(0,1,21):
                            X=pre.copy();X[:3,3]=(1-fraction)*pre[:3,3]+fraction*T[:3,3]
                            seed=model.ik(X,base,seed)
                            if seed is None or not free_check(seed,base)[0]:break
                            approach.append(seed.tolist())
                        if len(approach)!=21:continue
                        best={'classification':'KNOWN_MODEL_DIAGNOSTIC','asset_id':meta['asset_id'],'base':base,
                              'T_grasp':T.tolist(),'T':T.tolist(),'preplan':free,'approach':approach,
                              'path':path,'goal_state':goal,'planned_reached_state':path[-1]['state'],
                              'end_retreat':None,'grasp_geometry_selection':selection,'allowed_authored_colliders':allowed,
                              'planned_blocker':reason,'base_fixed_after_grasp':True,'saved_GT_grasp_template_used':False}
                        (output/'candidate.json').write_text(json.dumps(best,indent=2))
                        print('BEST',meta['asset_id'],np.rad2deg(best['planned_reached_state']),base,selection['mesh'],flush=True)
                        if best['planned_reached_state']>=goal:break
                    if best and best['planned_reached_state']>=goal:break
                if best and best['planned_reached_state']>=goal:break
            if time.time()-started>wall_s or (best and best['planned_reached_state']>=goal):break
        if time.time()-started>wall_s or (best and best['planned_reached_state']>=goal):break
    summary={'asset_id':meta['asset_id'],'found':best is not None,'wall_s':time.time()-started,
             'checked_grasp_station_pairs':checked,'target_rad':goal,'planned_reached_rad':best['planned_reached_state'] if best else None,
             'blocker':None if best else 'BOUNDED_AUTHORED_GEOMETRY_GRASP_BASE_SEARCH_EXHAUSTED'}
    (output/'planning_summary.json').write_text(json.dumps(summary,indent=2))
    (output/'candidate_audit.json').write_text(json.dumps(audit,indent=2))
    print(json.dumps(summary),flush=True)
    return best


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wall-s',type=float,default=300);a=p.parse_args();plan(json.loads(a.job.read_text()),a.output,a.wall_s)
