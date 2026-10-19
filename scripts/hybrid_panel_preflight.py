"""Observation-selected panel patches versus archived native solids, no physics.

Image rectangles are supplied by the inspecting Agent, never inferred from an
asset ID or GT mesh. Native solids are used separately for contact/safety audit.
This program cannot launch Isaac or produce a physical manipulation outcome.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from articulated_demo.kinematics import URDFChain, transform
from piper_mobile_demo.owned_scene import meshes, intersects
from interactive_twin_recovery.mobile import physical_scene_class
from cross_robot_adapter import RobotPinkIK
from types import SimpleNamespace


FINGERS = ('left_finger', 'right_finger')
JOINTS = ('drive_joint', 'left_finger_joint', 'left_inner_knuckle_joint',
          'right_outer_knuckle_joint', 'right_finger_joint', 'right_inner_knuckle_joint')
PAD = dict(left_finger=np.array([[-1.,0,0,0],[0,0,1.,0],[0,1.,0,0],[0,0,0,1.]]),
           right_finger=np.array([[1.,0,0,0],[0,0,-1.,0],[0,1.,0,0],[0,0,0,1.]]))


def rays(mesh, origins, direction):
    """Nearest positive intersection; no triangle proximity/GT pose correction."""
    t = mesh.triangles
    e1, e2 = t[:,1]-t[:,0], t[:,2]-t[:,0]
    p = np.cross(np.broadcast_to(direction,e2.shape),e2)
    det = np.einsum('ij,ij->i',e1,p)
    good = abs(det)>1e-12
    inv = np.divide(1.,det,out=np.zeros_like(det),where=good)
    out = []
    for origin in origins:
        s = origin-t[:,0]
        u = np.einsum('ij,ij->i',s,p)*inv
        q = np.cross(s,e1)
        v = q@direction*inv
        d = np.einsum('ij,ij->i',e2,q)*inv
        hit = good & (u>=0) & (v>=0) & (u+v<=1) & (d>=0)
        out.append(float(d[hit].min()) if hit.any() else float('inf'))
    return np.asarray(out)


def patch_from_rgbd(folder, rectangle):
    c = json.loads((folder/'camera.json').read_text())
    if (folder/'depth_m.npy').exists():depth=np.load(folder/'depth_m.npy')
    else:
        with np.load(folder/'checkpoint_arrays.npz') as saved:depth=saved['depth_m'].copy()
    K=np.asarray(c['K']); C=np.asarray(c['T_world_camera_optical'])
    x0,y0,x1,y1=rectangle
    yy,xx=np.mgrid[y0:y1:4,x0:x1:4]
    z=depth[yy,xx]; good=np.isfinite(z)&(z>0)
    exclusion=folder/'robot_self_exclusion.npy'
    if exclusion.exists():good &= ~np.load(exclusion).astype(bool)[yy,xx]
    elif (folder/'checkpoint_arrays.npz').exists():
        with np.load(folder/'checkpoint_arrays.npz') as saved:
            if 'robot_self_exclusion' in saved:good &= ~saved['robot_self_exclusion'].astype(bool)[yy,xx]
    px=np.c_[xx[good],yy[good],np.ones(good.sum())]
    Q=(px@np.linalg.inv(K).T)*z[good,None]
    points=Q@C[:3,:3].T+C[:3,3]
    center=np.median(points,axis=0)
    _,s,V=np.linalg.svd(points-center,full_matrices=False)
    n=V[-1]
    if n@(C[:3,3]-center)<0:n=-n
    return dict(points=points,center=center,normal=n,
        plane_residual_m=np.abs((points-center)@n),camera=c,
        rectangle=rectangle,point_count=len(points),singular_values=s)


def native_model(episode, export):
    """Same URDF, joint semantics and cooked solids, without remote mesh paths."""
    from types import SimpleNamespace
    c=json.loads((episode/'sensor_assembly/SENSOR_CONFIG.json').read_text())
    urdf=episode/'sensor_assembly/sim_urdf.urdf';c['urdf']=str(urdf)
    chain=URDFChain(urdf); root=ET.parse(urdf).getroot()
    joints={j.get('name'):j for j in root.findall('joint')}
    names=c['arm_joint_names']
    limits=np.array([[float(joints[n].find('limit').get(k)) for k in ['lower','upper']] for n in names])
    graph={l.get('name'):set() for l in root.findall('link')};fixed=[]
    for j in joints.values():
        a=j.find('parent').get('link');b=j.find('child').get('link');graph[a].add(b);graph[b].add(a)
        if j.get('type')=='fixed':fixed.append((a,b))
    cluster={n:{n} for n in graph}
    for _ in graph:
        for a,b in fixed:cluster[a]|=cluster[b];cluster[b]|=cluster[a]
    allowed={frozenset((a,b)) for a in graph for b in graph
             if cluster[a]&cluster[b] or any(graph[x]&cluster[b] for x in cluster[a])}
    alias=dict(zip(FINGERS,('gripper_link1','gripper_link2')))
    allowed={frozenset(alias.get(n,n) for n in pair) for pair in allowed}
    for side,logical in zip(('left','right'),alias.values()):
        allowed.add(frozenset((logical,side+'_inner_knuckle')))
        allowed.add(frozenset((side+'_inner_knuckle',side+'_outer_knuckle')))
    bodies={e.get('rigid_body_path','').split('/')[-1] for e in export['shapes'] if e['path'].startswith('/World/Piper')}
    def poses(q,base,angle=.85,width=None,finger_q=None):
        if width is not None:
            # Same aperture lookup as NativeGripperGeometry, measured from
            # unchanged archived inner finger surfaces and official kinematics.
            angle=float(np.interp(width,width_grid[::-1],angle_grid[::-1]))
        values=dict(zip(names,q));values.update(dict.fromkeys(JOINTS,angle))
        if finger_q is not None:values.update(dict(zip(JOINTS,np.asarray(finger_q))))
        B=transform(base[:3],[0,0,np.deg2rad(base[3])])
        P={n:B@chain.root_to_link(n,values) for n in bodies}
        P['tcp_link']=B@chain.root_to_link('tcp_link',values)
        for n in FINGERS:P[alias[n]]=P[n]@PAD[n]
        return P
    angle_grid=np.linspace(0,.85,1001);width_grid=[]
    native_inner={}
    for n in FINGERS:
        e=next(e for e in export['shapes'] if e.get('rigid_body_path','').split('/')[-1]==n)
        mesh=meshes(e,False)[0];mesh.apply_transform(np.linalg.inv(e['rigid_body_world_transform'])@e['world_transform'])
        native_inner[n]=mesh.vertices[:,1].min() if n=='left_finger' else mesh.vertices[:,1].max()
    B=np.linalg.inv(chain.root_to_link('xarm_gripper_base_link',{}))
    for a in angle_grid:
        points=[(B@chain.root_to_link(n,dict.fromkeys(JOINTS,a))@np.array([0,native_inner[n],0,1]))[:3] for n in FINGERS]
        width_grid.append(points[0][1]-points[1][1])
    width_grid=np.asarray(width_grid)
    m=SimpleNamespace(config=c,home=np.asarray(c['home_q']),names=names,limits=limits,
        chain=chain,allowed=allowed,poses=poses,
        margin=lambda q:float(np.min(np.minimum(np.asarray(q)-limits[:,0],limits[:,1]-q))))
    mapped=copy.deepcopy(export)
    for e in mapped['shapes']:
        n=e.get('rigid_body_path','').split('/')[-1]
        if n in alias:
            e['rigid_body_path']=e['rigid_body_path'].rsplit('/',1)[0]+'/'+alias[n]
            e['rigid_body_world_transform']=(np.asarray(e['rigid_body_world_transform'])@PAD[n]).tolist()
    return m,mapped


def audit(episode, observation, regions, prior, output, ik_starts=4):
    began=time.perf_counter();output.mkdir(parents=True,exist_ok=True)
    export=json.loads((episode/'cooked_initial.json').read_text())
    allowed=json.loads((episode/'sensor_metadata.json').read_text())['allowed_handle_colliders']
    model,mapped=native_model(episode,export)
    scene=physical_scene_class(Path(__file__).resolve().parents[1])(mapped,model,allowed)
    solver=RobotPinkIK(model.config)
    cam=json.loads((observation/'camera.json').read_text());base=cam['base'];start=np.asarray(cam['robot_q'][:7])
    axis=np.asarray(prior['axis_world']);origin=np.asarray(prior['origin_world_m'])
    def delta(deg):
        R=Rotation.from_rotvec(axis*np.deg2rad(deg)).as_matrix();D=np.eye(4);D[:3,:3]=R;D[:3,3]=origin-R@origin;return D
    raw=[];cooked=[]
    for e in export['shapes']:
        if e['path'].startswith('/World/Piper') or e['type']!='Mesh':continue
        for target,cook in [(raw,False),(cooked,True)]:
            for mesh in meshes(e,cook):
                mesh.apply_transform(e['world_transform']);target.append((e,mesh))
    # The true existing distal fingertip caps, not new collision patches.
    P=model.poses(model.home,[0,0,0,0]);fingers=[]
    for n in FINGERS:
        e=next(e for e in export['shapes'] if e.get('rigid_body_path','').split('/')[-1]==n)
        T=np.linalg.inv(P['tcp_link'])@P[n]@np.linalg.inv(e['rigid_body_world_transform'])@e['world_transform']
        mesh=meshes(e,False)[0];mesh.apply_transform(T);fingers.append(mesh)
    max_z=max(m.vertices[:,2].max() for m in fingers)
    cap=np.concatenate([m.vertices[m.vertices[:,2]>max_z-1e-5] for m in fingers]);tip=cap.mean(0)
    reports=[]
    for region in regions:
        patch=patch_from_rgbd(observation,region['rectangle']);center=patch['center'];normal=patch['normal'];direction=-normal
        np.savez_compressed(output/(region['name']+'_observed.npz'),points=patch['points'],normal=normal,center=center)
        origins=patch['points'][::max(1,len(patch['points'])//100)] + normal*.01
        target_audits={}
        for label,solids in [('raw',raw),('cooked',cooked)]:
            dist=np.asarray([rays(m,origins,direction) for _,m in solids]);idx=dist.argmin(0);d=dist[idx,np.arange(len(origins))]
            near=np.isfinite(d)&(abs(d-.01)<.003)
            counts={}
            for i in idx[near]:
                key=solids[i][0]['path'];counts[key]=counts.get(key,0)+1
            target_audits[label]=dict(observed_surface_supported_count=int(near.sum()),rays=len(d),matching_shapes=counts,
                median_surface_gap_m=float(np.median(d-.01)) if np.isfinite(d).any() else None)
        target_paths=set(target_audits['cooked']['matching_shapes'])
        moving_paths={e['path'] for e in export['shapes'] if e.get('rigid_body_path','').split('/')[-1]==scene.moving_link}
        target_paths &= moving_paths
        record=dict(name=region['name'],rectangle=region['rectangle'],center_world=center.tolist(),normal_world=normal.tolist(),
            points=len(patch['points']),p95_planarity_residual_m=float(np.percentile(patch['plane_residual_m'],95)),
            native_surface_audit=target_audits,candidates=[])
        reports.append(record)
        if not target_paths:
            record['reason']='OBSERVED_REGION_HAS_NO_MATCHING_NATIVE_MOVING_PANEL_SURFACE';continue
        # All candidates originate at observed points; native surface audit only
        # accepts/rejects proposed contacts, never moves them onto GT surfaces.
        up=np.array([0.,0.,1.]);x=np.cross(up,direction);x/=np.linalg.norm(x)
        R0=np.column_stack([x,np.cross(direction,x),direction])
        for roll in [0,90,180,270]:
            R=R0@Rotation.from_euler('z',roll,degrees=True).as_matrix()
            reference=np.eye(4);reference[:3,:3]=R;reference[:3,3]=center-R@tip
            candidate=dict(roll_deg=roll,paths=[],valid=False)
            record['candidates'].append(candidate)
            for open_deg in [6.,10.,15.]:
                entry=dict(open_deg=open_deg,valid=False,reason=None);candidate['paths'].append(entry)
                D=delta(open_deg);T=D@reference;n=D[:3,:3]@normal;moving=D@scene.moving_reference
                q=solver.solve(T@transform([0,0,-.04]),base,start,starts=ik_starts)
                if q is None:entry['reason']='NO_STANDOFF_IK';continue
                path=[q.tolist()];reason=None
                for backoff in np.linspace(.04,.001,21):
                    target=T.copy();target[:3,3]+=backoff*n
                    q=solver.solve(target,base,q,starts=1)
                    if q is None:reason='NO_APPROACH_IK';break
                    ok,why=scene.check(model.poses(q,base),moving,False)
                    if not ok:reason=why;break
                    path.append(q.tolist())
                entry.update(reason=reason or 'STANDOFF_AND_APPROACH_ONLY',approach_q=path,
                    minimum_joint_margin_rad=min(model.margin(q) for q in path),target_tcp=T.tolist())
                # No broad contact waiver here. Loaded push-path validation and
                # return route are deliberately not claimed by this stage.
            candidate['valid']=any(e['reason']=='STANDOFF_AND_APPROACH_ONLY' for e in candidate['paths'])
    report=dict(schema='hybrid-contact-preflight-v0',physical_episodes_started=0,
        input_images_inspected_by_agent=True,selected_from='wrist RGB-D rectangles supplied by Agent',
        native_geometry_role='separate contact/safety audit only; not executable point correction',
        fully_closed_driver_rad=.85,closed_tip_center_tcp=tip.tolist(),closed_tip_bounds_tcp=[cap.min(0).tolist(),cap.max(0).tolist()],
        region_audits=reports,full_pull_push_regrasp_route_validated=False,wall_s=time.perf_counter()-began)
    (output/'PANEL_PREFLIGHT.json').write_text(json.dumps(report,indent=2));return report


def scoped_panel_check(scene, poses, moving_pose, paths):
    """Offline contact proposal only: remove no pair except nominated finger/panel.

    Native scene/checker and every self/other environment collision remain.
    The final physical contact authorization additionally needs real cap checks.
    """
    target=[s for s in scene.scene if s.path in paths]
    first=copy.copy(scene);first.scene=[s for s in scene.scene if s.path not in paths]
    ok,why=first.check(poses,moving_pose,False)
    if not ok:return ok,why
    second=copy.copy(scene);second.scene=target
    second.robot=[s for s in scene.robot if s.body not in ('gripper_link1','gripper_link2')]
    return second.check(poses,moving_pose,False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--episode',type=Path,required=True);p.add_argument('--observation',type=Path,required=True)
    p.add_argument('--agent-regions',type=Path,required=True);p.add_argument('--prior',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=audit(a.episode,a.observation,json.loads(a.agent_regions.read_text()),json.loads(a.prior.read_text()),a.output)
    print(json.dumps(result,indent=2))
