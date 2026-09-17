"""GT-free surface/view coverage audit and deterministic next-pass heuristic."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import cv2,numpy as np,trimesh
from scipy.spatial.transform import Rotation


def _unit(x):
    x=np.asarray(x,float);return x/np.maximum(np.linalg.norm(x,axis=-1,keepdims=True),1e-12)


def fibonacci_sphere(n=42):
    i=np.arange(n);z=1-2*(i+.5)/n;phi=np.pi*(3-np.sqrt(5))*i;r=np.sqrt(1-z*z)
    return np.c_[r*np.cos(phi),r*np.sin(phi),z]


def _boundary(mesh):
    edges=mesh.edges_sorted;unique,count=np.unique(edges,axis=0,return_counts=True);b=unique[count==1]
    length=float(np.linalg.norm(mesh.vertices[b[:,0]]-mesh.vertices[b[:,1]],axis=1).sum()) if len(b) else 0.
    return int(len(b)),length


def audit(mesh_path:Path,dataset:Path,tracking:Path,output:Path|None=None):
    mesh=trimesh.load(mesh_path,force='mesh',process=False);hull=mesh.convex_hull
    tri=np.asarray(hull.triangles);centers=tri.mean(axis=1);cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    area=np.linalg.norm(cross,axis=1)*.5;normals=_unit(cross);center=np.average(centers,axis=0,weights=np.maximum(area,1e-12))
    K=np.loadtxt(dataset/'cam_K.txt');seen_by_pass={};view_dirs=[];frames=[]
    for pose in sorted((dataset/'poses').glob('*.json')):
        row=json.loads(pose.read_text());track=tracking/'ob_in_cam'/f'{pose.stem}.txt'
        if not track.exists():continue
        T=np.loadtxt(track);R=T[:3,:3];t=T[:3,3];C=centers@R.T+t;z=C[:,2]
        uv=np.c_[K[0,0]*C[:,0]/z+K[0,2],K[1,1]*C[:,1]/z+K[1,2]];u=np.rint(uv[:,0]).astype(int);v=np.rint(uv[:,1]).astype(int)
        depth=cv2.imread(str(dataset/'depth'/f'{pose.stem}.png'),cv2.IMREAD_UNCHANGED).astype(float)/1000
        mask=cv2.imread(str(dataset/'masks'/f'{pose.stem}.png'),0)>0;hand=cv2.imread(str(dataset/'masks_hand'/f'{pose.stem}.png'),0)>0
        hh,w=mask.shape;inside=(z>.05)&(u>=0)&(u<w)&(v>=0)&(v<hh);idx=np.flatnonzero(inside);visible=np.zeros(len(centers),bool)
        if len(idx):
            d=depth[v[idx],u[idx]];ncam=normals[idx]@R.T;view=-C[idx]/np.maximum(np.linalg.norm(C[idx],axis=1,keepdims=True),1e-12)
            visible[idx]=mask[v[idx],u[idx]]&~hand[v[idx],u[idx]]&(d>0)&(np.abs(d-z[idx])<.005)&(np.einsum('ij,ij->i',ncam,view)>.1)
        pid=str(int(row.get('pass_id',0)));seen_by_pass.setdefault(pid,np.zeros(len(centers),bool));seen_by_pass[pid]|=visible
        cam=-R.T@t;direction=_unit(cam-center).reshape(3);view_dirs.append(direction)
        object_px=int(mask.sum());valid=float(np.mean(depth[mask]>0)) if object_px else 0.;occ=float(np.sum(hand&cv2.dilate(mask.astype(np.uint8),np.ones((15,15),np.uint8)).astype(bool))/max(object_px,1))
        mask_edge=cv2.morphologyEx(mask.astype(np.uint8),cv2.MORPH_GRADIENT,np.ones((3,3),np.uint8))>0;mask_edge&=mask
        d=depth.astype(np.float32)/1000.;jump=np.zeros_like(mask,bool)
        good=d>0
        dx=np.abs(d[:,1:]-d[:,:-1])>.004;ok=good[:,1:]&good[:,:-1];jump[:,1:]|=dx&ok;jump[:,:-1]|=dx&ok
        dy=np.abs(d[1:,:]-d[:-1,:])>.004;ok=good[1:,:]&good[:-1,:];jump[1:,:]|=dy&ok;jump[:-1,:]|=dy&ok
        frames.append({'frame':pose.stem,'pass_id':int(pid),'surface_visible_fraction':float(area[visible].sum()/area.sum()),'depth_validity':valid,'gripper_occlusion':occ,'mask_boundary_fraction':float(mask_edge.sum()/max(object_px,1)),'boundary_depth_jump_fraction':float((mask_edge&jump).sum()/max(mask_edge.sum(),1)),'view_direction_object':direction.tolist()})
    cumulative=np.zeros(len(centers),bool);passes={}
    for pid in sorted(seen_by_pass,key=int):
        own=seen_by_pass[pid];new=own&~cumulative;cumulative|=own
        f=[x for x in frames if x['pass_id']==int(pid)]
        passes[pid]={'frames':len(f),'surface_coverage':float(area[own].sum()/area.sum()),'unique_added_coverage':float(area[new].sum()/area.sum()),'cumulative_coverage':float(area[cumulative].sum()/area.sum()),'mean_depth_validity':float(np.mean([x['depth_validity'] for x in f])),'mean_gripper_occlusion':float(np.mean([x['gripper_occlusion'] for x in f]))}
    dirs=fibonacci_sphere();vd=np.asarray(view_dirs);dir_score=np.max(dirs@vd.T,axis=1) if len(vd) else np.full(len(dirs),-1.)
    unobserved=~cumulative
    # Candidate directions favor large unobserved envelope patches whose outward
    # normals face the camera, while penalizing repetition of existing directions.
    surface_score=np.array([np.sum(area[unobserved]*np.maximum(normals[unobserved]@d,0.)**4) for d in dirs])
    score=surface_score/np.maximum(area.sum(),1e-12)+.15*(1-dir_score)
    desired=dirs[int(np.argmax(score))]
    # Pick a deterministic coarse reorientation that maps the primary pass view
    # set toward the desired surface direction. This only proposes a scan family;
    # the frozen MoveIt/grasp checks still decide whether it can execute.
    candidates=[]
    for axis in np.eye(3):
        for deg in (-90,-60,-45,45,60,90):
            Q=Rotation.from_rotvec(axis*np.deg2rad(deg)).as_matrix();pred=vd@Q
            novelty=float(np.mean(1-np.max(pred@vd.T,axis=1))) if len(vd) else 1.
            focus=float(np.mean(np.maximum(pred@desired,0.)))
            candidates.append((focus+.35*novelty,axis,deg,Q))
    _,axis,deg,Q=max(candidates,key=lambda x:x[0])
    be,bl=_boundary(mesh)
    # Region summaries are deliberately expressed in the reconstructed object
    # frame.  They are an audit convention (z-up and the first camera ray as
    # front), not semantic labels inferred from GT.  This makes missing caps and
    # the back side visible without using the ground-truth mesh or pose.
    region_masks={
        'top':normals[:,2]>=.5,
        'bottom':normals[:,2]<=-.5,
        'side':np.abs(normals[:,2])<.5,
    }
    if len(vd):
        front=vd[0]
        region_masks['front']=normals@front>=.5
        region_masks['back']=normals@front<=-.5
    region_coverage={}
    for name,m in region_masks.items():
        denom=float(area[m].sum());region_coverage[name]={
            'proxy_area_m2':denom,
            'observed_fraction':float(area[m&cumulative].sum()/max(denom,1e-12)),
            'unobserved_fraction':float(area[m&unobserved].sum()/max(denom,1e-12)),
            'triangles':int(m.sum()),
        }
    report={'schema':'fr3_surface_coverage/v1','gt_used':False,'surface_proxy':'convex envelope of reconstructed mesh; area weighted','object_frame_region_convention':'z axis is reconstructed object z; front is first valid camera ray; these are audit bins, not GT semantics','mesh':str(mesh_path),'envelope_area_m2':float(area.sum()),'observed_surface_coverage':float(area[cumulative].sum()/area.sum()),'unobserved_surface_area_proxy_m2':float(area[unobserved].sum()),'boundary_edges':be,'boundary_length_m':bl,'thin_edge_diagnostics':{'definition':'object-mask boundary pixels; depth jump is >4 mm across a valid 4-neighbour pair','mean_mask_boundary_fraction':float(np.mean([x['mask_boundary_fraction'] for x in frames])) if frames else 0.,'mean_boundary_depth_jump_fraction':float(np.mean([x['boundary_depth_jump_fraction'] for x in frames])) if frames else 0.,'max_boundary_depth_jump_fraction':float(np.max([x['boundary_depth_jump_fraction'] for x in frames])) if frames else 0.,'depth_jump_threshold_mm':4.0},'region_coverage':region_coverage,'passes':passes,'view_direction_bins':dirs.tolist(),'view_direction_bin_coverage':dir_score.tolist(),'recommended_next_view_direction_object':desired.tolist(),'recommended_regrasp':{'axis_common_object':axis.tolist(),'angle_deg':float(deg),'rotation_common_object':Q.tolist(),'selection':'unobserved envelope normals plus view-direction novelty'},'frames':frames}
    if output:Path(output).write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mesh',type=Path);p.add_argument('dataset',type=Path);p.add_argument('tracking',type=Path);p.add_argument('--output',type=Path);a=p.parse_args();print(json.dumps(audit(a.mesh,a.dataset,a.tracking,a.output),indent=2))
