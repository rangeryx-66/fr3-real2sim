"""Geometry-only ranking of bilateral pad patches and opening transmission.

No acceptance thresholds or runtime gates. Original visual/native meshes stay
unchanged. Area and normal estimates are planner predictions, not sensor data.
"""
import numpy as np
from piper_mobile_demo.owned_scene import meshes
from rank_prepared_handle_assets import load


def handle_surfaces(model, export, allowed, selection):
    _, parts = load(model.asset_urdf)
    visual = next(p[3] for p in parts if p[1] == 'visual' and p[2] == selection['mesh'])
    visual.apply_transform(model.asset_T)
    native = []
    for shape in export['shapes']:
        if shape['path'] in allowed:
            for mesh in meshes(shape):
                mesh.apply_transform(np.asarray(shape['world_transform']));native.append(mesh)
    return visual, native


def ray_surface(mesh, origins, direction):
    tri = mesh.triangles; e1 = tri[:,1]-tri[:,0]; e2 = tri[:,2]-tri[:,0]
    h = np.cross(direction, e2); det = np.einsum('ij,ij->i', e1,h)
    inv = np.divide(1.,det,out=np.zeros_like(det),where=abs(det)>1e-12)
    s = origins[:,None,:]-tri[None,:,0,:]
    u = np.einsum('kij,ij->ki',s,h)*inv
    q = np.cross(s,e1);v = np.einsum('kij,j->ki',q,direction)*inv
    distance = np.einsum('kij,ij->ki',q,e2)*inv
    valid=(abs(det)>1e-12)&(u>=0)&(v>=0)&(u+v<=1)&(distance>=0)
    distance=np.where(valid,distance,np.inf);ids=distance.argmin(axis=1)
    normals=mesh.face_normals[ids].copy()
    # Source visuals can be double-sided or inconsistently wound. Orient the
    # first visible surface toward the casting pad before comparing normals.
    normals*=np.where(normals@direction>0,-1.,1.)[:,None]
    return distance[np.arange(len(origins)),ids],normals


def patch_quality(model, surfaces, T, base, opening_tangent):
    P=model.poses(model.home,base,width=.1);D=T@np.linalg.inv(P['tcp_link']);P={n:D@X for n,X in P.items()}
    tangent=np.asarray(opening_tangent);tangent/=np.linalg.norm(tangent)
    reports={}
    for label,group in [('visual',[surfaces[0]]),('native',surfaces[1])]:
        sides=[]
        for finger in ('gripper_link1','gripper_link2'):
            bounds=model.pads[finger];grid=np.array([[x,y,bounds[0,2]] for x in np.linspace(*bounds[:,0],9) for y in np.linspace(*bounds[:,1],7)])
            origins=(np.c_[grid,np.ones(len(grid))]@P[finger].T)[:,:3];direction=-P[finger][:3,2]
            rays=[ray_surface(mesh,origins,direction) for mesh in group];distances=np.array([r[0] for r in rays]);which=distances.argmin(axis=0);distance=distances[which,np.arange(len(grid))];normals=np.array([r[1] for r in rays])[which,np.arange(len(grid))];hit=np.isfinite(distance)
            # Geometric surface-flatness band, derived from pad depth. Smooth
            # weights rank contact support; no pose is vetoed by this value.
            band=float(bounds[1,1]-bounds[0,1])/20
            weight=np.zeros(len(grid))
            if hit.any():weight[hit]=np.exp(-((distance[hit]-distance[hit].min())/band)**2)
            area=float(np.prod(bounds[1,:2]-bounds[0,:2]));closing=np.clip(-normals@direction,0,1)
            outward=normals@tangent
            # A front-facing wedge pushes against the desired opening force.
            # Side patches support tangential pulling; rear-facing ones help.
            transmission=closing**2*(1-np.maximum(outward,0))
            support=float(np.mean(weight*transmission))
            sides.append({'finger':finger,'ray_hit_fraction':float(hit.mean()),'effective_patch_area_m2':float(area*weight.mean()),'closing_alignment':float(np.sum(weight*closing)/max(weight.sum(),1e-30)),'outward_normal_projection':float(np.sum(weight*outward)/max(weight.sum(),1e-30)),'normal_support_score':support,'contact_depth_variation_m':float(np.ptp(distance[hit])) if hit.any() else None,'weighted_contact_world_m':(np.sum((origins+np.where(hit,distance,0)[:,None]*direction)*weight[:,None],axis=0)/max(weight.sum(),1e-30)).tolist()})
        reports[label]={'sides':sides,'bilateral_support_score':min(s['normal_support_score'] for s in sides),'bilateral_effective_patch_area_m2':min(s['effective_patch_area_m2'] for s in sides)}
    reports['score']=min(reports[k]['bilateral_support_score'] for k in ['visual','native'])
    reports['classification']='GEOMETRIC_RANK_ONLY_NOT_PHYSICAL_VALIDATION';return reports
