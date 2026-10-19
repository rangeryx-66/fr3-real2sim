"""One pose, one aperture, two pads: geometric ranking, never execution gates."""
import numpy as np
from handle_contact_quality import handle_surfaces,ray_surface


def common_contact_quality(model,surfaces,T,base,tangent):
    P=model.poses(model.home,base,width=.1);D=T@np.linalg.inv(P['tcp_link']);P={n:D@X for n,X in P.items()};tangent=np.asarray(tangent)/np.linalg.norm(tangent);report={};groups={};all_first=[]
    for label,meshes in [('visual',[surfaces[0]]),('native',surfaces[1])]:
        sides=[]
        for finger in ['gripper_link1','gripper_link2']:
            b=model.pads[finger];xy=np.array([[x,y] for x in np.linspace(*b[:,0],9) for y in np.linspace(*b[:,1],7)]);grid=np.c_[xy,np.full(len(xy),b[0,2])];origins=(np.c_[grid,np.ones(len(grid))]@P[finger].T)[:,:3];direction=-P[finger][:3,2];rays=[ray_surface(m,origins,direction) for m in meshes];dist=np.array([r[0] for r in rays]);which=dist.argmin(0);distance=dist[which,np.arange(len(grid))];normals=np.array([r[1] for r in rays])[which,np.arange(len(grid))];hit=np.isfinite(distance);interior=np.prod(np.sin(np.pi*(xy-b[0,:2])/(b[1,:2]-b[0,:2])),axis=1)
            sides.append((distance,normals,hit,interior,b,direction))
        groups[label]=sides;all_first.extend(s[0][s[2]].min() if s[2].any() else .05 for s in sides)
    closures=np.linspace(min(all_first),max(all_first),9)
    for label,sides in groups.items():
        samples=[]
        # Each sample has ONE common physical aperture; independent nearest
        # surface distances are never turned into two separate jaw positions.
        for closure in closures:
            support=[];records=[];average_normals=[]
            for finger,s in zip(['gripper_link1','gripper_link2'],sides):
                distance,normals,hit,interior,b,direction=s;band=(b[1,1]-b[0,1])/20;gap=distance-closure;weights=np.where(hit,np.exp(-np.minimum(abs(gap)/band,30)**2),0)*interior;alignment=np.clip(-normals@direction,0,1);area=float(np.prod(b[1,:2]-b[0,:2]));value=float(np.mean(weights*alignment**2));mean=(weights[:,None]*normals).sum(0)/max(weights.sum(),1e-30);mean/=max(np.linalg.norm(mean),1e-30);average_normals.append(mean);support.append(value);records.append({'finger':finger,'interior_supported_area_m2':float(area*np.mean(weights)),'support_score':value,'overlap_depth_max_m':float(max(0.,-np.min(gap[hit]))) if hit.any() else None,'contact_gap_min_m':float(np.min(gap[hit])) if hit.any() else None})
            opposition=float(np.clip((1-average_normals[0]@average_normals[1])/2,0,1));c=P['gripper_link1'][:3,2];normal_pull=max(0.,-average_normals[0]@tangent,-average_normals[1]@tangent);tangential=float(np.sqrt(max(0.,1-(c@tangent)**2)));direction_support=float((normal_pull+tangential)/np.sqrt(2));score=min(support)*opposition*direction_support;samples.append({'common_aperture_m':float(.1-2*closure),'bilateral_score':score,'opposition':opposition,'opening_direction_support':direction_support,'sides':records})
        report[label]={'samples':samples}
    chosen=max(range(len(closures)),key=lambda i:min(report[k]['samples'][i]['bilateral_score'] for k in ['visual','native']))
    for k in ['visual','native']:report[k]['best_common_pose_aperture']=report[k]['samples'][chosen]
    report['same_aperture_across_visual_native_and_both_fingers']=True
    report['score']=min(report[k]['samples'][chosen]['bilateral_score'] for k in ['visual','native']);report['classification']='GEOMETRY_RANK_ONLY_NOT_CONTACT_CERTIFICATION';return report


def joint_candidate_score(contact,reached,target,margin,continuity):
    # Usable range can at most double contact utility, so an almost-empty
    # contact patch cannot win solely through a long nominal IK arc.
    range_weight=1+min(1.,max(0.,reached/max(target,1e-30)))
    return float(contact*range_weight*(1+min(.1,max(0.,margin)/np.pi))/(1+max(0.,continuity)))
