"""Archived V4.3 diagnosis only: measured motion and native surface correspondence."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.serialization import result_packet


def patch_saved(folder,rectangle):
    m=json.loads((folder/'camera.json').read_text());K=np.asarray(m['K']);C=np.asarray(m['T_world_camera_optical'])
    with np.load(folder/'checkpoint_arrays.npz') as packed:
        D=packed['depth_m'];exclude=packed.get('robot_self_exclusion',np.zeros(D.shape,bool));x0,y0,x1,y1=rectangle
        yy,xx=np.mgrid[y0:y1:4,x0:x1:4];z=D[yy,xx];good=np.isfinite(z)&(z>0)&~exclude[yy,xx]
        Q=np.c_[(xx[good]-K[0,2])*z[good]/K[0,0],(yy[good]-K[1,2])*z[good]/K[1,1],z[good]]
    P=Q@C[:3,:3].T+C[:3,3];center=np.median(P,axis=0);_,s,V=np.linalg.svd(P-center,full_matrices=False);normal=V[-1]
    if normal@(C[:3,3]-center)<0:normal=-normal
    return P,center,normal,m


def audit(p,cooked,out):
    from diagnose_probing_v42 import audit as transmission_audit
    from interaction_runtime.kinematics import FastNativeModel
    from xarm7_native_execution import mapped_export
    from interactive_twin_recovery.mobile import physical_scene_class
    from revolute10_fixed import authored_handle_colliders
    from piper_mobile_demo.owned_scene import meshes
    from hybrid_panel_preflight import rays
    import trimesh
    out.mkdir(parents=True,exist_ok=True)
    transmission=transmission_audit(p,out/'transmission.json')
    j=json.loads((p/'job.json').read_text());m=FastNativeModel(None,Path(j['asset_root']),json.loads((Path(j['source'])/'report.json').read_text()));export=json.loads(cooked.read_text());allowed=authored_handle_colliders(export,m.manifest['interaction_geometry']['selection']);scene=physical_scene_class(ROOT)(mapped_export(export),m,allowed)
    measured=json.loads((p/'agent_responses/006.json').read_text())['result']['safe_exit']['park'];M=np.asarray(measured['moving_pose'])
    folder=p/'rgbd/generation_01_agent_02';P,center,n,camera=patch_saved(folder,[760,320,940,540]);results=[]
    for path,entry in scene.entries.items():
        body=entry.get('rigid_body_path','').split('/')[-1]
        if body!=scene.moving_link:continue
        local=np.linalg.inv(np.asarray(entry['rigid_body_world_transform']))@np.asarray(entry['world_transform']);item=dict(path=path,body=body,local_transform=local.tolist())
        for cooked_flag,label in [(True,'cooked'),(False,'raw_visual')]:
            mm=meshes(entry,cooked_flag)
            for mesh in mm:mesh.apply_transform(M@local)
            mesh=trimesh.util.concatenate(mm);hit=rays(mesh,[center+.01*n],-n)[0]
            closest,distance,_=trimesh.proximity.closest_point(mesh,P[::max(1,len(P)//200)])
            item[label]=dict(ray_m=float(hit) if np.isfinite(hit) else None,old_match=bool(np.isfinite(hit) and abs(hit-.01)<.003),surface_distance_mm=(1000*np.quantile(distance,[0,.5,.9,1])).tolist(),signed_normal_difference_mm=(1000*np.quantile((closest-P[::max(1,len(P)//200)])@n,[0,.5,1])).tolist(),bounds_world_m=mesh.bounds.tolist())
        results.append(item)
    report=dict(archived_only=True,reference='169263f',panel=dict(rectangle=[760,320,940,540],points=len(P),measured_center_world_m=center.tolist(),normal_world=n.tolist(),camera=camera,actual_moving_pose=M.tolist(),closed_moving_reference=scene.moving_reference.tolist(),moving_body=scene.moving_link,candidates=results),transmission=transmission)
    (out/'DIAGNOSIS.json').write_text(json.dumps(result_packet(report),indent=2));print('PANEL',json.dumps(report['panel'],default=str)[:15000]);return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('cooked',type=Path);p.add_argument('output',type=Path);a=p.parse_args();audit(a.episode,a.cooked,a.output)
