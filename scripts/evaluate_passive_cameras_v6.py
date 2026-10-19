"""Post-exit numeric native-surface visibility diagnostic; never online input.

Cooked surfaces are an approximation to rendered visual surfaces. Depth agreement
is a coverage diagnostic, not object segmentation or hinge reconstruction.
"""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def evaluate(root):
    p=root/'episode_00';done=json.loads((p/'PROCESS_COMPLETION.json').read_text());pid=json.loads((p/'PROCESS.json').read_text())['pid']
    assert done['actor_exited'] and not Path('/proc/%d'%pid).exists()
    from piper_mobile_demo.owned_scene import meshes
    from revolute10_fixed import authored_handle_colliders
    export=json.loads((p/'planning_current_scene.json').read_text());job=json.loads((p/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text())
    from interaction_runtime.kinematics import FastNativeModel
    model=FastNativeModel(None,Path(job['asset_root']),source);allowed=authored_handle_colliders(export,model.manifest['interaction_geometry']['selection']);owner=next(e['rigid_body_path'] for e in export['shapes'] if e['path'] in allowed);prefix='/'.join(owner.split('/')[:3])+'/'
    rng=np.random.default_rng(6019);points={'moving':[],'stationary':[]}
    for e in export['shapes']:
        body=e.get('rigid_body_path','')
        if not (body.startswith(prefix) or body==prefix[:-1]) or e['type']=='Plane':continue
        category='moving' if body==owner else 'stationary';T=np.array(e['world_transform'])
        if category=='moving':T=np.linalg.inv(np.array(e['rigid_body_world_transform']))@T
        for mesh in meshes(e):
            if len(mesh.faces)==0:continue
            faces=np.asarray(mesh.faces);V=np.asarray(mesh.vertices);area=np.linalg.norm(np.cross(V[faces[:,1]]-V[faces[:,0]],V[faces[:,2]]-V[faces[:,0]]),axis=1)
            if area.sum()==0:continue
            tri=V[faces[rng.choice(len(faces),size=500,p=area/area.sum())]];w=rng.random((500,2));bad=w.sum(1)>1;w[bad]=1-w[bad];Q=tri[:,0]+w[:,0,None]*(tri[:,1]-tri[:,0])+w[:,1,None]*(tri[:,2]-tri[:,0]);points[category].append(Q@T[:3,:3].T+T[:3,3])
    points={k:np.vstack(v) for k,v in points.items() if v};dataset=root/'passive_geometry_dataset';cal=json.loads((dataset/'calibration.json').read_text())['cameras'];index=json.loads((dataset/'index.json').read_text());states=json.loads((root/'fixed_camera_evaluation/privileged_states.json').read_text())['states'];rows=[]
    for item,state in zip(index['pairs'],states):
        saved=np.load(dataset/item['path']/'depth_pair.npz');row=dict(sequence=item['sequence'],angle_deg=float(np.rad2deg(state['joint_angle_rad'])),synchronized=item['synchronized'],groups={})
        for group,Q0 in points.items():
            T=np.array(state['T_world_moving']);Q=Q0@T[:3,:3].T+T[:3,3] if group=='moving' else Q0;visible=[];views=[]
            for c in cal:
                C=np.array(c['T_world_camera_optical']);K=np.array(c['K']);P=(Q-C[:3,3])@C[:3,:3];uv=P@K.T;uv=np.rint(uv[:,:2]/np.maximum(P[:,2,None],1e-8)).astype(int);inside=(P[:,2]>.1)&(uv[:,0]>=0)&(uv[:,0]<640)&(uv[:,1]>=0)&(uv[:,1]<480);match=np.zeros(len(Q),bool);j=np.flatnonzero(inside);z=saved[c['camera_id']+'_depth_m'][uv[j,1],uv[j,0]];match[j]=np.isfinite(z)&(abs(z-P[j,2])<.008);visible.append(match);views.append(dict(camera=c['camera_id'],in_frustum_fraction=float(inside.mean()),native_surface_depth_agreement_fraction=float(match.mean())))
            row['groups'][group]=dict(sampled_points=len(Q),cameras=views,joint_depth_agreement_fraction=float(np.any(visible,axis=0).mean()))
        rows.append(row)
    result=dict(evaluation_only=True,online_fixed_image_inspection=False,method='seeded cooked-surface samples projected into actual saved optical-Z; 8mm diagnostic agreement, not an admission gate',limitation='native collision/visual mismatch, hidden back faces and sampling bias; no segmentation/reconstruction claim',rows=rows);out=root/'fixed_camera_evaluation/COVERAGE.json';out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));return result
if __name__=='__main__':evaluate(Path(sys.argv[1]))
