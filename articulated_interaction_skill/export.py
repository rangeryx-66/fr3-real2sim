"""Backend adapters, not a new articulated reconstruction algorithm.

Ditto's official demo consumes normalized pc_start/pc_end (1,8192,3).
ART currently publishes a method interface, not an available parser; its
manifest below is explicitly ART-style, never an assertion of executed ART.
"""
import json
import shutil
from pathlib import Path
import numpy as np


def normalized_pair(start, end, count=8192, seed=20261005):
    A=np.asarray(start,float);B=np.asarray(end,float)
    if len(A)<10 or len(B)<10 or not np.isfinite(A).all() or not np.isfinite(B).all():
        raise ValueError('INVALID_PAIR_CLOUD')
    lo=np.minimum(A.min(0),B.min(0));hi=np.maximum(A.max(0),B.max(0))
    center=(lo+hi)/2;scale=float((hi-lo).max()*1.1)
    if scale<=0:raise ValueError('DEGENERATE_PAIR')
    rng=np.random.default_rng(seed)
    sample=lambda x:((x[rng.choice(len(x),count,replace=len(x)<count)]-center)/scale).astype(np.float32)
    return sample(A)[None],sample(B)[None],center,scale


def export_capture(output):
    root=Path(output);capture=json.loads((root/'multistate_capture.json').read_text())
    backend=root/'reconstruction';backend.mkdir(exist_ok=True)
    # Copy only sensor data, not simulator assets/GT joint parameters, into inputs.
    if (root/'states').exists():shutil.copytree(root/'states',backend/'observations',dirs_exist_ok=True)
    groups=[]
    for state in capture['states']:
        views=[]
        for view in state['views']:
            relative=Path(view['directory']);folder=backend/'observations'/relative.relative_to('states')
            views.append({'rgb':str(folder.relative_to(backend)/'rgb.png'),
                          'depth_m':str(folder.relative_to(backend)/'depth_m.npy'),
                          'object_mask':str(folder.relative_to(backend)/'mask.png'),
                          'moving_part_mask':str(folder.relative_to(backend)/'moving_part_mask.png'),
                          'point_cloud':str(folder.relative_to(backend)/'point_cloud.npz'),
                          'K':view['K'],'T_world_camera_optical':view['T_world_camera_optical']})
        groups.append({'state_id':state['state_id'],'estimated_state':state['estimated_articulation_state'],
                       'label_units':capture['units'],'views':views})
    art={'schema':'ART-style-multistate-v1','object_id':capture['object_id'],'states':groups,
         'known_camera_poses':True,'coordinate_system':'world meters, ROS optical camera',
         'state_labels_source':capture['articulation_labels'],
         'source':'https://kyleleey.github.io/ART/',
         'integration_status':'inputs prepared; official public loader/checkpoint not available on project page; reconstruction not executed'}
    (backend/'art_input.json').write_text(json.dumps(art,indent=2))
    pairs=[];states=capture['states']
    pair_indices=list(zip(range(len(states)-1),range(1,len(states))))
    if len(states)>2:pair_indices.append((0,len(states)-1))
    for a,b in pair_indices:
        def cloud(i):
            clouds=[np.load(root/v['directory']/'point_cloud.npz')['points_world_m'] for v in states[i]['views']]
            return np.concatenate(clouds)
        A,B,center,scale=normalized_pair(cloud(a),cloud(b))
        path=backend/f'pair_{a:03d}_{b:03d}.npz'
        np.savez_compressed(path,pc_start=A,pc_end=B,norm_center=center,norm_scale=scale,
                            state_start=states[a]['estimated_articulation_state'],state_end=states[b]['estimated_articulation_state'])
        pairs.append({'before':a,'after':b,'tensor_file':path.name,'center_world_m':center.tolist(),
                      'scale_m':scale,'shape':[1,8192,3],'rgbd_groups':[groups[a],groups[b]]})
    ditto={'schema':'Ditto-demo-tensors-v1','pairs':pairs,'tensor_keys':['pc_start','pc_end'],
           'source':'https://github.com/UT-Austin-RPL/Ditto/blob/master/notebooks/demo_depth_map.ipynb',
           'normalization':'shared bbox center; max side * 1.1; same world frame; deterministic 8192-point sample',
           'integration_status':'load with backend_sample.py; inference not executed',
           'HouseDitto':'same paired RGB-D/cloud observations; task-specific HouseDitto parser is not claimed implemented'}
    (backend/'ditto_pairs.json').write_text(json.dumps(ditto,indent=2))
    (backend/'house_ditto_input.json').write_text(json.dumps({'schema':'HouseDitto-style-pairs-v1','pairs':pairs,'source':'https://github.com/UT-Austin-RPL/HouseDitto','integration_status':'observation pairs ready; backend inference not executed'},indent=2))
    (backend/'urdf_anything_plus_input.json').write_text(json.dumps({'optional':True,'states':groups,'object_level_3d_cues':'per-view metric point clouds, no generated geometry or GT articulation'},indent=2))
    memory=root/'structured_memory.json';estimated=json.loads(memory.read_text()) if memory.exists() else {}
    effort=root/'effort_summary.json'
    twin={'schema':'observed-interaction-twin-update-v1','object_id':capture['object_id'],
          'moving_part_id':capture['moving_part_id'],'articulation_frame':'calibrated world frame, meters; transform into source joint parent frame before URDF writeback','estimated_articulation':estimated.get('estimated_articulation'),
          'observed_range_estimated':[min([g['estimated_state'] for g in groups],default=0),max([g['estimated_state'] for g in groups],default=0)],
          'observed_range_units':capture['units'],'full_joint_limit_inferred':False,
          'effort':json.loads(effort.read_text()) if effort.exists() else None,
          'reconstruction_inputs':'reconstruction/art_input.json','geometry_updated':False,
          'reconstruction_status':'not run; observation packages and estimated structure ready',
          'source_model_overwritten':False}
    (root/'twin_update.json').write_text(json.dumps(twin,indent=2))
    return {'states':len(groups),'pairs':len(pairs),'art_ready':bool(groups),'ditto_tensor_ready':bool(pairs)}
