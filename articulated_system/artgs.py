"""Actual official two-state loader; calibrated sensor data, no reference model."""
import json,hashlib
from pathlib import Path
import numpy as np
from PIL import Image


def prepare(capture_root, output, start, end, size=320):
    import cv2
    from plyfile import PlyData,PlyElement
    root=Path(capture_root).resolve();out=Path(output);out.mkdir(parents=True,exist_ok=True)
    doc=json.loads((root/'multistate_capture.json').read_text())
    if start==end:raise ValueError('TWO_DISTINCT_STATES_REQUIRED')
    states=[doc['states'][start],doc['states'][end]]
    if abs(states[1]['estimated_articulation_state']-states[0]['estimated_articulation_state'])<1e-6:raise ValueError('NO_OBSERVED_STATE_CHANGE')
    maps=[{v.get('view_id',i):v for i,v in enumerate(s['views'])} for s in states]
    common_ids=sorted(set(maps[0])&set(maps[1]))
    if len(common_ids)<2:raise ValueError('INSUFFICIENT_COMMON_CALIBRATED_VIEWS')
    # Hold out the SAME physical camera IDs in both states. Occluded views
    # cannot silently shift list indices and leak a reserved camera into train.
    held=common_ids[-2:] if len(common_ids)>=6 else [];train=[i for i in common_ids if i not in held]
    clouds=[];cloud_colors=[]
    for views in maps:
        P=[];C=[]
        for i in train:
            data=np.load(root/views[i]['directory']/'point_cloud.npz')
            if str(data['units'])!='m':raise ValueError('POINT_CLOUD_UNITS_NOT_METERS')
            P.append(data['points_world_m']);C.append(data['rgb'])
        clouds.append(np.concatenate(P));cloud_colors.append(np.concatenate(C))
    # Reserved camera geometry does not enter even the normalization prior.
    lo=np.minimum(clouds[0].min(0),clouds[1].min(0));hi=np.maximum(clouds[0].max(0),clouds[1].max(0));center=(lo+hi)/2;scale=float((hi-lo).max()*1.1)
    if not np.isfinite(scale) or scale<=0:raise ValueError('INVALID_METRIC_SCALE')
    v0=maps[0][train[0]]
    with Image.open(root/v0['directory']/'rgb.png') as image:w,h=image.size
    # Preserve the full calibrated rectangular sensor FOV. A centered square
    # crop removed door/drawer extremities in the old wrist-like inputs.
    output_w=int(size);output_h=max(1,int(round(size*h/w)))
    common=np.asarray(v0['K'],float).copy();common[0]*=output_w/w;common[1]*=output_h/h
    audit=[];yy,xx=np.indices((output_h,output_w),dtype=np.float32)
    for label,state,views,P,C in zip(['start','end'],states,maps,clouds,cloud_colors):
        for split,ids in [('train',train),('test',held or train[:1])]:
            frames=[];folder=out/label/split;(folder/'rgba').mkdir(parents=True,exist_ok=True);(folder/'depth').mkdir(exist_ok=True)
            for i in ids:
                v=views[i];src=root/v['directory'];meta=json.loads((src/'camera.json').read_text())
                if 'optical Z' not in meta['depth']:raise ValueError('UNVERIFIED_DEPTH_CONVENTION')
                rgb=np.asarray(Image.open(src/'rgb.png'));mask=np.asarray(Image.open(src/'mask.png'))>0;depth=np.load(src/'depth_m.npy');K=np.asarray(v['K'])
                mask &= np.isfinite(depth)&(depth>.02)&(depth<3.)
                mx=(xx-common[0,2])/common[0,0]*K[0,0]+K[0,2];my=(yy-common[1,2])/common[1,1]*K[1,1]+K[1,2]
                rgb=cv2.remap(rgb,mx.astype(np.float32),my.astype(np.float32),cv2.INTER_LINEAR);mask=cv2.remap(mask.astype(np.uint8),mx.astype(np.float32),my.astype(np.float32),cv2.INTER_NEAREST)>0
                d=cv2.remap(depth.astype(np.float32),mx.astype(np.float32),my.astype(np.float32),cv2.INTER_NEAREST);d[~mask|~np.isfinite(d)]=0.;d/=scale
                key=f'{i:04d}';Image.fromarray(np.dstack([rgb,mask.astype(np.uint8)*255])).save(folder/'rgba'/f'{key}.png')
                Image.fromarray(np.clip(np.rint(d*1000),0,65535).astype(np.uint16)).save(folder/'depth'/f'{key}.png')
                T=np.asarray(v['T_world_camera_optical'])@np.diag([1,-1,-1,1]);T[:3,3]=(T[:3,3]-center)/scale
                frames.append({'file_path':f'{label}/{split}/rgba/{key}.png','transform_matrix':T.tolist(),'time':0 if label=='start' else 1})
                audit.append({'state':state['state_id'],'camera_id':i,'split':split,'visible_pixels':int(mask.sum()),'masked_robot':meta.get('mask_source','').startswith('instance') or doc.get('mask_source','').startswith('simulator'),'depth_convention':meta['depth']})
            camera={'camera_angle_x':float(2*np.arctan(output_w/(2*common[0,0]))),'camera_angle_y':float(2*np.arctan(output_h/(2*common[1,1]))),'frames':frames}
            (out/f'transforms_{split}_{label}.json').write_text(json.dumps(camera,indent=2))
        P=(P-center)/scale;_,idx=np.unique(np.floor(P/.003).astype(np.int64),axis=0,return_index=True)
        if len(idx)>30000:idx=np.random.default_rng(61).choice(idx,30000,replace=False)
        array=np.empty(len(idx),dtype=[(x,'f4') for x in ['x','y','z','nx','ny','nz']]+[(x,'u1') for x in ['red','green','blue']])
        for j,x in enumerate(['x','y','z']):array[x]=P[idx,j]
        for x in ['nx','ny','nz']:array[x]=0
        for j,x in enumerate(['red','green','blue']):array[x]=C[idx,j]
        PlyData([PlyElement.describe(array,'vertex')],text=False).write(str(out/f'point_cloud_{label}.ply'))
    (out/'transforms_train.json').write_text((out/'transforms_train_start.json').read_text());(out/'points3d.ply').write_bytes((out/'point_cloud_start.ply').read_bytes())
    provenance={'adapter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'backend':'ArtGS official Scene/readInfo_2states','capture_root':str(root),'states':[start,end],'normalization':{'world_center_m':center.tolist(),'world_scale_m':scale,'world_to_normalized':'(point-center)/scale','computed_from_training_views_only':True},'train_views':train,'held_out_views':held,'view_selection':'shared physical camera IDs, not shifted list positions','independent_view_validation_available':bool(held),'camera_axes':'OpenGL from ROS optical','common_intrinsics':common.tolist(),'output_resolution_wh':[output_w,output_h],'intrinsics_adapter':'full rectangular sensor FOV; calibrated rays resampled to common K; no central crop','depth_storage':'normalized optical Z *1000 uint16; official loader /1000','initialization':'separate per-state training clouds','observations_from_distinct_states_merged':False,'GT_mesh_or_axis_input':False,'joint_family_input':'official train_predict; optional measured-EE family separately labelled','parts_assumption':2,'audit':audit}
    (out/'input_provenance.json').write_text(json.dumps(provenance,indent=2));return provenance
