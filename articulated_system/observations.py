"""Calibrated sensor-only input for the real PARIS loader (two static states)."""
import json
from pathlib import Path
import numpy as np
from PIL import Image


def prepare_paris(capture_root, output, start, end, train_views=None, size=320):
    root=Path(capture_root);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    capture=json.loads((root/'multistate_capture.json').read_text())
    if start==end:raise ValueError('TWO_DISTINCT_STATES_REQUIRED')
    states=[capture['states'][start],capture['states'][end]]
    if abs(states[1]['estimated_articulation_state']-states[0]['estimated_articulation_state'])<1e-6:raise ValueError('NO_OBSERVED_STATE_CHANGE')
    # Never fuse observations from different articulated states into one cloud.
    clouds=[]
    for state in states:
        clouds.append(np.concatenate([np.load(root/v['directory']/'point_cloud.npz')['points_world_m'] for v in state['views']]))
    lo=np.minimum(clouds[0].min(0),clouds[1].min(0));hi=np.maximum(clouds[0].max(0),clouds[1].max(0))
    center=(lo+hi)/2;scale=float((hi-lo).max()*1.1)
    if scale<=0:raise ValueError('INVALID_METRIC_SCALE')
    available=min(len(s['views']) for s in states)
    train=list(range(available)) if train_views is None else list(train_views)
    held=[i for i in range(available) if i not in train]
    flip=np.diag([1.,-1.,-1.,1.])
    audit=[]
    for label,state,cloud in zip(['start','end'],states,clouds):
        for split,indices in [('train',train),('val',held or train[:1]),('test',held or train[:1])]:
            camera={};folder=out/label/split;folder.mkdir(parents=True,exist_ok=True)
            for i in indices:
                v=state['views'][i];src=root/v['directory'];K=np.array(v['K'],float)
                rgb=np.asarray(Image.open(src/'rgb.png'));mask=np.asarray(Image.open(src/'mask.png'))>0
                depth=np.load(src/'depth_m.npy');meta=json.loads((src/'camera.json').read_text())
                if 'optical Z' not in meta['depth']:raise ValueError('UNVERIFIED_DEPTH_CONVENTION')
                mask &= np.isfinite(depth)&(depth>.02)&(depth<3.)
                # Render instance mask excludes robot/table: invisible surface
                # holes remain holes, never filled with reference/GT geometry.
                h,w=mask.shape;side=min(h,w);left=(w-side)//2;top=(h-side)//2
                rgba=np.dstack((rgb,mask.astype(np.uint8)*255))[top:top+side,left:left+side]
                Image.fromarray(rgba).resize((size,size),Image.Resampling.LANCZOS).save(folder/f'{i:04d}.png')
                K[0,2]-=left;K[1,2]-=top;K[:2]*=size/side
                if not np.allclose(K[0,0],K[1,1],rtol=1e-3) or not np.allclose(K[:2,2],size/2,atol=1.):raise ValueError('PARIS_LOADER_REQUIRES_CENTERED_SQUARE_PIXELS')
                camera['K']=K.tolist();T=np.array(v['T_world_camera_optical'],float)@flip
                T[:3,3]=(T[:3,3]-center)/scale;camera[f'{i:04d}']=T.tolist()
                audit.append({'state':state['state_id'],'view':i,'split':split,'visible_pixels':int(mask.sum()),'masked_robot':True,'depth_convention':meta['depth']})
            (out/label/f'camera_{split}.json').write_text(json.dumps(camera,indent=2))
    provenance={'backend':'official PARIS blender_paris loader','capture_root':str(root.resolve()),'states':[start,end],
                'normalization':{'world_center_m':center.tolist(),'world_scale_m':scale,'world_to_normalized':'(point-center)/scale'},
                'camera_axes':'PARIS/OpenGL x right,y up,z backward; converted from ROS optical',
                'train_views':train,'held_out_views':held,'independent_view_validation_available':bool(held),
                'observations_from_distinct_states_merged':False,'GT_mesh_or_axis_input':False,'audit':audit}
    (out/'input_provenance.json').write_text(json.dumps(provenance,indent=2));return provenance
