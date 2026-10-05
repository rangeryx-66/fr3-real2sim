"""Reuse actual initial-state observations; never restore physical state."""
import copy,json,shutil,hashlib
from pathlib import Path
import numpy as np
from articulated_interaction_skill.capture import backproject


def _closed_views(recorder,state_dir,value):
    source=recorder.job.get('resume_closed_capture')
    if not source or abs(value)>1e-8 or recorder.states:return []
    source=Path(source);audit={'source':str(source),'physical_state_restore':False,'object_commands':False,'scope':'observation reuse only; failure does not stop regrasp or fresh capture','views':[]}
    namespace=getattr(recorder,'_resume_namespace',0)
    def save(): (recorder.output/f'observation_resume_source_{namespace}.json').write_text(json.dumps(audit,indent=2))
    try:
        doc=json.loads((source/'multistate_capture.json').read_text())
        if str(doc['object_id'])!=str(recorder.metadata['asset_id']) or doc.get('capture_mode')!='wrist_camera_capture':raise ValueError('CHECKPOINT_NOT_SAME_WRIST_OBJECT')
        data=json.loads((source/'states/state_000/views_checkpoint.json').read_text())
        if abs(data['estimated_state'])>1e-8:raise ValueError('INITIAL_STATE_CHECKPOINT_REQUIRED')
        folder=source/'initial_sensor_observation';meta=json.loads((folder/'camera.json').read_text())
        prior,_=backproject(np.load(folder/'depth_m.npy'),np.array(meta['K']),np.array(meta['T_world_camera_optical']),np.load(folder/'sensor_mask.npy'),stride=4)
        from articulated_system.recovery import register
        D,fit=register(prior,recorder.initial_framing_cloud,np.eye(4))
        displacement=float(np.max(np.linalg.norm(prior@D[:3,:3].T+D[:3,3]-prior,axis=1)))
        audit.update(observed_registration=fit,observed_displacement_m=displacement)
        limit=recorder.config['capture']['resume_observation_max_displacement_m']
        if fit['fitness']<.95 or fit['rmse_m']>limit or displacement>limit:raise ValueError('SAME_OBSERVED_STATE_NOT_VERIFIED')
        salvage=source/'salvaged_actual_views.json'
        candidates=data['views']+(json.loads(salvage.read_text()).get('views',[]) if salvage.exists() else [])
        views=[];policy=recorder.config['capture']
        from wrist_reconstruction.geometry import distinct_view
        for original in candidates:
            src=source/original['directory'];m=json.loads((src/'camera.json').read_text());qa=m['qa']
            if not np.allclose(m['K'],recorder.cal['K'],atol=1e-3) or m['resolution_wh']!=recorder.cal['resolution_wh']:continue
            if qa['robot_pixel_ratio']>policy['maximum_robot_pixel_ratio'] or qa['object_frame_clipped'] or qa['visible_moving_part_pixels']<policy['minimum_moving_part_pixels']:continue
            if not policy['minimum_object_coverage']<=qa['object_coverage']<=policy['maximum_object_coverage']:continue
            if not all((src/n).exists() for n in ['rgb.png','depth_m.npy','mask.png','point_cloud.npz']):continue
            previous=getattr(recorder,'_resume_existing',[])+views
            if not distinct_view(np.array(original['T_world_camera_optical']),[np.array(v['T_world_camera_optical']) for v in previous],policy.get('minimum_camera_baseline_m',.04),policy.get('minimum_camera_angle_deg',8.)):continue
            view=copy.deepcopy(original);view['source_view_id']=view['view_id'];view['view_id']+=namespace*1000
            dst=state_dir/f"view_{view['view_id']:02d}";shutil.copytree(src,dst,dirs_exist_ok=True)
            view.update(directory=str(dst.relative_to(recorder.output)),acquisition_reused_from=str(src),acquired_in_current_execution=False)
            views.append(view);audit['views'].append({'view_id':view['view_id'],'source':str(src),'rgb_sha256':hashlib.sha256((src/'rgb.png').read_bytes()).hexdigest()})
            if len(previous)+1>=policy.get('maximum_clean_views',8):break
        audit.update(status='REUSED_ACTUAL_OBSERVATIONS',count=len(views));save();return views
    except Exception as error:
        audit.update(status='FRESH_CAPTURE_REQUIRED',reason=str(error));save();return []


def closed_views(recorder,state_dir,value):
    sources=recorder.job.get('resume_closed_capture')
    if not sources or abs(value)>1e-8 or recorder.states:return []
    if isinstance(sources,(str,Path)):sources=[sources]
    views=[];audits=[]
    for namespace,source in enumerate(sources):
        proxy=copy.copy(recorder);proxy.job=dict(recorder.job,resume_closed_capture=str(source));proxy._resume_namespace=namespace;proxy._resume_existing=views
        views.extend(_closed_views(proxy,state_dir,value))
        audits.append(json.loads((recorder.output/f'observation_resume_source_{namespace}.json').read_text()))
        if len(views)>=recorder.config['capture'].get('maximum_clean_views',8):break
    report={'status':'REUSED_ACTUAL_OBSERVATIONS' if views else 'FRESH_CAPTURE_REQUIRED','count':len(views),'physical_state_restore':False,'sources':audits,'scope':'observation reuse only; physical actions reexecuted; no regrasp veto'}
    (recorder.output/'observation_resume.json').write_text(json.dumps(report,indent=2));return views
