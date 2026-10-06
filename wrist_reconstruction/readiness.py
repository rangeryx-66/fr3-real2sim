"""Both-object backend gate and honest independent task result tables."""
import json,csv,subprocess,sys
from pathlib import Path

def reuse_closed_capture(root,sources):
    """Merge existing real observations only, after the actor has stopped.

    A matching scene initialization is required. No simulator state is restored
    and acquisition timestamps/camera/robot poses are retained verbatim.
    """
    import copy,shutil
    root=Path(root);manifest=root/'multistate_capture.json';jobfile=root/'frozen_wrist_job.json'
    if not manifest.exists() or not jobfile.exists():return False
    doc=json.loads(manifest.read_text());job=json.loads(jobfile.read_text())
    if any(s.get('clean_wrist_capture') and abs(s['estimated_articulation_state'])<1e-8 and len(s.get('views',[]))>=8 for s in doc['states']):return True
    checks=('asset_root','source','camera_calibration','frozen_proxy_sha256','initial_articulation_rad','plant','fixed_fixture')
    audits=[]
    for source in sources:
        source=Path(source);audit={'source':str(source),'physical_state_restore':False};audits.append(audit)
        try:
            oldjob=json.loads((source/'frozen_wrist_job.json').read_text());old=json.loads((source/'multistate_capture.json').read_text())
            if any(oldjob.get(k)!=job.get(k) for k in checks):raise ValueError('INITIAL_SCENE_OR_SENSOR_CONFIGURATION_DIFFERS')
            if str(old['object_id'])!=str(doc['object_id']) or old.get('capture_mode')!='wrist_camera_capture':raise ValueError('NOT_SAME_WRIST_OBJECT')
            state=next(s for s in old['states'] if s.get('clean_wrist_capture') and abs(s['estimated_articulation_state'])<1e-8 and len(s['views'])>=8)
            policy=job['wrist_experiment']['capture']
            for v in state['views']:
                folder=source/v['directory'];meta=json.loads((folder/'camera.json').read_text());qa=meta['qa']
                if qa['robot_pixel_ratio']>policy['maximum_robot_pixel_ratio'] or qa['object_frame_clipped'] or qa['visible_moving_part_pixels']<policy['minimum_moving_part_pixels']:raise ValueError('SOURCE_QA_INVALID')
                if not policy['minimum_object_coverage']<=qa['object_coverage']<=policy['maximum_object_coverage']:raise ValueError('SOURCE_COVERAGE_INVALID')
                if not all((folder/n).exists() for n in ('rgb.png','depth_m.npy','mask.png','point_cloud.npz')):raise ValueError('SOURCE_FILES_MISSING')
            state=copy.deepcopy(state);state['state_id']=max([s['state_id'] for s in doc['states']] or [-1])+1
            state['observation_reused_from']=str(source);state['acquired_in_current_execution']=False
            for v in state['views']:
                src=source/v['directory'];dst=root/'states'/f"state_{state['state_id']:03d}"/f"view_{v['view_id']:02d}"
                dst.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(src,dst,dirs_exist_ok=True)
                v.update(directory=str(dst.relative_to(root)),acquisition_reused_from=str(src),acquired_in_current_execution=False)
            doc['states'].append(state);doc['states'].sort(key=lambda s:s['estimated_articulation_state'])
            # The existing ArtGS adapter addresses manifest positions using
            # state_id. Preserve acquisition IDs separately when merging.
            for index,s in enumerate(doc['states']):
                s.setdefault('acquisition_state_id',s['state_id']);s['state_id']=index
            manifest.write_text(json.dumps(doc,indent=2));audit['status']='REUSED_REAL_CLOSED_OBSERVATIONS';break
        except (OSError,ValueError,KeyError,StopIteration) as error:audit.update(status='NOT_REUSED',reason=str(error))
    (root/'closed_observation_reuse.json').write_text(json.dumps(audits,indent=2))
    return any(a.get('status')=='REUSED_REAL_CLOSED_OBSERVATIONS' for a in audits)

def both_ready(out,c):
    rows=[]
    for o in c['objects']:
        root=Path(out)/('capture_'+o['id']);p=root/'multistate_capture.json'
        reuse=Path(out)/'supplemental_closed_sources.json'
        if (root/'report.json').exists() and reuse.exists():
            reuse_closed_capture(root,json.loads(reuse.read_text()).get(o['id'],[]))
        states=json.loads(p.read_text())['states'] if p.exists() else []
        clean=[s for s in states if s.get('clean_wrist_capture') and len(s.get('views',[]))>=8]
        span=max([s['estimated_articulation_state'] for s in clean] or [0])-min([s['estimated_articulation_state'] for s in clean] or [0])
        required=c['backend']['minimum_pair_span']['revolute' if o['id']=='7320' else 'prismatic']
        evaluation=Path(out)/('capture_'+o['id'])/'maximum_range_evaluation.json'
        actual=json.loads(evaluation.read_text()).get('maximum_actual_state') if evaluation.exists() else None
        # Label span alone cannot stand in for actual physical object motion.
        ready=len(clean)>=c['backend']['minimum_clean_states'] and span>=required and actual is not None and actual>=required
        rows.append({'object':o['id'],'clean_states':len(clean),'label_span':span,'actual_maximum':actual,'ready':ready})
    return all(r['ready'] for r in rows),rows

def summarize(out):
    lines=['','## Maximum-range physical results (not reconstruction acceptance)','']
    for obj in ['7320','45746']:
        root=Path(out)/('capture_'+obj)
        def read(name,default):
            p=root/name;return json.loads(p.read_text()) if p.exists() else default
        if (root/'effort_segments.json').exists() and (root/'effort_samples.jsonl').exists():
            interpreter=Path(__file__).resolve().parents[1]/'environments/artgs/bin/python'
            command=[str(interpreter) if interpreter.exists() else sys.executable,'-c','from wrist_reconstruction.effort_summary import summarize;import sys;summarize(sys.argv[1])',str(root)]
            subprocess.run(command,cwd=Path(__file__).resolve().parents[1],check=True,timeout=300)
        actual=read('maximum_range_evaluation.json',{});history=read('reposition_history.json',[])
        capture=read('multistate_capture.json',{});effort=read('effort_segments.json',[]);moves=read('mobile_progress.json',{}).get('repositions',0)
        lines += [f'### {obj}','',f"Actual maximum: {actual.get('maximum_actual_state','PENDING independent post-run evaluation')}; units: {'degrees' if obj=='7320' else 'meters'}. Mobile moves: {moves}; regrasp successes: {sum(bool(x.get('regrasp_completed')) for x in history)}; effort-valid segments: {sum(x.get('effort_validity')=='VALID_EFFORT_SEGMENT' for x in effort)}.",'','| State | Release | Scan | Wrist reobserve | Regrasp | Subsequent pull |','|---|---|---|---|---|---|']
        rows=[]
        for h in history:
            row={'state_estimate':h.get('start_state_estimated'),'release':h.get('released',False),'scan':h.get('wrist_scan_completed','not requested'),'reobserve':any('current_wrist_observation' in a for a in h.get('regrasp_attempts',[])),'regrasp':h.get('regrasp_completed',False),'subsequent_pull':'see observed_progress.jsonl and evaluation trajectory','status':h.get('status')}
            rows.append(row);lines.append('| '+' | '.join(str(row[k]) for k in ['state_estimate','release','scan','reobserve','regrasp','subsequent_pull'])+' |')
        if root.exists():
            with (root/'state_table.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=['state_estimate','release','scan','reobserve','regrasp','subsequent_pull','status']);w.writeheader();w.writerows(rows)
        lines+=['',f"Clean captures: {[(s.get('estimated_articulation_state'),len(s.get('views',[]))) for s in capture.get('states',[]) if s.get('clean_wrist_capture')]}."]
    return lines
