"""Called only after world.pause and GT gate opening; no actor readback."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def summarize(root,gt,kind):
    root=Path(root);p=root/'evaluation_private/object_trajectory.jsonl'
    axis=np.asarray(gt['axis_world']);previous=None;value=0.;maximum=0.;count=0;states={}
    manifest=root/'multistate_capture.json'
    capture=json.loads(manifest.read_text()) if manifest.exists() else {'states':[]}
    queries=sorted((float(v.get('timestamp_sim_s',s['timestamp_sim_s'])),s['state_id'],v['view_id']) for s in capture['states'] for v in s['views'] if not v.get('acquired_in_current_execution') is False)
    query_index=0;view_states={}
    if p.exists():
        with p.open() as stream:
            for line in stream:
                row=json.loads(line);T=np.asarray(row['T_object'])
                if previous is not None:
                    if kind=='revolute':value+=float(np.rad2deg(Rotation.from_matrix(T[:3,:3]@previous[:3,:3].T).as_rotvec()@axis))
                    else:value+=float((T[:3,3]-previous[:3,3])@axis)
                previous=T;count+=1;maximum=max(maximum,value)
                while query_index<len(queries) and queries[query_index][0]<=row['t']:
                    qt,sid,vid=queries[query_index];view_states[(sid,vid)]={'actual_state':value,'evaluation_timestamp_sim_s':row['t'],'capture_timestamp_sim_s':qt}
                    query_index+=1
                states[row['phase']]={'last_actual_state':value,'maximum_actual_state':max(value,states.get(row['phase'],{}).get('maximum_actual_state',0))}
    doc={'evaluation_only':True,'access_after_world_pause':True,'source':str(p),'maximum_actual_state':maximum,'final_actual_state':value,'units':'degrees' if kind=='revolute' else 'meters','samples':count,'phases':states,'not_returned_to_online_controller':True}
    for state in capture['states']:
        measured=[]
        for view in state['views']:
            audit=view_states.get((state['state_id'],view['view_id']))
            if audit:
                view['post_stop_actual_state']=audit;measured.append(audit['actual_state'])
        if measured:
            state['post_stop_actual_state_range']=[min(measured),max(measured)];state['post_stop_actual_state']=float(np.median(measured))
    if manifest.exists():manifest.write_text(json.dumps(capture,indent=2))
    doc['capture_states']=[{'state_id':s['state_id'],'actual_state_range':s.get('post_stop_actual_state_range'),'evaluated_views':sum('post_stop_actual_state' in v for v in s['views'])} for s in capture['states']]
    (root/'maximum_range_evaluation.json').write_text(json.dumps(doc,indent=2));return doc
