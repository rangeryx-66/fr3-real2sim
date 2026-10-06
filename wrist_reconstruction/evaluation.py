"""Called only after world.pause and GT gate opening; no actor readback."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def summarize(root,gt,kind):
    root=Path(root);p=root/'evaluation_private/object_trajectory.jsonl'
    axis=np.asarray(gt['axis_world']);previous=None;value=0.;maximum=0.;count=0;states={}
    if p.exists():
        with p.open() as stream:
            for line in stream:
                row=json.loads(line);T=np.asarray(row['T_object'])
                if previous is not None:
                    if kind=='revolute':value+=float(np.rad2deg(Rotation.from_matrix(T[:3,:3]@previous[:3,:3].T).as_rotvec()@axis))
                    else:value+=float((T[:3,3]-previous[:3,3])@axis)
                previous=T;count+=1;maximum=max(maximum,value)
                states[row['phase']]={'last_actual_state':value,'maximum_actual_state':max(value,states.get(row['phase'],{}).get('maximum_actual_state',0))}
    doc={'evaluation_only':True,'access_after_world_pause':True,'source':str(p),'maximum_actual_state':maximum,'final_actual_state':value,'units':'degrees' if kind=='revolute' else 'meters','samples':count,'phases':states,'not_returned_to_online_controller':True}
    (root/'maximum_range_evaluation.json').write_text(json.dumps(doc,indent=2));return doc
