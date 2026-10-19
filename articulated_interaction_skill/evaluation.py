"""Post-run multistate check. Simulator link poses never feed the controller."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def evaluate_capture(output):
    root=Path(output);path=root/'evaluation_private/object_trajectory.json'
    if not path.exists():return {'status':'EVALUATION_LOG_UNAVAILABLE'}
    doc=json.loads((root/'multistate_capture.json').read_text());rows=json.loads(path.read_text());states=doc['states']
    if not states or not rows:return {'status':'NO_CAPTURED_STATE'}
    t=np.array([r['t'] for r in rows]);T=np.array([r['T_object'] for r in rows]);E=np.array([r['T_ee'] for r in rows])
    i0=int(np.argmin(abs(t-states[0]['timestamp_sim_s'])));Q0=Rotation.from_matrix(T[i0,:3,:3]);P0=T[i0,:3,3]
    relation=np.linalg.inv(T[i0])@E[i0];result=[]
    for s in states:
        i=int(np.argmin(abs(t-s['timestamp_sim_s'])))
        rotation=float(np.rad2deg((Rotation.from_matrix(T[i,:3,:3])*Q0.inv()).magnitude()))
        translation=float(np.linalg.norm(T[i,:3,3]-P0));relative=np.linalg.inv(T[i])@E[i]
        result.append({'state_id':s['state_id'],'estimated_state':s['estimated_articulation_state'],
                       'actual_relative_rotation_deg':rotation,'actual_relative_translation_m':translation,
                       'relative_gripper_part_translation_drift_m':float(np.linalg.norm(relative[:3,3]-relation[:3,3])),
                       'evaluation_time_error_s':float(abs(t[i]-s['timestamp_sim_s']))})
    field='actual_relative_rotation_deg' if doc['joint_family_requested']=='revolute' else 'actual_relative_translation_m'
    minimum=.5 if field.endswith('deg') else .002
    distinct=[]
    for r in result:
        if not distinct or abs(r[field]-distinct[-1][field])>=minimum:distinct.append(r)
    evaluation={'evaluation_only':True,'read_after_execution':True,'source':'independent write-only object SE(3) logger',
                'states':result,'physically_distinct_captured_states':len(distinct),
                'three_state_capture_verified':len(distinct)>=3,'joint_state_read_for_control':False}
    (root/'capture_evaluation.json').write_text(json.dumps(evaluation,indent=2));return evaluation
