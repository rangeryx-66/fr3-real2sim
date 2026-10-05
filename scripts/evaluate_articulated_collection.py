"""Post-stop physical metrics from the independent write-only object logger."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def evaluate(root):
    root=Path(root)
    if not (root/'report.json').exists():raise RuntimeError('EPISODE_NOT_FINISHED')
    scope=json.loads((root/'evaluation_private/scope.json').read_text())
    if not scope.get('evaluation_only') or scope.get('controller_readback'):raise RuntimeError('EVALUATION_SCOPE_INVALID')
    rows=json.loads((root/'evaluation_private/object_trajectory.json').read_text());capture=json.loads((root/'multistate_capture.json').read_text());report=json.loads((root/'report.json').read_text())
    if not capture['states']:return {'status':'NO_STABLE_CAPTURED_GRASP'}
    times=np.asarray([row['t'] for row in rows])
    first_index=int(np.argmin(abs(times-capture['states'][0]['timestamp_sim_s'])))
    first_object=np.asarray(rows[first_index]['T_object']);state_evaluation=[]
    for state in capture['states']:
        index=int(np.argmin(abs(times-state['timestamp_sim_s'])));T=np.asarray(rows[index]['T_object'])
        state_evaluation.append({'state_id':state['state_id'],'estimated_state':state['estimated_articulation_state'],
            'actual_rotation_from_first_capture_deg':float(np.rad2deg(Rotation.from_matrix(T[:3,:3]@first_object[:3,:3].T).magnitude())),
            'actual_translation_from_first_capture_m':float(np.linalg.norm(T[:3,3]-first_object[:3,3])),
            'timestamp_alignment_error_s':float(abs(times[index]-state['timestamp_sim_s']))})
    start=capture['states'][0]['timestamp_sim_s'];active={'FINAL_HOLD','EXPLORATORY','ESTIMATED_FOLLOW','PROBE_HOLD','FORCE_HOLD'}
    reference=None;segments=[];current=[];released=False
    for row in rows:
        if row['t']<start:continue
        phase=row['phase']
        if phase.startswith('SYSTEM_RELEASE'):
            if current:segments.append(current);current=[]
            reference=None;released=True;continue
        if released and phase=='COMPLIANT_SETTLE':released=False
        if released or phase not in active:continue
        T=np.linalg.inv(np.asarray(row['T_object']))@np.asarray(row['T_ee'])
        if reference is None:reference=T.copy()
        current.append({'t':row['t'],'translation_drift_m':float(np.linalg.norm(T[:3,3]-reference[:3,3])),
                        'rotation_drift_deg':float(np.rad2deg(Rotation.from_matrix(T[:3,:3]@reference[:3,:3].T).magnitude()))})
    if current:segments.append(current)
    doc={'evaluation_only':True,'read_after_episode_report':True,'controller_readback':False,'GT_used_for_controller_or_grasp':False,
         'source':'independent object SE(3) + measured EE SE(3)','metric':'gripper-part relative transform; normal object opening not counted as slip',
         'capture_state_evaluation':state_evaluation,'state_evaluation_reference':'first actual captured object pose; evaluation only, never replaces estimated input labels',
         'grasp_epochs':[{'index':i,'start_s':s[0]['t'],'end_s':s[-1]['t'],'max_translation_drift_m':max(x['translation_drift_m'] for x in s),'max_rotation_drift_deg':max(x['rotation_drift_deg'] for x in s)} for i,s in enumerate(segments)],
         'maximum_true_relative_translation_drift_m':max((x['translation_drift_m'] for s in segments for x in s),default=None),
         'reported_final_object_displacement':report.get('evaluation',{}).get('actual_joint_displacement'),
         'minimum_joint_margin_rad':report.get('minimum_joint_margin_rad'),'physical_stop':report['status'],
         'limitation':'relative motion includes gripper compliance as well as slip; independent simulation diagnostic, unavailable as exact online robot measurement'}
    (root/'physical_collection_evaluation.json').write_text(json.dumps(doc,indent=2));return doc

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);print(json.dumps(evaluate(p.parse_args().capture),indent=2))
