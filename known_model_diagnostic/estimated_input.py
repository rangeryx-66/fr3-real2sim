"""Replace only motion geometry using an existing estimate; never refit it.
The diagnostic's frozen cooked scene remains explicit GT-dependent provenance.
This module does not turn that scene into a verified no-GT main-method run.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from interaction_identification.contact_probe import robot_only_model
from interactive_twin_recovery.mobile import scene_at
from wrist_reconstruction.geometry import calibration
from wrist_reconstruction.planner import camera_clearance


def delta_from_estimate(estimate, state):
    kind=estimate['joint_type'];data=estimate[kind]
    axis=np.asarray(data['axis'],float);axis/=np.linalg.norm(axis)
    D=np.eye(4)
    if kind=='prismatic':D[:3,3]=axis*float(state)
    else:
        R=Rotation.from_rotvec(axis*float(state)).as_matrix();point=np.asarray(data['point_on_axis'],float)
        D[:3,:3]=R;D[:3,3]=point-R@point
    return D


def compare(root,output,destination):
    """Freeze strategy/stances, replace references, and recheck entire paths."""
    root=Path(root);output=Path(output);destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    report=json.loads((output/'report.json').read_text())
    if not report['success']:raise RuntimeError('ONLY_FREEZE_ACTUALLY_SUCCESSFUL_COMPLETE_TRIAL')
    job=json.loads((output/'job.json').read_text());memory=json.loads(Path(job['operation_memory']).read_text())
    if memory.get('GT_inputs'):raise RuntimeError('ESTIMATE_PROVENANCE_CONTAINS_GT_INPUT')
    estimate=memory['estimated_articulation'];model=robot_only_model(root/'config/piper.urdf');cal=calibration(job['camera_calibration'])
    whole=json.loads((output/'whole_plan.json').read_text());export=json.loads((output/'cooked_initial.json').read_text())
    legs=[(whole['base'],output/'aligned_whole_path.json')]
    if 'second_station' in whole:legs.append((whole['second_station']['base'],output/'aligned_second_leg.json'))
    audit=[];plan=[];blockers=[];native_rows=json.loads((output/'observations.json').read_text())
    for base,path in legs:
        known=json.loads(path.read_text())['known_model_whole_path'];E0=np.asarray(known[0]['T_tcp']);s0=float(known[0]['state']);seed=np.asarray(known[0]['q'])
        scene=scene_at(root,export,model,whole['base'],base)
        # Frozen native evaluator pose at this leg's initial state: explicit
        # GT collision provenance, never mislabeled as an observed scene.
        native_start=min(native_rows,key=lambda r:abs((np.deg2rad(r['door_angle_deg']) if report['actual_units']=='degrees' else r['door_angle_deg'])-s0));B0=np.asarray(native_start['T_moving_link'])
        rows=[];errors=[]
        for saved in known:
            D=delta_from_estimate(estimate,float(saved['state'])-s0);target=D@E0
            q=model.ik(target,base,seed=seed,starts=1)
            diff=float(np.linalg.norm(target[:3,3]-np.asarray(saved['T_tcp'])[:3,3]));errors.append(diff)
            if q is None:blockers.append(dict(state=saved['state'],reason='ESTIMATED_INPUT_NO_CONTINUOUS_IK'));break
            if model.margin(q)<=.05 or np.max(abs(q-seed))>.2:blockers.append(dict(state=saved['state'],reason='ESTIMATED_INPUT_MARGIN_OR_BRANCH'));break
            ok,why=scene.check(model.poses(q,base,width=whole['grasp_aperture_reference_m']),D@B0,True)
            if ok:ok,why=camera_clearance(scene,model.poses(q,base,width=whole['grasp_aperture_reference_m']),cal)
            if not ok:blockers.append(dict(state=saved['state'],reason=why));break
            rows.append(dict(state=saved['state'],q=q.tolist(),T_tcp=target.tolist(),margin_rad=model.margin(q),reference_difference_m=diff));seed=q
        audit.append(dict(base=base,checked_states=len(rows),requested_states=len(known),maximum_reference_difference_m=max(errors,default=0.)))
        plan.append(dict(base=base,path=rows))
    result=dict(mode='ESTIMATED_MOTION_INPUT_ON_FROZEN_DIAGNOSTIC_SCENE',estimate_source=job['operation_memory'],
                estimation_refit=False,GT_motion_input=False,GT_cooked_collision_geometry=True,
                strategy_changed=False,stances_changed=False,controller_configuration_changed=False,
                physical_execution_verified=False,main_no_GT_method_integrated=False,end_escape_and_regrasp_recertified=False,
                audits=audit,blockers=blockers,
                limitation='Reference substitution and dense path samples only; end escape/regrasp are not recertified, so this is not an executable full-task certificate. Frozen native collision geometry remains GT-dependent. No estimated-mode physical range is claimed.')
    (destination/'input_difference.json').write_text(json.dumps(result,indent=2));(destination/'estimated_reference_plan.json').write_text(json.dumps(plan,indent=2));return result
