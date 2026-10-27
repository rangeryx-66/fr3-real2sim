"""Turn measured knowledge into the actual existing ABC command fields.

No plant files, native physics readbacks or oracle parameters are read here.
Native planning, measured feedback and independent guards still own admission.
"""
from copy import deepcopy
import hashlib,json,math
from .physics_aware import DemandModel,Capability,choose,rest_to_rest_time


FIELDS=('requested_preload_n','reference_speed_m_s','reference_acceleration_m_s2')


def decision(method, measured, menu, *, distance, motion, sustained_pad_n, emergency_wrist_n):
    if method not in ('reactive','stale','updated'):
        raise ValueError('FORMAL_MEASURED_METHOD_REQUIRED')
    if (measured.get('schema')!='physics-aware-measured-input/v1'
            or measured.get('hidden_dynamics_used') is not False
            or measured.get('validation_used_for_fit_or_selection') is not False):
        raise ValueError('SEALED_MEASUREMENT_INPUT_REQUIRED')
    capabilities=[Capability(**row) for row in measured['contact_capabilities']]
    requests=menu['qualified_requested_preloads_n']
    if any(c.request_n not in requests for c in capabilities):
        raise ValueError('UNQUALIFIED_PRELOAD_CAPABILITY')
    speeds=menu['reference_speeds_m_s'];accelerations=menu['reference_accelerations_m_s2']
    if (not speeds or not accelerations or not requests
            or not all(type(x) in (int,float) and math.isfinite(x) and x>0
                for x in list(speeds)+list(accelerations)+list(requests))
            or max(speeds)>motion['speed_m_s'] or max(accelerations)>motion['acceleration_m_s2']):
        raise ValueError('FINITE_COMMON_NATIVE_CONTROL_MENU_REQUIRED')
    qualified=[c for c in capabilities if c.source and c.transmitted_axis_force_lower_n>0
        and len(c.measured_pad_normal_upper_n)==2
        and max(c.measured_pad_normal_upper_n)<sustained_pad_n]
    if not qualified:raise ValueError('NO_MEASURED_SAFE_CONTACT_CAPABILITY')
    # Strong reactive arm: fastest common qualified reference and greatest
    # measured usable grip. Normal F/T/slip/reference feedback is unchanged.
    grip=max(qualified,key=lambda c:(c.transmitted_axis_force_lower_n,c.request_n))
    reactive=dict(requested_preload_n=float(grip.request_n),reference_speed_m_s=float(max(speeds)),
        reference_acceleration_m_s2=float(max(accelerations)))
    action=reactive;selection=None;fallback=None
    if method!='reactive':
        if measured.get('model_usable') is not True:
            fallback='UNSUPPORTED_MODEL_USE_SAME_REACTIVE_CONTROL'
        else:
            model=DemandModel(**measured['demand_model'])
            selection=choose(model,capabilities,distance=distance,speeds=speeds,accelerations=accelerations,
                motion=motion,sustained_pad_n=sustained_pad_n,emergency_wrist_n=emergency_wrist_n)
            if selection['action'] is None:fallback='NO_PREDICTED_FEASIBLE_ACTION_USE_REACTIVE_AND_OBSERVE'
            else:action={k:selection['action'][k] for k in FIELDS}
    digest=lambda value:hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return dict(schema='physics-aware-control-decision/v1',method=method,action=action,
        input_sha256=digest(measured),common_menu_sha256=digest(menu),model_selection=selection,
        fallback=fallback,model_changed_control=any(action[k]!=reactive[k] for k in FIELDS),
        planned_rest_to_rest_s=rest_to_rest_time(distance,action[FIELDS[1]],action[FIELDS[2]]),
        ordinary_failure_ends_episode=False,original_reactive_feedback_and_hard_guards=True,
        actual_native_path_admission_required=True,GT_used_for_control=False)


def prepare_command(result, heldout_segments, *, inspected_image_sha256, reason):
    """Create the real prepare_grasp declaration consumed by existing owners."""
    if result.get('schema')!='physics-aware-control-decision/v1' or not heldout_segments or not reason:
        raise ValueError('DECLARED_POLICY_AND_INDEPENDENT_TASK_REQUIRED')
    segments=deepcopy(heldout_segments);action=result['action']
    for row in segments:
        if not row['role'].startswith('VALIDATION') or 'fit_intervals_s' in row:
            raise ValueError('FORMAL_OPERATION_MUST_NOT_UPDATE_FIT')
        row.update({k:action[k] for k in FIELDS[1:]})
    return dict(operation='prepare_grasp',inspected_image_sha256=inspected_image_sha256,
        planned_segments=segments,planning_target_q=max(s['target_q'] for s in segments),
        grasp_family='existing_bar',native_g1_preload_request_n=action['requested_preload_n'],reason=reason)


def acquire_command(prepared, *, inspected_plan_sha256, reason):
    """Use exactly the selected preload and declared reference in physical B."""
    return dict(operation='acquire_execute',inspected_image_sha256=inspected_plan_sha256,
        segments=deepcopy(prepared['planned_segments']),
        native_g1_preload_request_n=prepared['native_g1_preload_request_n'],retain_contact=False,reason=reason)
