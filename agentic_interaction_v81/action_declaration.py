"""Change a finite action declaration without regenerating an unchanged grasp.

This is a no-motion Agent operation. Native measured-state execution validation
still runs, and each actual successor is checked by the frozen controller.
"""
import hashlib
import json


def ticket(ops):
    from interaction_runtime.recovery_candidates_v7 import observation_key
    path = ops.r.output/'visual_plan.json'
    return dict(plan_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                observation_sha256=observation_key(ops.g['agent_observations']),
                station_generation=ops.r.base_generation,
                physical_acquisitions=ops.acquisitions)


def redeclare(ops, command):
    from interaction_runtime.segments import validate_action
    if not ops.current_plan_ready or not getattr(ops, '_v81_plan_ticket', None):
        return dict(declared=False, reason='CURRENT_UNEXECUTED_AUTOMATIC_PLAN_REQUIRED')
    if ticket(ops) != ops._v81_plan_ticket:
        return dict(declared=False, reason='PLAN_OBSERVATION_STATION_OR_ACQUISITION_CHANGED')
    provenance = ops.r.output/'V8_PLAN_INSPECTION_PROVENANCE.json'
    if not provenance.exists():
        return dict(declared=False, reason='INSPECT_CURRENT_NATIVE_GRIPPER_PLAN_FIRST')
    inspected = json.loads(provenance.read_text())
    image = ops.r.output/'AGENT_GRASP_PLAN_INSPECTION.png'
    if (inspected.get('plan_sha256') != ops._v81_plan_ticket['plan_sha256']
            or not image.exists()
            or inspected.get('sha256') != hashlib.sha256(image.read_bytes()).hexdigest()
            or command.get('inspected_image_sha256') != inspected.get('sha256')):
        return dict(declared=False, reason='STALE_OR_UNINSPECTED_PLAN_IMAGE')
    actions = command['segments']
    plan = json.loads((ops.r.output/'visual_plan.json').read_text())
    states = [float(p['state']) for p in plan['path']]
    for action in actions:
        validate_action(action)
        ops.articulation().validate_target(action)
        if not min(states) <= action['target_q'] <= max(states):
            return dict(declared=False, reason='ACTION_EXCEEDS_EXISTING_CHECKED_TASK_ARC')
    checked = ops.validate_execution()
    if not checked.get('valid'):
        return dict(declared=False, reason='MEASURED_STATE_EXECUTION_REVALIDATION_FAILED', check=checked)
    prismatic = ops.r.job['articulation_v7']['joint_type'] == 'prismatic'
    if prismatic and 'v75_declared_chain' not in ops.g:
        return dict(declared=False, reason='ORIGINAL_PRISMATIC_DECLARATION_MISSING')
    # The unchanged V7.5 owner independently checks this declaration, then
    # reruns native whole-chain validation and admission at actual acquisition.
    # Synchronize its input; never bypass its attempt/closing/load checks.
    if prismatic:ops.g['v75_declared_chain'] = actions
    ops._v81_declared_chain = actions
    return dict(declared=True, segments=actions, robot_motion_executed=False,
                reused_only_current_unexecuted_measured_geometry_plan=True,
                plan_sha256=ops._v81_plan_ticket['plan_sha256'],
                measured_state_revalidated=True,
                original_prismatic_declaration_synchronized=prismatic,
                each_successor_requires_original_live_check=True,
                scores_controllers_physics_and_safety_unchanged=True)
