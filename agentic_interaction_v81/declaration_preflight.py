"""Cheap complete-chain checks before any reconstruction or candidate search."""
def check(actions, articulation):
    from agent_command_protocol_v12 import validate
    from interaction_runtime.segments import validate_action
    if not isinstance(actions, list) or not actions:
        raise ValueError('PREDECLARE_FINITE_NATIVE_COORDINATE_CHAIN')
    # Reuse the real execution protocol: roles, finite profiles, <=4 actions,
    # FIT intervals and VALIDATION exclusion must agree with the eventual call.
    validate(dict(protocol_version=1, sequence=0,
                  operation='attempt_grasp_and_probe', segments=actions), 0)
    for action in actions:
        validate_action(action)
        articulation.validate_target(action)
        from .replanning import alternatives
        for alternative in alternatives(action):
            validate_action(alternative);articulation.validate_target(alternative)
    if not any(a['role'].startswith('VALIDATION') for a in actions):
        raise ValueError('COMPLETE_VALIDATION_NOT_PLANNED')
    return dict(valid=True, actions=len(actions), robot_motion_executed=False,
                geometry_or_candidate_search_started=False)
