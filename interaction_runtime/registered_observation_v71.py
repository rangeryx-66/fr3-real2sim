"""Initial-image infrastructure repair; frozen V7 interaction owners unchanged.

An unavailable first survey route is not a physical attempt. Capture at the
measured, verified-unloaded wrist instead; Codex must inspect it and select any
subsequent view through the original evaluate/move operations. No camera target
or grasp is supplied by this fallback.
"""
import numpy as np
from agent_command_protocol_v12 import atomic_json


def initial_route(r):
    from native_g1_wrist_views import survey_candidates,select_route
    from native_g1_hybrid_contact import current_view_scene
    g=r.g
    proposal=next(survey_candidates(r.model,g['wrist_config']))
    return select_route(r.model,r.solver,current_view_scene(g),r.base,
        np.asarray(r.robot.get_joint_positions())[r.arm],[proposal],
        g['wrist_config'],g['wrist_X'])


def initialize(r,ops):
    from .probing_v2 import result_packet
    g=r.g;route,audit=initial_route(r)
    if route is not None:
        r.move_path(route['q_path'][1:],'PREGRASP')
        r.hold(g['wrist_config']['settle_s']);g['wrist_routes'].append(route)
        source='original first survey route'
    else:
        r.hold(g['wrist_config']['settle_s'])
        if (r.supervisor.latched is not None or r.grasp_reference is not None
                or max(r.filtered())>r.policy.data['force']['unloaded_pad_n']
                or not r.supervisor.force.settled()):
            raise RuntimeError('INITIAL_IMAGE_REQUIRES_ORIGINAL_VERIFIED_UNLOADED_BOUNDARY')
        source='measured unloaded wrist; first survey route unavailable; Agent must inspect'
    first=ops.observe()
    atomic_json(r.output/'INITIAL_OBSERVATION_READY.json',result_packet(first))
    atomic_json(r.output/'INITIAL_OBSERVATION_INFRASTRUCTURE.json',dict(
        source=source,original_route_audit=audit,measured_q=np.asarray(r.robot.get_joint_positions()).tolist(),
        measured_object_coordinate=r.actual(),fallback_used=route is None,
        object_or_robot_teleportation=False,grasp_or_view_target_supplied=False,
        physical_attempt_started=False,frozen_motion_and_perception_owners=True))


def serve_registered(r):
    from .probing_v2 import serve
    from .probing_v7 import OperationsV7
    from .prismatic_preload_v71 import PreloadOperations
    from .segments import validate_action
    from .articulation_v7 import Articulation
    kind=r.job['articulation_v7']['joint_type']
    factory=PreloadOperations if kind=='prismatic' else OperationsV7
    def initialized(runtime):
        ops=factory(runtime)
        try:initialize(runtime,ops)
        except BaseException:
            ops.close_workers();raise
        return ops
    def check(action):
        validate_action(action)
        Articulation(kind,(1,0,0),(0,0,0),tuple(r.job['articulation_v7']['limits_native'])).validate_target(action)
        if not action['role'].startswith(('FIT','VALIDATION')):raise ValueError('Predeclare FIT/VALIDATION')
    return serve(r,operations_factory=initialized,initialize=False,action_validator=check)
