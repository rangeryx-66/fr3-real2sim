"""One preregistered matched engineering condition; never an Agent outcome.

Reuse the existing controlled-preload fixture's actual acquisition, checkpoint,
frozen closure, measured-state controller and complete guarded exit. Cached
observed FAILED-attempt geometry is explicit; no successful-pose fallback.
The full Agent entry never invokes this without an engineering-only manifest.
"""
import hashlib
import json
from pathlib import Path


def validate(config):
    if set(config)!={'kind','fixture','fixture_sha256','requested_preload_n',
                     'following','geometry','segments'}:
        raise ValueError('STRICT_CONTACT_COMPARISON_CONFIGURATION_REQUIRED')
    if config['kind']!='prismatic_short_matched_v81':raise ValueError('ENGINEERING_COMPARISON_ONLY')
    if config['requested_preload_n'] not in (.5,.75,1.,1.5):raise ValueError('LEGAL_PRELOAD_MENU_ONLY')
    if config['following'] not in ('legacy_window','rate_consistent'):raise ValueError('KNOWN_FOLLOWING_REQUIRED')
    if config['geometry'] not in ('observed_selected','same_section_zero_insertion'):
        raise ValueError('PREDECLARED_OBSERVED_FAMILY_ONLY')
    path=Path(config['fixture'])
    if hashlib.sha256(path.read_bytes()).hexdigest()!=config['fixture_sha256']:
        raise ValueError('MATCHED_OBSERVED_FIXTURE_COMMITMENT_MISMATCH')
    data=json.loads(path.read_text())
    if data.get('GT_grasp_used') is not False or data.get('failed_whole_attempt_source') is not True:
        raise ValueError('OBSERVED_FAILED_ATTEMPT_FIXTURE_REQUIRED')
    return data


def serve(r):
    import copy
    import numpy as np
    from agent_command_protocol_v12 import atomic_json
    from interaction_runtime.prismatic_preload_v71 import PreloadFixtureOperations
    from interaction_runtime.prismatic_radial_session_v751 import PrismaticRadialOperationsV751
    from .operations import operations_class
    from .declaration_preflight import check
    config=r.job['v81_contact_comparison'];data=copy.deepcopy(validate(config))
    if r.job['v81_phase']!='engineering' or r.job['articulation_v7']['joint_type']!='prismatic':
        raise ValueError('MATCHED_FIXTURE_IS_NOT_A_REPETITION_OR_REVOLUTE_EPISODE')
    class Matched(PreloadFixtureOperations,operations_class(PrismaticRadialOperationsV751)):
        pass
    ops=Matched(r);r.optional_recording_disabled=True
    try:
        provenance=r.output/'SOURCE_PROVENANCE.json'
        record=json.loads(provenance.read_text())
        record.update(fresh_observation=False,fresh_grasp_generation=False,
            saved_grasp_input=True,saved_successful_grasp_input=False,
            controlled_observed_failed_fixture=True,
            comparison_fixture_sha256=config['fixture_sha256'])
        atomic_json(provenance,record)
        actions=config['segments'];check(actions,ops.articulation())
        if config['geometry']=='same_section_zero_insertion':
            # Exact same-section/roll/pitch member of the FROZEN bar family:
            # T.pos = anchor + observed_normal * insertion - R * pad_offset.
            # No optimized offset, GT surface or asset-specific parameter.
            candidate=data['candidate'];T=np.array(data['T']);normal=np.asarray(data['normal_world'])
            T[:3,3]-=normal*float(candidate['insertion_m'])
            candidate.update(T=T.tolist(),T_grasp=T.tolist(),insertion_m=0.,
                diagnostic_family_regeneration=True,diagnostic_contact_scores_not_rescored=True)
            data['T']=T.tolist()
        fixture=r.output/'MATCHED_OBSERVED_INPUT.json';atomic_json(fixture,data)
        r.job['preload_fixture']=str(fixture)
        r.v81_contact_following_mode=config['following']
        r.transverse_unloading_enabled=True
        r.g.update(v75_declared_chain=actions,v75_planned_preload=float(config['requested_preload_n']))
        ops._v81_declared_chain=actions
        r.g['phase']='SETTLE';r.hold(.5)
        geometry=r.output/'agent_geometry';geometry.mkdir(exist_ok=True)
        atomic_json(geometry/'observed_geometry.json',dict(normal_world=data['normal_world'],
            source='committed archived observed comparison fixture; not new perception'))
        ops.prepare_fixture()
        atomic_json(r.output/'MATCHED_CONDITION.json',dict(config=config,
            source='archived observed FAILED whole attempt; matched diagnostic only',
            full_original_approach_closing_chain_and_exit_checks=True,
            fresh_perception_or_Agent_generalization_claim=False,
            object_state_reset_within_scene=False,physical_parameters_changed=False))
        result=ops.attempt(dict(segments=actions,contact_following=config['following'],
            native_g1_preload_request_n=float(config['requested_preload_n']),
            reason='Explicit preregistered short matched contact engineering comparison'))
        atomic_json(r.output/'MATCHED_CONTACT_RESULT.json',result)
        return dict(status='MATCHED_CONTACT_COMPARISON_ENDED',result=result,
                    benchmark=False,fresh_Agent_outcome=False)
    finally:ops.close_workers()
