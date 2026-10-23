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


def check_declaration(config,job):
    from interaction_runtime.articulation_v7 import Articulation
    from .declaration_preflight import check
    # Target-unit/range validation needs no live object transform or first
    # sensor row. This dummy frame is NEVER supplied to motion or estimation.
    prior=Articulation(job['articulation_v7']['joint_type'],(1,0,0),(0,0,0),
                       tuple(job['articulation_v7']['limits_native']))
    return check(config['segments'],prior)


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
        actions=config['segments'];check_declaration(config,r.job)
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
        # This is NOT an Agent acquisition/recovery decision and does not run
        # SAM. Do not fabricate a measured SAM cost to pass the full Agent's
        # perception-cost admission. Check its EXACT native whole chain here,
        # then use the original finite operation owner. Per-step resource,
        # force, collision, joint and actuator guards remain authoritative.
        from interaction_runtime.prismatic_candidate_v75 import PrismaticCandidateCheckV75
        candidate=json.loads((r.output/'visual_plan.json').read_text())
        native=PrismaticCandidateCheckV75(r.g,r.policy,r.c,r.solver)(candidate)
        atomic_json(r.output/'MATCHED_NATIVE_CHAIN_CHECK.json',native)
        if not native['valid']:
            return dict(status='MATCHED_NATIVE_PREFLIGHT_REJECTED',physical_acquisitions=0,
                        check=native,safe_exit=ops.release_park())
        ops.actions=list(actions)
        from interaction_runtime.operations import Operations
        result=Operations.attempt(ops,dict(segments=actions,
            reason='Preregistered one-acquisition engineering fixture, no SAM-cost claim'))
        result.update(contact_following_v81=config['following'],
            resource_admission='registered finite low-level fixture; full Agent perception-cost admission not invoked',
            original_native_whole_chain_check=native['valid'])
        atomic_json(r.output/'MATCHED_CONTACT_RESULT.json',result)
        return dict(status='MATCHED_CONTACT_COMPARISON_ENDED',result=result,
                    benchmark=False,fresh_Agent_outcome=False)
    finally:ops.close_workers()
