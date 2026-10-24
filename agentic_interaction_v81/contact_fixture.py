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
    if config['kind'] not in ('prismatic_short_matched_v81','prismatic_preload_extension_v81'):raise ValueError('ENGINEERING_COMPARISON_ONLY')
    from .preload_extension import request
    request(config['requested_preload_n'],study=config['kind']=='prismatic_preload_extension_v81')
    if config['following'] not in ('legacy_window','rate_consistent'):raise ValueError('KNOWN_FOLLOWING_REQUIRED')
    if config['geometry'] not in ('observed_selected','same_section_zero_insertion'):
        raise ValueError('PREDECLARED_OBSERVED_FAMILY_ONLY')
    path=Path(config['fixture'])
    if hashlib.sha256(path.read_bytes()).hexdigest()!=config['fixture_sha256']:
        raise ValueError('MATCHED_OBSERVED_FIXTURE_COMMITMENT_MISMATCH')
    data=json.loads(path.read_text())
    fresh=(config['kind']=='prismatic_preload_extension_v81'
        and data.get('kind')=='fresh_matched_observation_reference_v81'
        and data.get('physical_acquisitions_at_export')==0
        and data.get('saved_successful_grasp') is False)
    if data.get('GT_grasp_used') is not False or not (data.get('failed_whole_attempt_source') is True or fresh):
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


def export_reference(ops,command):
    """One fresh observed pose for a declared matched preload comparison only."""
    from .action_declaration import ticket
    from agent_command_protocol_v12 import atomic_json
    from agentic_interaction_v8.resources import digest
    r=ops.r;root=r.output
    if (not r.job.get('v81_preload_study') or r.job.get('v81_phase')!='engineering'
            or ops.acquisitions or not ops.current_plan_ready
            or ops._v81_plan_ticket!=ticket(ops)):
        return dict(exported=False,reason='CURRENT_FRESH_ZERO_ACQUISITION_ENGINEERING_PLAN_REQUIRED')
    record=json.loads((root/'V8_PLAN_INSPECTION_PROVENANCE.json').read_text())
    board=root/'AGENT_GRASP_PLAN_INSPECTION.png'
    if command.get('inspected_image_sha256')!=record['sha256'] or digest(board)!=record['sha256']:
        return dict(exported=False,reason='ACTUAL_PLAN_IMAGE_INSPECTION_REQUIRED')
    if r.supervisor.latched is not None or r.grasp_reference is not None:
        return dict(exported=False,reason='UNLOADED_UNLATCHED_STATE_REQUIRED')
    candidate=json.loads((root/'visual_plan.json').read_text())
    from native_g1_visual_input import observed_geometry
    geometry=observed_geometry(root/'agent_geometry')
    section=next(s for s in candidate['grasp_geometry_selection']['sections']
        if s['fraction']==candidate['section_fraction'])
    data=dict(kind='fresh_matched_observation_reference_v81',T=candidate['T'],
        seed_q=candidate['path'][0]['q'],candidate=candidate,
        normal_world=geometry['normal_world'],bar_axis_world=geometry['axis_world'],
        anchor_world=section['anchor_world_m'],GT_grasp_used=False,saved_successful_grasp=False,
        physical_acquisitions_at_export=0,failed_whole_attempt_source=False,
        observation_source=dict(episode=r.job['episode_id'],plan_sha256=digest(root/'visual_plan.json'),
            observation_ticket=ticket(ops),inspected_image_sha256=record['sha256'],
            reason=command['reason'],pose='fresh automatic observed candidate; fixed for controlled preload comparison only'))
    data['fixture_content_sha256']=hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()
    path=root/'FRESH_MATCHED_REFERENCE.json';atomic_json(path,data)
    return dict(exported=True,fixture=str(path),sha256=digest(path),
        physical_contact_executed=False,benchmark_or_full_Agent_pose_fallback=False)


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
            controlled_observed_failed_fixture=bool(data.get('failed_whole_attempt_source')),
            controlled_fresh_observed_reference=data.get('kind')=='fresh_matched_observation_reference_v81',
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
            source='one fixed observed reference; matched diagnostic only, not a full Agent outcome',
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
        if r.job.get('v81_integrated_repair'):
            from .validation_queue import enqueue
            enqueue(ops,dict(segments=actions,reason='Registered matched preload condition'))
            ops._v81_action_index=0;ops._v81_block_first_record=r.record.count
            ops._v81_current_acquisition_verified=False
        from interaction_runtime.operations import Operations
        result=Operations.attempt(ops,dict(segments=actions,
            reason='Preregistered one-acquisition engineering fixture, no SAM-cost claim'))
        result.update(contact_following_v81=config['following'],
            resource_admission='registered finite low-level fixture; full Agent perception-cost admission not invoked',
            original_native_whole_chain_check=native['valid'])
        if r.job.get('v81_integrated_repair'):
            from .validation_queue import boundary
            result['pending_validation']=boundary(ops)
        atomic_json(r.output/'MATCHED_CONTACT_RESULT.json',result)
        return dict(status='MATCHED_CONTACT_COMPARISON_ENDED',result=result,
                    benchmark=False,fresh_Agent_outcome=False)
    finally:ops.close_workers()
