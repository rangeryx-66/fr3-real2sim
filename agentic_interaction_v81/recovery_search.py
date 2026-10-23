"""Agent-requested second batch of existing observed prismatic families.

Keep the frozen first batch, geometry, scores and native execution predicates.
A rejected bounded shortlist is not an exhaustive search of its input pool.
This seam is inactive for revolute planning and never supplies a saved pose.
"""
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
from unittest.mock import patch


def family(g):
    return tuple(float(g[k]) for k in ('section','insertion_m','roll','pitch'))


def batch(pool, limit, round_index, original):
    if type(round_index) is not int or round_index not in (0,1):
        raise ValueError('BOUNDED_RECOVERY_SEARCH_ROUND_REQUIRED')
    first=original(pool,limit)
    if round_index==0:return first
    used={family(g) for g in first}
    return original([g for g in pool if family(g) not in used],limit)


def observation_identity(ops):
    from interaction_runtime.recovery_candidates_v7 import observation_key
    return dict(observations=observation_key(ops.g['agent_observations']),
        selected_masks=[hashlib.sha256((Path(v['folder'])/'handle_mask.npy').read_bytes()).hexdigest()
            for v in ops.g['agent_observations'] if v['accepted_for_geometry']],
        station_generation=ops.r.base_generation,
        base=[float(x) for x in ops.r.base],
        acquisitions=ops.acquisitions)


def check(ops,command):
    index=command.get('recovery_search_round',0)
    if type(index) is not int or index not in (0,1):
        raise ValueError('BOUNDED_RECOVERY_SEARCH_ROUND_REQUIRED')
    if index==0:return 0
    if ops.r.job['articulation_v7']['joint_type']!='prismatic':
        raise ValueError('RECOVERY_SEARCH_EXTENSION_IS_PRISMATIC_ONLY')
    if (ops.g['recovery_state']!='UNLOADED_OBSERVATION'
            or ops.r.supervisor.latched is not None):
        raise ValueError('VERIFIED_UNLOADED_UNLATCHED_BOUNDARY_REQUIRED')
    from .boundary_operations import wrist_evidence
    if not command.get('reason') or not wrist_evidence(ops,command):
        raise ValueError('FRESH_INSPECTED_WRIST_RECOVERY_REASON_REQUIRED')
    prior=getattr(ops,'_v81_failed_search',None)
    decline=command.get('decline_current_plan')
    if decline is not None:
        if (not isinstance(decline,dict) or set(decline)!={'image_sha256','reason'}
                or not isinstance(decline['reason'],str) or not decline['reason'].strip()
                or not ops.current_plan_ready or not getattr(ops,'_v81_plan_ticket',None)):
            raise ValueError('CURRENT_INSPECTED_PLAN_REJECTION_REQUIRED')
        from .action_declaration import ticket
        output=ops.r.output;provenance=output/'V8_PLAN_INSPECTION_PROVENANCE.json'
        image=output/'AGENT_GRASP_PLAN_INSPECTION.png'
        if not provenance.exists() or not image.exists() or ticket(ops)!=ops._v81_plan_ticket:
            raise ValueError('STALE_DECLINED_PLAN')
        record=json.loads(provenance.read_text())
        if (record.get('plan_sha256')!=ops._v81_plan_ticket['plan_sha256']
                or record.get('sha256')!=decline['image_sha256']
                or record.get('sha256')!=hashlib.sha256(image.read_bytes()).hexdigest()):
            raise ValueError('STALE_DECLINED_PLAN_IMAGE')
        if getattr(ops,'_v81_current_search_round',0)!=0:
            raise ValueError('BOUNDED_RECOVERY_SEARCH_ROUND_REQUIRED')
        return 1
    if (not prior or prior['round']!=0
            or prior['reason']!='BOUNDED_VISUAL_APPROACH_IK_SEARCH_EXHAUSTED'
            or prior['identity']!=observation_identity(ops)):
        raise ValueError('SAME_CURRENT_OBSERVATIONS_FIRST_BATCH_FAILURE_REQUIRED')
    return 1


@contextmanager
def scope(ops,index):
    identity=observation_identity(ops)  # reject stale/missing evidence before expensive search
    import native_g1_robust_contact_ranking as frozen
    original=frozen.diverse_shortlist
    audit=[]
    def shortlist(pool,limit=18):
        selected=batch(pool,limit,index,original)
        audit.append(dict(input_families=len(pool),limit=limit,round=index,
            selected=[list(family(g)) for g in selected],
            same_poses_and_scores=True,native_checks_still_required=True))
        return selected
    from contextlib import nullcontext
    context=patch.object(frozen,'diverse_shortlist',shortlist) if index==1 else nullcontext()
    with context:
        try:yield
        finally:
            path=ops.r.output/'RECOVERY_SEARCH_V81.json'
            path.write_text(json.dumps(dict(identity=identity,batches=audit,
                search_is_not_physical_recovery=True),indent=2))


@contextmanager
def skip_dominated_checks():
    """Skip only utility values that cannot replace an already valid winner.

    The frozen planner computes this exact utility AFTER fine arc/approach IK.
    Its own winner comparison is strict `key > best[0]`. Never prune before a
    complete native-feasible candidate exists, or based on a coarse score.
    """
    from interaction_runtime.prismatic_candidate_v75 import PrismaticCandidateCheckV75 as Owner
    original=Owner.__call__
    def checked(self,candidate):
        key=candidate['shared_selection_utility']
        best=getattr(self,'_v81_verified_utility',None)
        if best is not None and key<=best:
            return dict(valid=False,reason='DOMINATED_BY_ALREADY_NATIVE_VALIDATED_UTILITY',
                pruned_not_collision_failure=True,shared_selection_utility=key,
                verified_winner_utility=best,native_checks_not_claimed=True)
        result=original(self,candidate)
        if result['valid']:self._v81_verified_utility=key
        return result
    with patch.object(Owner,'__call__',checked):yield


def complete(ops,index,result):
    ops._v81_current_search_round=index
    reason=result.get('reason')
    code='BOUNDED_VISUAL_APPROACH_IK_SEARCH_EXHAUSTED'
    if reason==repr(RuntimeError(code)):reason=code
    ops._v81_failed_search=(dict(round=index,identity=observation_identity(ops),
        reason=reason,original_reason=result.get('reason')) if not result.get('plan') else None)
    result['recovery_search_round']=index
    result['bounded_shortlist_exhaustion_is_not_global_infeasibility']=not result.get('plan',False)


def retire_plan(ops):
    """Move stale derived plans, never copy raw RGB-D or destroy prior evidence."""
    ops.current_plan_ready=False;ops._v81_plan_ticket=None
    output=ops.r.output
    names=['visual_plan.json','AGENT_GRASP_PLAN_INSPECTION.png','V8_PLAN_INSPECTION_PROVENANCE.json',
        'visual_planning/candidate.json']
    files=[output/n for n in names if (output/n).is_file()]
    if not files:return
    parent=output/'plan_history';parent.mkdir(exist_ok=True)
    generation=max([int(p.name) for p in parent.iterdir() if p.is_dir() and p.name.isdigit()],default=-1)+1
    destination=parent/('%03d'%generation);destination.mkdir()
    for p in files:p.rename(destination/str(p.relative_to(output)).replace('/','__'))


def current_packet(ops,packet):
    """An old plan picture must not appear to be the latest recovery result."""
    ready=bool(ops.current_plan_ready)
    packet['current_plan_ready']=ready
    if not ready:packet['plan_board']=None
    prior=getattr(ops,'_v81_failed_search',None)
    next_round=(1 if prior and prior['round']==0
        and prior['reason']=='BOUNDED_VISUAL_APPROACH_IK_SEARCH_EXHAUSTED'
        and prior['identity']==observation_identity(ops) else None)
    packet['recovery_search']=dict(next_round=next_round,
        maximum_rounds=2,explicit_Agent_request_required=True,
        inspected_plan_rejection_available=bool(ready and getattr(ops,'_v81_current_search_round',0)==0))
    return packet
