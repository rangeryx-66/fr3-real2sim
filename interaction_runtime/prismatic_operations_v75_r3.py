"""V7.5 R3 explicit prismatic operations; R2/revolute files stay immutable."""
import json
from unittest.mock import patch
from .prismatic_feedback_v75 import PrismaticEvidenceOperationsV75
from .prismatic_memory_v75_r3 import recovery_memory,stamp_catalog,resolve_candidate,LEGAL_REQUESTS


def build_conditioned_geometry(g):
    import native_g1_contact_recovery as recovery
    import native_g1_multiview_input as multi
    import native_g1_agent_incremental_activation as incremental
    from .recovery_candidates_v7 import catalog,order
    blocked,memory=recovery_memory(g['contact_attempts'],g['a'].output,g['policy']['preload_n'])
    original=multi.rescore
    def alternatives(*args,**kwargs):
        scored=original(*args,**kwargs)
        remaining=order(scored,blocked,g.get('preferred_existing_candidate_family'))
        g['recovery_candidate_catalog']=stamp_catalog(catalog(scored,blocked,g['agent_observations']),memory)
        recovery.write(g,'AGENT_CANDIDATE_CATALOG.json',g['recovery_candidate_catalog'])
        recovery.write(g,'EPISODE_CANDIDATE_MEMORY.json',dict(memory,before=len(scored),after=len(remaining)))
        return remaining
    with patch.object(multi,'rescore',alternatives):result=incremental.build_geometry(g)
    if result.get('plan'):
        candidate=json.loads((g['a'].output/'visual_plan.json').read_text())
        result.update(candidate_family=list(recovery.family(candidate)),failed_episode_families=[list(f)for f in blocked])
    return result


class PrismaticOperationsV75R3(PrismaticEvidenceOperationsV75):
    def build_geometry(self):
        import native_g1_contact_recovery as recovery
        with patch.object(recovery,'build_geometry',build_conditioned_geometry):
            return super().build_geometry()

    def request_plan(self,command):
        request=command.get('candidate_request')
        if request is None:return super().request_plan(command)
        self.current_plan_ready=False
        requested=command.get('native_g1_preload_request_n',self.r.closure_policy['preload_n'])
        if isinstance(requested,bool)or requested not in LEGAL_REQUESTS:
            return dict(plan=False,reason='TESTED_PRELOAD_MENU_ONLY',recoverable=True)
        if self.g['recovery_state']!='UNLOADED_OBSERVATION'or self.r.supervisor.latched is not None:
            return dict(plan=False,reason='VERIFIED_UNLOADED_BOUNDARY_REQUIRED',recoverable=False)
        try:
            selected=resolve_candidate(request,self.g.get('recovery_candidate_catalog'),
                self.g['agent_observations'],self.g['contact_attempts'],self.r.output,requested)
        except ValueError as error:return dict(plan=False,reason=str(error),recoverable=True,robot_action_executed=False)
        forwarded={k:v for k,v in command.items()if k!='candidate_request'}
        self.g['preferred_existing_candidate_family']=selected
        try:
            result=super().request_plan(forwarded)
            self.g.pop('preferred_existing_candidate_family',None)
            if result.get('plan'):result['Agent_selected_existing_family']=list(selected)
            else:
                rejected=result;result=super().request_plan(forwarded)
                result['Agent_requested_family_infeasible']=dict(family=list(selected),
                    reason=rejected.get('reason'),automatic_eligible_fallback=True)
            return result
        finally:self.g.pop('preferred_existing_candidate_family',None)

    def release_park(self):
        if self.r.supervisor.latched is not None:
            return dict(safe=False,reason='HARD_STOP_NO_AUTOMATIC_CONTINUATION',stop=self.r.supervisor.latched)
        import native_g1_safe_park as park
        from .prismatic_unloaded_park_v75_r3 import plan_unloaded_exit
        original=park.plan_exit
        with patch.object(park,'plan_exit',lambda g:plan_unloaded_exit(g,original)):
            return super().release_park()
