"""Agent-selected compact-cap grasp family on existing observed surfaces.

Unlike a bar, a compact cap has no stable privileged longitudinal axis. Pinch
around its observed centre with several in-plane jaw directions. Rear depth
remains the existing declared uncertainty, not a newly measured solid. No GT,
asset identity, saved pose, score weight, collider or actuator changes.
"""
import copy,json,inspect,uuid
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def radial_transforms(T,anchor,normal):
    n=np.asarray(normal,float);n/=np.linalg.norm(n);a=np.asarray(anchor,float)
    for angle in range(0,360,45):
        R=Rotation.from_rotvec(-n*np.deg2rad(angle)).as_matrix()
        D=np.eye(4);D[:3,:3]=R;D[:3,3]=a-R@a
        yield float(angle),D@np.asarray(T)


def generate(geometries,model,surfaces,base,axis,folder,output,nonce=None):
    import plan_shared_door_skill as shared
    import native_g1_multiview_input as multi
    import xarm7_native_planning as native
    data=json.loads((Path(folder)/'observed_geometry.json').read_text())
    section=min(data['sections'],key=lambda s:abs(s['fraction']))
    anchor=np.asarray(section['anchor_world_m']);normal=np.asarray(data['normal_world'])
    templates=[g for g in geometries if abs(g['section'])<1e-12 and abs(g['roll'])<1e-12 and abs(g['pitch'])<1e-12]
    if not templates:raise ValueError('NO_CENTRED_NATIVE_TEMPLATE_FOR_RADIAL_PINCH')
    # Exactly the official native aperture range used in the preserved adapter.
    source=inspect.getsource(shared.closing_quality).replace('np.linspace(.1,0.,41)','np.linspace(.084,.001641,41)')
    ns=dict(shared.__dict__);exec(compile(source,__file__+'::native_aperture_closure','exec'),ns)
    common_source=inspect.getsource(native.common_quality).replace(' from handle_contact_quality import ray_surface\n','')
    common_ns=dict(native.__dict__,ray_surface=multi.fast_ray)
    exec(compile(common_source,__file__+'::observed_surfaces_only','exec'),common_ns)
    scene=multi.closing_scene(model,folder);width=float(np.clip(data['dimensions_m'][1],.001641,.084))
    result=[]
    for template in templates:
        for angle,T in radial_transforms(template['T'],anchor,normal):
            quality=ns['closing_quality'](model,scene,T,base,width)
            quality['patch']=common_ns['common_quality'](model,surfaces,T,base,axis)
            quality['physical_aperture_m']=float(np.clip(quality['patch']['native']['best_common_pose_aperture']['common_aperture_m'],.001641,.084))
            quality['support_score']=quality['patch']['score']/max(1.,1+quality['root_contact_count'])if quality['non_handle_closing_collision']is None else 0.
            quality['rank']+=quality['support_score']
            quality['radial_cap_generation_nonce']=nonce
            result.append(dict(T=T,section=0.,insertion_m=template['insertion_m'],roll=angle,pitch=0.,contact=quality))
    audit=dict(family='compact_radial_cap',plan_nonce=nonce,Agent_selection_required=True,generated=len(result),
               in_plane_angles_deg=list(range(0,360,45)),section='observed central cap only, no bar-end sections',
               anchor_world_m=anchor.tolist(),anchor_depth='same declared rear uncertainty as input; not measured backside',
               score_weights_changed=False,scoring_surfaces='open observed patches + separately declared existing rear hypotheses',
               candidate_shortlist_budget_changed=False,GT_used=False)
    (Path(output)/'RADIAL_CAP_FAMILY.json').write_text(json.dumps(audit,indent=2)+'\n')
    return result


def radial_operations_class():
    from interaction_runtime.prismatic_v751 import PrismaticOperationsV751
    class RadialOperations(PrismaticOperationsV751):
        def request_plan(self,command):
            family=command.get('grasp_family','compact_radial_cap'if getattr(self,'radial_selected',False)else'existing_bar')
            if family not in ('existing_bar','compact_radial_cap'):return dict(plan=False,reason='UNKNOWN_OBSERVED_GRASP_FAMILY')
            self.radial_selected=family=='compact_radial_cap'
            return super().request_plan(command)
        def build_geometry(self):
            if not getattr(self,'radial_selected',False):return super().build_geometry()
            from unittest.mock import patch
            import native_g1_contact_recovery as recovery
            from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75
            # Install at the recovery owner, AFTER the inherited planner has
            # selected its scoring implementation. An outer multi.rescore
            # patch is overwritten by the legacy bidirectional-rank selector.
            nonce=uuid.uuid4().hex;self.g['v751_radial_plan_nonce']=nonce
            try:
                with patch.object(recovery,'build_geometry',build_radial_conditioned_geometry):
                    result=PrismaticEvidenceOperationsV75.build_geometry(self)
            finally:self.g.pop('v751_radial_plan_nonce',None)
            if result.get('plan'):
                audit=self.r.output/'visual_planning/RADIAL_CAP_FAMILY.json'
                if not audit.exists():
                    self.current_plan_ready=False
                    return dict(plan=False,reason='RADIAL_FAMILY_PROVENANCE_MISSING_NO_BAR_FALLBACK',recoverable=True)
                chosen=json.loads((self.r.output/'visual_plan.json').read_text())
                marker=json.loads(audit.read_text())
                if (marker.get('family')!='compact_radial_cap'or marker.get('plan_nonce')!=nonce
                        or chosen['contact_geometry_audit'].get('radial_cap_generation_nonce')!=nonce
                        or chosen['section_fraction']!=0. or chosen['pitch_deg']!=0.
                        or chosen['roll_deg']%45!=0.):
                    self.current_plan_ready=False
                    return dict(plan=False,reason='RADIAL_FAMILY_PROVENANCE_MISSING_NO_BAR_FALLBACK',recoverable=True)
            return result
    return RadialOperations


def build_radial_conditioned_geometry(g):
    """Same R3 memory/order/check owners, with one explicit generator seam."""
    from unittest.mock import patch
    import native_g1_contact_recovery as recovery
    import native_g1_multiview_input as multi
    import native_g1_agent_incremental_activation as incremental
    from interaction_runtime.prismatic_memory_v75_r3 import recovery_memory,stamp_catalog
    from interaction_runtime.recovery_candidates_v7 import catalog,order
    blocked,memory=recovery_memory(g['contact_attempts'],g['a'].output,g['policy']['preload_n'])
    actual_scorer=multi.rescore
    def alternatives(geometries,model,surfaces,base,origin,axis,config,output):
        candidates=generate(geometries,model,surfaces,base,axis,g['a'].output/'agent_geometry',output,g.get('v751_radial_plan_nonce'))
        scored=actual_scorer(candidates,model,surfaces,base,origin,axis,config,output)
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
