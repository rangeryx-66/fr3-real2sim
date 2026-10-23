"""Real frozen prismatic owner's declaration guards, without an Isaac actor.

Requires the documented simulation Python dependencies. The synthetic ticket
contains no object geometry. Stop at NO_CURRENT_AUTOMATIC_PLAN after verifying
the unchanged owner's declaration/preload guards; no contact is executed.
"""
import hashlib
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace


def verify():
    from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75 as Owner
    from interaction_runtime.articulation_v7 import Articulation
    from .action_declaration import ticket, redeclare
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder);view=p/'view';view.mkdir()
        for name in ('rgb.png','depth_m.npy','camera.json'):(view/name).write_bytes(name.encode())
        plan=p/'visual_plan.json';plan.write_text(json.dumps(dict(path=[dict(state=0.),dict(state=.2)])))
        image=p/'AGENT_GRASP_PLAN_INSPECTION.png';image.write_bytes(b'synthetic contract fixture')
        sha=hashlib.sha256(image.read_bytes()).hexdigest()
        (p/'V8_PLAN_INSPECTION_PROVENANCE.json').write_text(json.dumps(dict(sha256=sha,
            plan_sha256=hashlib.sha256(plan.read_bytes()).hexdigest())))
        actions=[dict(role='VALIDATION_NEW',target_q=.15,coordinate_unit='m',profile='constant',hold=True)]
        ops=object.__new__(Owner)
        ops.r=SimpleNamespace(output=p,base_generation=0,closure_policy=dict(preload_n=1.),
            job=dict(articulation_v7=dict(joint_type='prismatic')))
        ops.g=dict(agent_observations=[dict(folder=str(view),accepted_for_geometry=True)],
            v75_declared_chain=['original'],v75_planned_preload=1.)
        ops.acquisitions=0;ops.current_plan_ready=True
        ops.articulation=lambda:Articulation('prismatic',(0,1,0),(0,0,0),(0,.3))
        ops.validate_execution=lambda:dict(valid=True)  # software fixture, not native route evidence
        ops._v81_plan_ticket=ticket(ops)
        command=dict(segments=actions,inspected_image_sha256=sha,native_g1_preload_request_n=1.)
        before=Owner.attempt(ops,command)
        assert before['reason']=='DECLARE_AND_REVALIDATE_CHANGED_CHAIN_BEFORE_ACQUISITION'
        assert redeclare(ops,command)['declared']
        ops.current_plan_ready=False  # deliberately stop before physics/geometry imports
        after=Owner.attempt(ops,command)
        assert after['reason']=='NO_CURRENT_AUTOMATIC_PLAN'
        preload=Owner.attempt(ops,dict(command,native_g1_preload_request_n=1.5))
        assert preload['reason']=='RECHECK_CLOSING_CHAIN_AFTER_PRELOAD_CHANGE'
        unchanged=Owner.attempt(ops,dict(command,segments=['different']))
        assert unchanged['reason']=='DECLARE_AND_REVALIDATE_CHANGED_CHAIN_BEFORE_ACQUISITION'
        return dict(status='REAL_FROZEN_OWNER_HANDSHAKE_PASS',before=before,after=after,
                    unmatched_preload_rejected=True,undeclared_chain_rejected=True,
                    physics_actor_launched=False,native_route_verified=False,
                    original_owner_source_unchanged=True)


if __name__=='__main__':print(json.dumps(verify(),indent=2))
