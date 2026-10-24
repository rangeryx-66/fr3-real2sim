"""Exercise the actual frozen V75 menu seam, without an Isaac actor."""
from types import SimpleNamespace
from unittest.mock import patch
from .preload_extension import menu_scope


def verify():
    from interaction_runtime.prismatic_feedback_v75 import PrismaticEvidenceOperationsV75 as Owner
    from interaction_runtime.articulation_v7 import Articulation
    parent=Owner.__mro__[1];ops=object.__new__(Owner)
    ops.r=SimpleNamespace(job={'v81_integrated_repair':True,'v81_preload_study':True},
        closure_policy={'preload_n':1.,'finger_effort_limit_n':10.},
        grasp_reference=None,supervisor=SimpleNamespace(latched=None,force=SimpleNamespace(settled=lambda:True)),
        filtered=lambda:[0.,0.],policy=SimpleNamespace(data={'force':{'unloaded_pad_n':.05}}))
    ops.g={'recovery_state':'UNLOADED_OBSERVATION'}
    ops.articulation=lambda:Articulation('prismatic',(0,1,0),(0,0,0),(0,.3))
    actions=[dict(role=role,target_q=q,coordinate_unit='m',profile='constant',hold=True)
        for role,q in [('FIT_SHORT',.018),('VALIDATION_SHORT',.006)]]
    command=dict(planned_segments=actions,native_g1_preload_request_n=3.)
    original=Owner.request_plan(ops,command)
    assert original['reason']=='TESTED_PRELOAD_MENU_ONLY'
    recorded=[]
    def no_geometry(self,c):
        recorded.append(self.r.closure_policy['preload_n'])
        return {'plan':False,'reason':'DIAGNOSTIC_STOP_BEFORE_GEOMETRY_OR_PHYSICS'}
    with patch.object(parent,'request_plan',no_geometry),menu_scope(ops):
        for value in (1.5,2.,2.5,3.):
            result=Owner.request_plan(ops,dict(command,native_g1_preload_request_n=value))
            assert result['reason']=='DIAGNOSTIC_STOP_BEFORE_GEOMETRY_OR_PHYSICS'
            assert ops.g['v75_planned_preload']==value
    assert recorded==[1.5,2.,2.5,3.]
    assert ops.r.closure_policy['finger_effort_limit_n']==10.
    return dict(status='ACTUAL_FROZEN_OWNER_MENU_ADAPTER_PASS',requested_values_reached=recorded,
        original_native_request_authority_preserved=True,actor_launched=False)


if __name__=='__main__':
    import json
    print(json.dumps(verify(),indent=2))
