"""Direct return retains original collision/approach and fallback owners."""
import sys,unittest
from types import SimpleNamespace
from unittest.mock import patch,Mock
import numpy as np
from agentic_interaction_v81.execution_return import direct_return


class ReturnTests(unittest.TestCase):
    def test_checked_current_route_skips_only_obsolete_detour(self):
        owner=SimpleNamespace(validate_execution=Mock(return_value={'legacy':True}),
            check_path=Mock(return_value={'valid':True,'samples':25}),
            approach_from=Mock(return_value={'valid':True,'preplan':[]}),
            observation=SimpleNamespace(write=Mock()))
        g=dict(robot=SimpleNamespace(get_joint_positions=lambda:np.ones(7)*.01),
            arm=np.arange(7),qtarget=np.zeros(7),wrist_routes=[{'q_path':[[1]*7]}])
        candidate=dict(preplan=[[0]*7]);runtime=SimpleNamespace(grasp_reference=None,supervisor=SimpleNamespace(latched=None))
        with patch.dict(sys.modules,{'native_g1_agent_execution':owner}):
            original=owner.validate_execution
            with direct_return(runtime):
                result=owner.validate_execution(g,candidate,False)
            self.assertIs(owner.validate_execution,original)
        self.assertTrue(result['valid']);self.assertEqual(len(result['return_path']),3)
        self.assertEqual(result['return_path'][0],[.01]*7)
        self.assertEqual(result['skipped_redundant_view_routes'],1)
        owner.check_path.assert_called_once();owner.approach_from.assert_called_once()
        original.assert_not_called()

    def test_collision_approach_and_loaded_conditions_fall_back_without_override(self):
        for condition in ['collision','approach','loaded','latched']:
            owner=SimpleNamespace(validate_execution=Mock(return_value={'valid':False,'reason':'original'}),
                check_path=Mock(return_value={'valid':condition!='collision'}),
                approach_from=Mock(return_value={'valid':condition!='approach'}))
            g=dict(robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(7)),
                arm=np.arange(7),qtarget=np.zeros(7),wrist_routes=[])
            runtime=SimpleNamespace(grasp_reference='loaded'if condition=='loaded'else None,
                supervisor=SimpleNamespace(latched='hard'if condition=='latched'else None))
            with self.subTest(condition=condition),patch.dict(sys.modules,{'native_g1_agent_execution':owner}):
                original=owner.validate_execution
                with direct_return(runtime):result=owner.validate_execution(g,dict(preplan=[[0]*7]),False)
                self.assertEqual(result,{'valid':False,'reason':'original'})
                original.assert_called_once()


if __name__=='__main__':unittest.main()
