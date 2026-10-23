"""Future reach checks must not insert or alter physical probing actions."""
import copy,sys,unittest
from contextlib import nullcontext
from types import ModuleType,SimpleNamespace
from unittest.mock import patch
from agentic_interaction_v81.operations import operations_class
from interaction_runtime.articulation_v7 import Articulation


class HorizonTests(unittest.TestCase):
    def run_request(self,horizon):
        class Owner:
            def request_plan(self,command):
                return dict(plan=True,controller_actions=copy.deepcopy(command['planned_segments']))
        ops=operations_class(Owner)();ops.r=SimpleNamespace(job={'articulation_v7':{'joint_type':'revolute'}})
        ops.articulation=lambda:Articulation('revolute',(0,0,1),(0,0,0),(0,1.75))
        action=dict(role='FIT_SHORT',target_q=.2,coordinate_unit='rad',profile='constant',hold=True)
        validation=dict(action,role='VALIDATION_REVERSE',target_q=.1)
        actions=[action,validation]
        command=dict(planned_segments=actions,planning_target_q=horizon)
        native=ModuleType('native_g1_execution_candidates');native.current_export=lambda *a:a[0]
        mobile=ModuleType('interactive_twin_recovery.mobile');mobile.at_base=lambda *a:a[0]
        with patch.dict(sys.modules,{'native_g1_execution_candidates':native,'interactive_twin_recovery.mobile':mobile}), \
             patch('agentic_interaction_v81.performance.cached_planning_fk',return_value=nullcontext()), \
             patch('agentic_interaction_v81.recovery_memory.task_memory') as prismatic:
            result=ops.request_plan(command)
            prismatic.assert_not_called()
        return result,ops,actions

    def test_long_horizon_preserves_the_short_physical_action(self):
        result,ops,actions=self.run_request(1.5708)
        self.assertEqual(ops.r.job['benchmark_goal_rad'],1.5708)
        self.assertEqual(result['controller_actions'],actions)
        self.assertEqual(ops._v81_declared_chain,actions)
        self.assertEqual(result['executable_first_target_native'],.2)
        self.assertTrue(result['planning_horizon_is_not_motion'])

    def test_horizon_cannot_exceed_native_limits(self):
        with self.assertRaises(ValueError):self.run_request(2.)


if __name__=='__main__':unittest.main()
