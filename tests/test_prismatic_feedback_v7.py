import hashlib,json,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

class PrismaticIsolationTests(unittest.TestCase):
    def test_frozen_revolute_inputs_are_byte_identical(self):
        m=json.loads((ROOT/'docs/experiments/prismatic_transmission_v7_20261020/REVOLUTE_FREEZE.json').read_text())
        for name,expected in m['source_sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),expected,name)

    def test_prismatic_feedback_has_no_revolute_or_benchmark_admission(self):
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackRuntime
        for j in [dict(articulation_v7=dict(joint_type='revolute')),dict(articulation_v7=dict(joint_type='prismatic'),experiment='V7_BENCHMARK')]:
            with self.assertRaises(ValueError):PrismaticFeedbackRuntime(j)

    def test_stationary_reference_still_observes_real_state_with_continuous_limits(self):
        from contextlib import nullcontext
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackRuntime
        from interaction_runtime.policy import Policy
        r=PrismaticFeedbackRuntime.__new__(PrismaticFeedbackRuntime)
        measured=np.ones(7)*.01;r.robot=SimpleNamespace(get_joint_positions=lambda:measured)
        r.arm=np.arange(7);r.measured_rail_feedback=True;r.reference_bias=np.ones(7)*.001
        r.reference_drive=SimpleNamespace(advance=lambda *a:np.eye(4))
        r.qtarget=measured+r.reference_bias;r.g={'qvelocity':np.zeros(7)}
        r.policy=Policy.load(ROOT/'configs/interaction_runtime_v1.json');r.dt=1/240;r.tick=8
        r.motion_limits=lambda phase:r.policy.motion(phase);r.c={'arm_speed_limits':[3.]*7}
        seen=[];r.solver=SimpleNamespace(velocity_step=lambda target,base,q,dt:(seen.append(q.copy()) or q+.1))
        r.base=np.zeros(4);r.profile=SimpleNamespace(measure=lambda *a:nullcontext())
        r.ensure_execution_budget=lambda *a:None;r.check_q=lambda *a:(True,None)
        r.step=lambda:'original physics and safety step'
        previous=r.qtarget.copy()
        self.assertEqual(r.cartesian_step(np.eye(4),'CONTACT_PRELOAD'),'original physics and safety step')
        np.testing.assert_array_equal(seen[0],measured)
        self.assertLessEqual(np.max(abs(r.g['qvelocity'])),.8*r.dt+1e-12)
        self.assertLessEqual(np.max(abs(r.qtarget-previous)),.8*r.dt**2+1e-12)
        np.testing.assert_array_equal(r.reference_bias,np.ones(7)*.001)

    def test_unloaded_budget_exit_decelerates_before_release_phase(self):
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackOperations
        from interaction_runtime.probing_v7 import OperationsV7
        from interaction_runtime.policy import Policy
        ops=PrismaticFeedbackOperations.__new__(PrismaticFeedbackOperations)
        ops.g={'phase':'PREGRASP'};order=[]
        ops.r=SimpleNamespace(phase='PREGRASP',supervisor=SimpleNamespace(latched=None),grasp_reference=None,filtered=lambda:np.zeros(2),policy=Policy.load(ROOT/'configs/interaction_runtime_v1.json'),event=lambda *a,**k:None,stop_command=lambda:order.append(('stop',ops.g['phase'])),hold=lambda t:order.append(('hold',ops.g['phase'],t)))
        with patch.object(OperationsV7,'release_park',side_effect=lambda:order.append(('release',ops.g['phase'])) or {'safe':True}):
            self.assertTrue(ops.release_park()['safe'])
        self.assertEqual(order[0],('stop','PREGRASP'))
        self.assertEqual(order[1],('hold','SAFE_PARK_MOTION',.3))
        self.assertEqual(ops.r.policy.motion('RELEASE')['measured_speed_stop_m_s'],.03)

    def test_nonlatched_contact_stop_allows_only_checked_exit_not_pulling(self):
        from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackOperations
        from interaction_runtime.probing_v7 import OperationsV7
        for latch in [None,{'reason':'FORCE_EMERGENCY'}]:
            ops=PrismaticFeedbackOperations.__new__(PrismaticFeedbackOperations)
            ops.r=SimpleNamespace(supervisor=SimpleNamespace(latched=latch),event=lambda *a,**k:None,grasp_reference=object())
            ops.g={'reference':object()};calls=[]
            ops.release_park=lambda:(calls.append('guarded exit') or {'safe':True})
            result={'segments':[{'reason':'CONTACT_LOST_PAUSE_IN_SAME_SCENE'}],'operation_status':'SAFETY_STOP','safe_exit':{'safe':False}}
            with patch.object(OperationsV7,'attempt',return_value=result):
                r=ops.attempt({'segments':[]})
            self.assertEqual(len(calls),1 if latch is None else 0)
            self.assertIs(ops.r.supervisor.latched,latch)
            self.assertEqual(r['operation_status'],'SAFETY_STOP')

    def test_task_capacity_does_not_choose_best_pad_or_best_direction(self):
        from native_g1_prismatic_ranking import bidirectional_capacity
        def r(x):return dict(sides=[dict(opening_capacity_assumed_mu=v) for v in x])
        self.assertAlmostEqual(bidirectional_capacity(r([1.,0.]),r([.2,.2])),.2)
        self.assertGreater(bidirectional_capacity(r([.5,.5]),r([.5,.5])),bidirectional_capacity(r([1.,0.]),r([.2,.2])))

if __name__=='__main__':unittest.main()
