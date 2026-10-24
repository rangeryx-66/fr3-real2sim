"""Regression for the measured R02 cross-start reversal counterexample."""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

from agentic_interaction_v81.replanning import candidate_gate, coverage


class ReversalAdmissionTests(unittest.TestCase):
    def candidate(self):
        return dict(path=[dict(state=.789470374584198), dict(state=1.1)],
                    section_fraction=.3, insertion_m=0., roll_deg=0., pitch_deg=0.)

    def actions(self):
        return [dict(target_q=q, coordinate_unit='rad', role=role,
                     profile='constant', hold=True)
                for q, role in [(1.1, 'FIT_OPEN'), (.55, 'FIT_REVERSE'), (1., 'VALIDATION')]]

    def certificate(self):
        return dict(valid=True, endpoints=[dict(target_rad=a['target_q'], exit_valid=True)
                                          for a in self.actions()])

    def test_cross_start_needs_its_own_certificate(self):
        p=self.candidate()
        self.assertFalse(coverage(p, [.55])['full_requested_actions_feasible'])
        p['execution_feasibility']=dict(v81_revolute_chain=self.certificate())
        self.assertTrue(coverage(p, [1.1, .55, 1.])['full_requested_actions_feasible'])
        self.assertFalse(coverage(p, [1.5707963267948966])['full_requested_actions_feasible'])
        p['execution_feasibility']['v81_revolute_chain']['valid']=False
        self.assertFalse(coverage(p, [.55])['full_requested_actions_feasible'])

    def test_native_checker_runs_before_rejecting_later_reverse(self):
        calls=[]
        with tempfile.TemporaryDirectory() as folder:
            class Native:
                def __init__(self):
                    self.audit=[]; self.policy=self.config=self.solver=None
                    self.g=dict(job=dict(v81_integrated_repair=True,
                        articulation_v7=dict(joint_type='revolute')),
                        a=SimpleNamespace(output=Path(folder)))
                def __call__(self, candidate):
                    calls.append('original'); result=dict(valid=True)
                    self.audit.append(dict(result)); return result
            module=ModuleType('native_g1_execution_candidates'); module.CandidateCheck=Native
            with patch.dict(sys.modules, {'native_g1_execution_candidates':module}), \
                 patch('agentic_interaction_v81.revolute_certificate.evaluate',
                       return_value=self.certificate()) as chain:
                with candidate_gate(self.actions(), 1.1):
                    result=Native()(self.candidate())
                self.assertTrue(result['valid']); self.assertEqual(calls, ['original'])
                chain.assert_called_once()
                self.assertTrue(result['target_coverage']['full_requested_actions_feasible'])
                # A genuinely incomplete first target still cannot reach contact admission.
                partial=self.candidate(); partial['path'][-1]['state']=1.0
                with candidate_gate(self.actions(), 1.1):
                    self.assertFalse(Native()(partial)['valid'])
                self.assertEqual(calls, ['original'])
                # Real native chain rejection remains authoritative.
                chain.return_value=dict(valid=False, reason='REVOLUTE_CHAIN_NATIVE:collision')
                with candidate_gate(self.actions(), 1.1):
                    blocked=Native()(self.candidate())
                self.assertFalse(blocked['valid'])
                self.assertEqual(blocked['reason'], 'REVOLUTE_CHAIN_NATIVE:collision')

    def test_measured_first_reverse_reaches_checker_but_needs_full_certificate(self):
        # Active90 command 22: actual .334965, nominal horizon pi/2,
        # first reverse .23. All candidates were rejected before native checks.
        actions=self.actions()
        for a,q in zip(actions,[.23,1.5707963267948966,1.15]):a['target_q']=q
        candidate=self.candidate()
        candidate['path']=[dict(state=.3349651098251343),dict(state=1.5707963267948966)]
        certificate=dict(valid=True,endpoints=[dict(target_rad=a['target_q'],exit_valid=True) for a in actions])
        calls=[]
        with tempfile.TemporaryDirectory() as folder:
            class Native:
                def __init__(self):
                    self.audit=[];self.policy=self.config=self.solver=None
                    self.g=dict(job=dict(v81_integrated_repair=True,
                        articulation_v7=dict(joint_type='revolute')),a=SimpleNamespace(output=Path(folder)))
                def __call__(self,candidate):
                    calls.append('original');result=dict(valid=True)
                    self.audit.append(dict(result));return result
            module=ModuleType('native_g1_execution_candidates');module.CandidateCheck=Native
            with patch.dict(sys.modules,{'native_g1_execution_candidates':module}), \
                 patch('agentic_interaction_v81.revolute_certificate.evaluate',return_value=certificate) as native_chain:
                with candidate_gate(actions,1.5707963267948966) as audit:
                    result=Native()(candidate)
                self.assertTrue(result['valid'])
                self.assertTrue(audit[0]['deferred_to_revolute_whole_chain'])
                native_chain.assert_called_once()
                native_chain.return_value=dict(valid=False,reason='REVOLUTE_CHAIN_NATIVE:collision')
                with candidate_gate(actions,1.5707963267948966):
                    self.assertFalse(Native()(candidate)['valid'])
                native_chain.return_value=dict(valid=True,endpoints=[])
                with candidate_gate(actions,1.5707963267948966):
                    self.assertFalse(Native()(candidate)['valid'])
                self.assertEqual(len(calls),3)


if __name__ == '__main__':
    unittest.main()
