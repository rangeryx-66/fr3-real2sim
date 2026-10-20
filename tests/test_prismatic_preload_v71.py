import unittest
from types import SimpleNamespace
from unittest.mock import patch
from interaction_runtime.prismatic_preload_v71 import preload_policy,PreloadOperations
from interaction_runtime.prismatic_feedback_v7 import PrismaticFeedbackOperations


class PreloadPolicyTest(unittest.TestCase):
    def test_only_request_changes(self):
        p=dict(preload_n=.5,finger_effort_limit_n=10,other={'a':1})
        for value in [.5,.75,1.]:
            q=preload_policy(p,value)
            self.assertEqual(q,{**p,'preload_n':value});self.assertEqual(p['preload_n'],.5)
            self.assertIs(q['other'],p['other'])

    def test_original_request_authority(self):
        for value in [0,-1,float('nan'),float('inf'),10.1,True,False]:
            with self.assertRaises(ValueError):preload_policy(dict(finger_effort_limit_n=10),value)

    def operation(self,latched=None,reference=None,loads=(0.,0.),settled=True):
        value=object.__new__(PreloadOperations)
        value.r=SimpleNamespace(supervisor=SimpleNamespace(latched=latched,force=SimpleNamespace(settled=lambda:settled)),
            grasp_reference=reference,filtered=lambda:loads,
            policy=SimpleNamespace(data={'force':{'unloaded_pad_n':.05}}),
            closure_policy=dict(preload_n=.5,finger_effort_limit_n=10),event=lambda *a,**k:None)
        value.g={};return value

    def test_no_loaded_or_latched_change(self):
        for args in [dict(latched={'reason':'force'}),dict(reference=object()),dict(loads=(.1,0)),dict(settled=False)]:
            o=self.operation(**args)
            with patch.object(PrismaticFeedbackOperations,'attempt') as parent:
                r=o.attempt(dict(native_g1_preload_request_n=.75));parent.assert_not_called()
            self.assertFalse(r['executed']);self.assertEqual(o.r.closure_policy['preload_n'],.5)

    def test_unloaded_change_reuses_existing_attempt(self):
        o=self.operation();original=o.r.closure_policy
        command=dict(native_g1_preload_request_n=.75,reason='observed insufficient rail traction')
        with patch.object(PrismaticFeedbackOperations,'attempt',return_value={'acquired':False}) as parent:
            self.assertEqual(o.attempt(command),{'acquired':False});parent.assert_called_once_with(command)
        self.assertEqual(original['preload_n'],.5);self.assertEqual(o.r.closure_policy['preload_n'],.75)


if __name__=='__main__':unittest.main()
