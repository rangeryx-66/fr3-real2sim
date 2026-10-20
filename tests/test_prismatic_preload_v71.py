import unittest
from interaction_runtime.prismatic_preload_v71 import preload_policy


class PreloadPolicyTest(unittest.TestCase):
    def test_only_request_changes(self):
        p=dict(preload_n=.5,finger_effort_limit_n=10,other={'a':1})
        for value in [.5,.75,1.]:
            q=preload_policy(p,value)
            self.assertEqual(q,{**p,'preload_n':value});self.assertEqual(p['preload_n'],.5)
            self.assertIs(q['other'],p['other'])

    def test_original_request_authority(self):
        for value in [0,-1,float('nan'),float('inf'),10.1]:
            with self.assertRaises(ValueError):preload_policy(dict(finger_effort_limit_n=10),value)


if __name__=='__main__':unittest.main()
