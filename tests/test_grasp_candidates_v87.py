import json,math,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from agentic_interaction_v87.grasp_candidates import transforms,pose_id,novel_shortlist
from agentic_interaction_v87.activation import activate


class CandidateTests(unittest.TestCase):
    def payload(self):
        W=np.eye(4);W[:3,3]=[1,2,3]
        C=np.eye(4);C[:3,3]=[.2,.3,.4]
        G=np.eye(4);G[2,3]=.172
        item=dict(rank=0,score=.9,T_C_G=C.tolist(),T_G_TCP=G.tolist())
        return dict(gripper='XARM7_NATIVE_G1',frame='camera_optical',T_B_C=W.tolist(),grasps=[item]),dict(T_world_camera_optical=W.tolist()),W@C@G

    def test_camera_and_tcp_transforms_are_applied_exactly_once(self):
        payload,camera,expected=self.payload();result=transforms(payload,camera)
        np.testing.assert_allclose(result[0]['T'],expected)
        self.assertEqual(result[0]['pose_id'],pose_id(expected))

    def test_foreign_gripper_stale_camera_and_nonrigid_pose_are_rejected(self):
        payload,camera,_=self.payload();payload['gripper']='Dex1'
        with self.assertRaises(ValueError):transforms(payload,camera)
        payload,camera,_=self.payload();camera['T_world_camera_optical'][0][3]+=1
        with self.assertRaises(ValueError):transforms(payload,camera)
        payload,camera,_=self.payload();payload['grasps'][0]['T_C_G'][0][0]=2
        with self.assertRaises(ValueError):transforms(payload,camera)

    def test_duplicate_poses_are_not_repeated_search_candidates(self):
        payload,camera,_=self.payload();payload['grasps']*=2
        self.assertEqual(len(transforms(payload,camera)),1)

    def test_search_diversity_does_not_change_pose_or_safety_score(self):
        candidates=[]
        for x in [0,.001,.02,.04]:
            T=np.eye(4);T[0,3]=x;candidates.append(dict(T=T,contact=dict(rank=1-x)))
        selected=novel_shortlist(candidates,2)
        self.assertIs(selected[0],candidates[0]);self.assertIs(selected[1],candidates[2])
        self.assertEqual(selected[1]['T'][0,3],.02)

    def test_grasp_resources_require_explicit_native_registration(self):
        def existing(spec,job):job['original']=True
        job={};activate(dict(graspgenx=dict(python='/bin/python',source='/models/graspgenx',gpu=0)),job,existing,'base')
        self.assertTrue(job['original']);self.assertEqual(job['v87_graspgenx']['gpu'],0)
        for bad in [dict(python='python',source='/models',gpu=0),dict(python='/bin/python',source='/models',gpu=-1)]:
            with self.assertRaises(ValueError):activate(dict(graspgenx=bad),{},existing,'base')


if __name__=='__main__':unittest.main()
