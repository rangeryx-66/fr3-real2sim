"""Current-state transforms and refusal propagation, not a physics certificate."""
import unittest
from types import SimpleNamespace
import numpy as np
from interaction_runtime.articulation_v7 import Articulation
from agentic_interaction_v81.station_forecast import check_arc


class ForecastTests(unittest.TestCase):
    def exercise(self,block=False):
        a=Articulation('prismatic',(1,0,0),(0,0,0),(0,.5));T=np.eye(4);T[0,3]=.2
        targets=[];objects=[]
        def solve(target,base,seed,starts):targets.append(target.copy());return np.zeros(7)
        def collision(poses,moving,allowed):
            objects.append(moving.copy());return (not block or moving[0,3]<.025,'native cabinet collision')
        result=check_arc(a,T,np.eye(4),np.zeros(4),.1,.15,.02,
            SimpleNamespace(solve=solve,original_limits=np.tile([-2.,2.],(7,1))),
            lambda *args,**kwargs:{},SimpleNamespace(check=collision))
        return result,targets,objects
    def test_current_coordinate_is_the_delta_origin(self):
        result,targets,objects=self.exercise()
        self.assertTrue(result['full_arc_supported']);self.assertAlmostEqual(targets[0][0,3],.2)
        self.assertAlmostEqual(targets[-1][0,3],.25);self.assertAlmostEqual(objects[-1][0,3],.05)
        self.assertFalse(result['executable_route'])
    def test_collision_stops_forecast_without_claiming_no_other_grasp(self):
        result,_,_=self.exercise(True)
        self.assertFalse(result['full_arc_supported']);self.assertLess(result['reached_native'],.15)
        self.assertEqual(result['reason'],'native cabinet collision')
        self.assertTrue(result['tested_current_candidate_only'])


if __name__=='__main__':unittest.main()
