"""Bounded physical-step ownership at an already released parking cell."""
import unittest
from dataclasses import replace
from agentic_interaction_v81.station import Boundary
from agentic_interaction_v81.policy_checks import settle_station


class Runtime:
    dt=.01
    def __init__(self,frames):
        self.frames=frames;self.calls=0;self.g={};self.c={'home_q':[0]*7}
    def station_boundary(self,record=True):return self.frames[min(self.calls,len(self.frames)-1)]
    def ensure_execution_budget(self,phase):assert phase=='PREGRASP'
    def hold(self,dt):self.calls+=1


def boundary(**changes):
    b=Boundary('test',0,0,0.,(0,0,0,0),(0,)*7,(.06,)*6,0.,'m',True,True,True,True,True,None,'a'*64)
    return replace(b,**changes)


class SettleTests(unittest.TestCase):
    def test_requires_continuous_stability_after_transient(self):
        r=Runtime([boundary(robot_stationary=False)]*3+[boundary()]*40)
        settle_station(r);self.assertGreaterEqual(r.calls,32);self.assertLessEqual(r.calls,34)
    def test_never_waits_under_load_or_outside_parking_cell(self):
        for b in [boundary(unloaded_verified=False),boundary(arm_q=(.1,)*7),boundary(safety_latch='hard')]:
            r=Runtime([b])
            with self.assertRaises(ValueError):settle_station(r)
            self.assertEqual(r.calls,0)
    def test_persistent_motion_has_finite_budget(self):
        r=Runtime([boundary(robot_stationary=False)])
        with self.assertRaisesRegex(ValueError,'FINITE_HOLD'):settle_station(r,maximum_s=.1)
        self.assertEqual(r.calls,10)
    def test_existing_physical_guard_exception_propagates(self):
        r=Runtime([boundary()])
        def stop(_):raise RuntimeError('native collision')
        r.hold=stop
        with self.assertRaisesRegex(RuntimeError,'native collision'):settle_station(r)


if __name__=='__main__':unittest.main()
