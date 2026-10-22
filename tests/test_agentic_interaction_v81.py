"""Offline contract tests; these deliberately do not stand in for native tests."""
from dataclasses import replace
import math
import unittest

from agentic_interaction_v81.preflight import fit_feedback, native_range, support_requirements
from agentic_interaction_v81.station import (
    Boundary,evaluate_station,invalidate_after_relocation,smooth_translation)


def boundary():
    return Boundary('engineering_00',0,0,10.0,(0.,0.,.237,56.3),
                    (0.,-.5,0.,1.,0.,.5,0.),(.1,)*6,.04,'m',
                    True,True,True,True,True,None,'a'*64)


class StationTests(unittest.TestCase):
    def test_motion_is_rest_to_rest_and_dynamically_bounded(self):
        path=smooth_translation((0,0,.2,0),(.3,.4,.2,0),.08,.25,1/240)
        self.assertEqual(path[0]['base'],[0.,0.,.2,0])
        self.assertEqual(path[-1]['base'],[.3,.4,.2,0])
        self.assertEqual(path[0]['velocity_xy'],[0.,0.])
        self.assertEqual(path[-1]['velocity_xy'],[0.,0.])
        for a,b in zip(path,path[1:]):
            self.assertLessEqual(math.hypot(*b['velocity_xy']),.08+1e-12)
            acc=math.hypot(*[(y-x)/(b['t']-a['t']) for x,y in zip(a['velocity_xy'],b['velocity_xy'])])
            self.assertLessEqual(acc,.25+1e-12)

    def test_no_assumed_yaw_or_height_authority(self):
        for target in [(0,0,.25,0),(0,0,.2,90)]:
            with self.assertRaisesRegex(ValueError,'XY_STAGE'):
                smooth_translation((0,0,.2,0),target,.08,.25,.01)

    def test_hard_latch_cannot_be_bypassed_by_clearance(self):
        with self.assertRaisesRegex(ValueError,'HARD_STOP'):
            replace(boundary(),safety_latch='COLLISION').check()

    def test_loaded_or_unverified_state_cannot_relocate(self):
        for key in ['unloaded_verified','released','clearance_verified',
                    'robot_stationary','object_stationary']:
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'UNLOADED'):
                replace(boundary(),**{key:False}).check()

    def test_native_released_geometry_and_object_state_reach_callbacks(self):
        state=boundary();seen=[]
        def swept(s,path):
            seen.append(s)
            self.assertEqual(s.finger_q,state.finger_q)
            self.assertEqual(s.object_coordinate,.04)
            return dict(valid=True)
        result=evaluate_station(state,(.1,.1,.237,56.3),speed=.08,acceleration=.25,
            collision_interval_s=.02,validate_swept_route=swept,
            validate_destination=lambda s,t:dict(valid=True))
        self.assertTrue(result['valid']);self.assertGreaterEqual(len(seen),1)
        self.assertFalse(result['physical_motion_executed'])

    def test_rejected_direct_route_does_not_prove_no_exit(self):
        calls=[]
        def swept(s,path):
            calls.append(path)
            return dict(valid=len(calls)>1,reason='direct blocked' if len(calls)==1 else None)
        result=evaluate_station(boundary(),(.2,.2,.237,56.3),speed=.08,acceleration=.25,
            collision_interval_s=.02,validate_swept_route=swept,
            validate_destination=lambda s,t:dict(valid=True))
        self.assertTrue(result['valid']);self.assertFalse(result['alternatives'][0]['valid'])

    def test_exit_feasibility_is_required_at_destination(self):
        result=evaluate_station(boundary(),(.2,.2,.237,56.3),speed=.08,acceleration=.25,
            collision_interval_s=.02,validate_swept_route=lambda s,p:dict(valid=True),
            validate_destination=lambda s,t:dict(valid=False,reason='no legal park'))
        self.assertFalse(result['valid']);self.assertIsNone(result['selected'])

    def test_relocation_invalidates_cached_grasp_and_observation_routes(self):
        a=boundary();b=replace(a,station_generation=1,observation_generation=1,
            simulation_timestamp_s=20,base_xyzyaw=(.1,.2,.237,56.3))
        result=invalidate_after_relocation(a,b)
        self.assertTrue(result['reobserve_required'])
        self.assertIn('visual_plan',result['invalidated'])
        self.assertFalse(result['physical_base_execution_verified'])

    def test_cannot_reuse_old_generation_or_different_scene(self):
        a=boundary()
        for b in [a,replace(a,episode_id='other'),replace(a,station_generation=1,
                  simulation_timestamp_s=20,observation_generation=0)]:
            with self.subTest(state=b),self.assertRaisesRegex(ValueError,'LINEAGE'):
                invalidate_after_relocation(a,b)


class IdentificationPlanningTests(unittest.TestCase):
    def test_frozen_support_not_an_arbitrary_longer_timeout(self):
        s=support_requirements()
        self.assertEqual(s['minimum_process_records'],247)
        self.assertEqual(s['required_contiguous_records'],963)
        self.assertAlmostEqual(s['minimum_observed_span_s'],962/240)

    def test_completed_reverse_action_is_not_qualified_reverse_fit(self):
        model=dict(segments=[dict(id=1,role='FIT_PULL',blocks=[dict(records=1080,valid=838)]),
            dict(id=2,role='FIT_REVERSE',quality=dict(action_ok=True),
                 blocks=[dict(records=240,status='insufficient local filter support')])])
        result=fit_feedback(model)
        self.assertEqual(result['eligible_trajectory_count'],1)
        self.assertTrue(result['any_declared_FIT_without_support'])

    def test_validation_cannot_fill_missing_fit(self):
        r=fit_feedback(dict(segments=[dict(id=3,role='VALIDATION_REVERSE',blocks=[dict(valid=1000)])]))
        self.assertEqual(r['eligible_trajectory_count'],0)
        self.assertFalse(r['validation_used'])

    def test_unsupported_inertia_stays_unknown(self):
        r=fit_feedback(dict(selected=dict(parameter_names=['I','b','C'],parameters=[.1,1.,.02],
                                       conditionally_supported=[False,True,True])))
        self.assertIsNone(r['conditional_parameters']['I'])

    def test_qualification_counts_take_precedence_over_prequalification(self):
        r=fit_feedback(dict(segments=[dict(id=1,role='FIT_X',blocks=[dict(valid=300,
             prismatic_sliding_qualification=dict(applied=True,qualified_samples=0))])]))
        self.assertEqual(r['eligible_trajectory_count'],0)

    def test_native_range_does_not_claim_loaded_reachability(self):
        r=native_range('revolute',(0,math.radians(100)),math.pi/2)
        self.assertTrue(r['inside_native_limits'])
        self.assertFalse(r['physical_reachability_verified'])
        self.assertFalse(native_range('prismatic',(0,.1),.15)['inside_native_limits'])

    def test_bad_sampling_and_window_rejected(self):
        for kw in [dict(dt=0),dict(dt=float('nan')),dict(window=120),dict(edge=-1),dict(desired_retained_s=0)]:
            with self.subTest(kw=kw),self.assertRaises(ValueError):support_requirements(**kw)


if __name__=='__main__':unittest.main()
