import unittest
import numpy as np
from interaction_runtime.prismatic_startup import PrismaticStartup
from interaction_runtime.contact_evidence_v7 import active_friction


def sample(t,q=0.,dq=0.,command=.0006,tcp=0.,force=0.,contact=None):
    return dict(t=t,q=q,dq=dq,command_rail=command,tcp_rail=tcp,rail_effort=force,contact=contact or {})


def startup(direction=1):
    initial=sample(0.)
    return PrismaticStartup(initial,[initial]*240,dict(target_q=direction*.02,reference_speed_m_s=.003),dict(speed_m_s=.012,acceleration_m_s2=.04),.00185,1/240)


class StartupTests(unittest.TestCase):
    def test_compliant_loading_precedes_resolved_sliding_without_false_stall(self):
        s=startup()
        for t in np.arange(0,4,1/240):
            q=max(t-2,0)*.0005;tcp=min(.0006*t,.0012)+q;command=.0006+min(.001*t,.00185)+q
            s.observe(sample(t,q,.0005 if t>2 else 0.,command,tcp,.15*min(t/2,1)))
            self.assertIsNone(s.stop_reason)
        self.assertTrue(s.sliding);self.assertGreater(s.sliding_started_s,2.)
        self.assertFalse(s.packet()['grasp_reference_reset'])

    def test_tracking_deficit_is_not_labelled_excessive_object_friction(self):
        s=startup()
        for t in np.arange(0,7,1/240):
            s.observe(sample(t,command=.0006+min(t*.001,.00185)))
            if s.stop_reason:break
        self.assertIn('TRACKING_LIMITED',s.stop_reason);self.assertLess(t,7.)

    def test_actual_tangential_relative_motion_with_traction_plateau_is_distinct(self):
        s=startup();contact=dict(traction_available=True,traction_on_object_rail_N=.1,slip_velocity_available=True,relative_tangent_speed_m_s=.001)
        for t in np.arange(0,7,1/240):
            x=min(t*.001,.00185);s.observe(sample(t,command=.0006+x,tcp=x,force=.1,contact=contact))
            if s.stop_reason:break
        self.assertIn('TANGENTIAL_SLIP',s.stop_reason)

    def test_static_loading_is_unresolved_resistance_not_fabricated_slip(self):
        s=startup()
        for t in np.arange(0,7,1/240):
            # A bounded compressed contact with little surface displacement.
            x=min(t*.00001,.00004);s.observe(sample(t,command=.0006+x,tcp=x,force=min(.1*t,.3)))
            if s.stop_reason:break
        self.assertIn('STATIC_RESISTANCE_OR_CONSTRAINT_UNRESOLVED',s.stop_reason)

    def test_reverse_motion_and_single_pad_report_do_not_add_a_veto(self):
        s=startup(-1)
        for t in np.arange(0,3,1/240):
            x=-.001*t;s.observe(sample(t,x,-.001,.0006+x,x,-.2))
        self.assertTrue(s.sliding);self.assertIsNone(s.stop_reason)

    def test_persistent_stall_after_sliding_stops_finitely(self):
        s=startup()
        for t in np.arange(0,5,1/240):
            q=.001*min(t,1);dq=.001 if t<1 else 0
            s.observe(sample(t,q,dq,.0006+q,q,.2))
            if s.stop_reason:break
        self.assertIn('PERSISTENT_SLIDING_STALL',s.stop_reason);self.assertLess(t,4.)

    def test_native_friction_pair_ranges_and_zero_reports_are_preserved(self):
        force=np.arange(18).reshape(6,3);point=force*.1
        f,p=active_friction((force,point,[[2,1]],[[1,4]]))
        np.testing.assert_array_equal(f,force[[1,2,4]])
        self.assertEqual(active_friction((force,point,[[0]],[[0]]))[0].shape,(0,3))
        with self.assertRaises(ValueError):active_friction((force,point,[[7]],[[0]]))
        with self.assertRaises(ValueError):active_friction((force,point,[[2,2]],[[0,1]]))

    def test_tracking_offset_uses_actual_incremental_deficit_and_is_finite(self):
        s=startup();q,p=s.clock_feedback(sample(2,q=.0001,command=.0025,tcp=.0009))
        self.assertAlmostEqual(p['tracking_offset_m'],.001)
        self.assertAlmostEqual(q,.0011)
        q,p=s.clock_feedback(sample(3,command=.020,tcp=0.))
        self.assertAlmostEqual(p['tracking_offset_m'],s.lookahead)
        self.assertAlmostEqual(p['total_reference_lead_bound_m'],2*s.lookahead)
        _,p=s.clock_feedback(sample(4,command=.0006,tcp=.001))
        self.assertEqual(p['tracking_offset_m'],0.)
        s=startup(-1);q,p=s.clock_feedback(sample(2,q=-.0001,command=-.0013,tcp=-.0009))
        self.assertAlmostEqual(q,-.0011)

    def test_compliant_plant_deadlock_and_tracking_compensation_without_force_gain(self):
        from interaction_runtime.progress import ProgressClock
        # Synthetic consistency check, not a fit to hidden simulator truth.
        # Same two series compliances, mass/friction and speed in both replays.
        def replay(compensate):
            dt=1/240;s=startup();s.initial=sample(0.,command=0.);clock=ProgressClock(0,.012,lookahead=s.lookahead)
            q=v=u=x=0.;maximum_lead=0.
            for i in range(4800):
                feedback,info=s.clock_feedback(sample(i*dt,q,v,u,x)) if compensate else (q,{})
                if compensate:clock.target=.012+info['tracking_offset_m']
                u,_=clock.advance(feedback,.001,1.,dt,.001,.01,.01)
                force=(1/(1/150+1/500))*(u-q)
                a=0. if abs(v)<1e-8 and abs(force)<=.3 else (force-.3*np.sign(v if abs(v)>1e-8 else force)-3*v)/2
                v+=a*dt;q+=v*dt;x=u-force/150
                maximum_lead=max(maximum_lead,u-q)
                if q>=.012:break
            return q,maximum_lead
        old,_=replay(False);new,lead=replay(True)
        self.assertLess(old,1e-5);self.assertGreaterEqual(new,.012)
        self.assertLessEqual(lead,2*.00185+1e-5)


if __name__=='__main__':unittest.main()
