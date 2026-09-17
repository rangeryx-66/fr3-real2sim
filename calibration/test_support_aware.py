"""Regression tests for supported pivot, short lift, true drop and free slip."""
import unittest
import numpy as np
from support_aware import SupportMonitor

VERTICES=np.array([[x,y,z] for x in [-.025,.025] for y in [-.025,.025] for z in [-.025,.025]])
class SupportTests(unittest.TestCase):
    def monitor(self):
        m=SupportMonitor(VERTICES);m.begin(.025);return m
    def feed(self,m,start,duration,z,tcp_z,force=(15,15)):
        for i in range(int(duration*240)):
            m.sample(start+i/240,np.array([.5,0,z]),np.array([1,0,0,0]),np.array([.5,0,tcp_z]),np.array([1,0,0,0]),np.array(force))
    def test_supported_tcp_motion_cannot_be_slip(self):
        m=self.monitor()
        for i in range(480):m.sample(i/240,[.5,0,.025],[1,0,0,0],[.5,0,.06+i*.0001],[1,0,0,0],[15,15])
        q=m.query();self.assertEqual(q['category'],'SUPPORTED_SETTLING');self.assertFalse(q['flags']['DROP']);self.assertIsNone(q['free_metrics'])
    def test_retained_short_lift_is_not_drop(self):
        m=self.monitor();self.feed(m,0,3,.098,.13)
        o=m.hold_outcome(.5);self.assertEqual(o['category'],'INSUFFICIENT_LIFT')
        self.assertTrue(o['stability']['flags']['RETAINED']);self.assertFalse(o['stability']['flags']['DROP'])
    def test_success_requires_full_hold_and_height(self):
        m=self.monitor();self.feed(m,0,3,.111,.15)
        self.assertTrue(m.hold_outcome(.5)['success'])
        self.assertFalse(m.hold_outcome(2.)['success'])
    def test_actual_loss_and_fall_to_table(self):
        m=self.monitor();self.feed(m,0,1,.111,.15);self.feed(m,1,1,.025,.15,(0,0))
        self.assertEqual(m.query()['category'],'DROP')
    def test_free_space_slip(self):
        m=self.monitor();self.feed(m,0,1,.111,.15)
        for i in range(240):m.sample(1+i/240,[.5,0,.111-i*.00002],[1,0,0,0],[.5,0,.15],[1,0,0,0],[15,15])
        self.assertEqual(m.query()['category'],'CONTINUOUS_SLIP')
    def test_held_object_recontact_restarts_support_phase(self):
        m=self.monitor();self.feed(m,0,1,.04,.08);self.feed(m,1,1,.025,.08)
        q=m.query();self.assertEqual(q['category'],'SUPPORTED_SETTLING')
        self.assertIsNone(q['clear_t']);self.assertTrue(q['flags']['CLEAR_TABLE'])
        self.assertFalse(q['flags']['DROP']);self.assertEqual(q['support_returns'],1)
    def test_gap_cannot_be_hidden_by_later_contact(self):
        m=self.monitor();self.feed(m,0,1,.025,.08,(15,0));self.feed(m,1,1,.025,.08)
        self.assertEqual(m.query()['category'],'CONTACT_LOSS')
    def test_height_invariant_to_usd_origin_offset(self):
        a=self.monitor();shift=np.array([.02,-.03,.10]);b=SupportMonitor(VERTICES+shift);b.begin(.025)
        for i in range(720):
            position=np.array([.5,0,.111]);q=[1,0,0,0];tcp=[.5,0,.15]
            a.sample(i/240,position,q,tcp,q,[15,15]);b.sample(i/240,position-shift,q,tcp,q,[15,15])
        self.assertAlmostEqual(a.query()['net_lift_m'],b.query()['net_lift_m'])
        self.assertAlmostEqual(a.query()['mesh_bottom_z_m'],b.query()['mesh_bottom_z_m'])
        self.assertEqual(a.hold_outcome(.5)['success'],b.hold_outcome(.5)['success'])
if __name__=='__main__':unittest.main()
