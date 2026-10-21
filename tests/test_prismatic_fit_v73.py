import unittest
import tempfile
from unittest.mock import patch
from pathlib import Path
import numpy as np
from update_prismatic_v73_model import qualify


class QualificationTest(unittest.TestCase):
    def piece(self,velocity,role='FIT_TEST'):
        return dict(native_qd=np.array(velocity),segments=[role],block_id='test',valid=np.ones(len(velocity),bool),processing=dict(centered_future_support_samples=2,minimum_absolute_native_velocity=1e-4))

    def test_static_neighbor_cannot_enter_sliding_fit_through_filter(self):
        d=self.piece([.002]*20+[0]*3+[.002]*20);r=qualify(d)
        self.assertFalse(d['valid'][18:25].any());self.assertTrue(d['valid'][3:17].all());self.assertGreater(r['excluded_samples'],3)

    def test_direction_reversal_not_rescued_by_filter_mean(self):
        d=self.piece([.002]*20+[-.002]*20);qualify(d)
        self.assertFalse(d['valid'][18:22].any());self.assertTrue(d['valid'][4:15].all());self.assertTrue(d['valid'][25:35].all())

    def test_independent_validation_remains_byte_identical(self):
        d=self.piece([.002]*20+[0]*5+[-.002]*20,'VALIDATION_TEST');before=d['valid'].tobytes();r=qualify(d)
        self.assertFalse(r['applied']);self.assertEqual(before,d['valid'].tobytes())

    def test_original_invalid_samples_never_reenter_fit(self):
        d=self.piece([.002]*40);d['valid'][8:12]=False;qualify(d);self.assertFalse(d['valid'][8:12].any())

    def test_revolute_cannot_activate_prismatic_worker_and_loader_restores(self):
        import update_prismatic_v73_model as worker
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(worker.frozen,'load_segments',return_value=([],dict(prior=dict(joint_type='revolute')),[])) as original:
                with self.assertRaisesRegex(ValueError,'Prismatic-only'):worker.update(Path(folder))
                self.assertIs(worker.frozen.load_segments,original)

    def test_known_sliding_parameters_survive_static_force_transients(self):
        from xarm_ft_v31_estimator import common_filter
        from xarm_ft_v7_estimator import samples,fit
        dt=1/240;t=np.arange(0,30,dt);v=.005+.002*np.sin(2*np.pi*t/2.7)
        static=(t>12)&(t<15);v[static]=0;acc=np.gradient(v,dt)
        effort=2*acc+3*v+.3*np.sign(v)
        # Static contact can exert less than Coulomb breakaway resistance.
        effort[static]=.12
        valid=np.ones(len(t),bool);valid[:121]=False;valid[-121:]=False
        d=dict(t=t,q=np.cumsum(v)*dt,native_qd=v,qd=common_filter(v),qdd=common_filter(acc),direction=common_filter(np.sign(v)),torque=common_filter(effort),valid=valid,segments=np.full(len(t),'FIT_SYNTHETIC'),block_id='synthetic#0',processing=dict(window=121,centered_future_support_samples=62,minimum_absolute_native_velocity=1e-4))
        config=dict(domain_native=[0,.3],intervals=[2],joint_type='prismatic',units=dict(coordinate='m',effort='N'),parameter_scales=[3,5,.5],ridge=1e-8)
        qualify(d);rows=samples(d,config['domain_native'],config['intervals']);m=fit(rows,config)
        np.testing.assert_allclose(m['parameters'],[2,3,.3],atol=1e-6)

if __name__=='__main__':unittest.main()
