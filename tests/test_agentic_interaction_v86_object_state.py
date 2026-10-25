"""All-object state monitoring must remain passive and time aligned."""
from types import SimpleNamespace
import unittest
import numpy as np

from agentic_interaction_v81.object_state_telemetry import instrument


def fixture(enabled=True):
    calls=[]
    q=np.array([.06, .002]);dq=np.array([.01, -.001])
    articulation=SimpleNamespace(dof_names=['target_rail','passive_rail'],
        get_joint_positions=lambda:(calls.append('q') or q),
        get_joint_velocities=lambda:(calls.append('dq') or dq))
    record=SimpleNamespace(count=0,metadata={},coordinate_fields=lambda row:{'t':row['t'],'joint_q':row['selected']})
    def prior(row,poses): calls.append(('original',row['t']));row['original']=True
    r=SimpleNamespace(job={'v86_object_state_telemetry':enabled},record=record,
        scene={'articulation':articulation,'selected_asset_dof':0},enrich_measurement=prior)
    return r,calls,q,dq


class ObjectStateTests(unittest.TestCase):
    def test_opt_in_constructor_and_idempotence(self):
        r,calls,_,_=fixture(False);old=r.enrich_measurement
        self.assertEqual(instrument(r),{'enabled':False});self.assertIs(r.enrich_measurement,old)
        self.assertEqual(calls,[])
        r.job['v86_object_state_telemetry']=1
        with self.assertRaisesRegex(ValueError,'BOOLEAN'):instrument(r)
        r.job['v86_object_state_telemetry']=True;r.record.count=1
        with self.assertRaisesRegex(ValueError,'BEFORE_FIRST'):instrument(r)
        r.record.count=0;meta=instrument(r);method=r.enrich_measurement
        r.record.count=10;self.assertEqual(instrument(r),meta);self.assertIs(r.enrich_measurement,method)
        meta['dof_names'].append('changed');self.assertEqual(len(r.record.metadata['v86_object_state']['dof_names']),2)

    def test_one_tick_preserves_selected_state_and_copies_passive_state(self):
        r,calls,q,dq=fixture();meta=instrument(r)
        self.assertEqual(calls,[]);row={'t':12.5,'selected':.06}
        r.enrich_measurement(row,{})
        self.assertEqual(calls,[('original',12.5),'q','dq'])
        values=r.record.coordinate_fields(row)
        self.assertEqual(values['t'],12.5);self.assertEqual(values['joint_q'],.06)
        np.testing.assert_array_equal(values['object_all_q_native'],[.06,.002])
        self.assertEqual(values['object_all_q_native_valid'],1)
        self.assertEqual(meta['passive_dofs'],[1]);self.assertFalse(meta['object_actuation'])
        self.assertFalse(meta['feeds_controller_or_estimator'])
        q[:]=99;dq[:]=99
        np.testing.assert_array_equal(values['object_all_q_native'],[.06,.002])
        values['object_all_q_native'][:]=88
        np.testing.assert_array_equal(row['object_all_q_native'],[.06,.002])

    def test_missing_or_broken_getter_is_numeric_unavailable_not_control_failure(self):
        r,calls,_,_=fixture()
        def broken():raise RuntimeError('optional query unavailable')
        r.scene['articulation'].get_joint_positions=broken
        r.scene['articulation'].get_joint_velocities=lambda:np.array([np.nan,0.])
        instrument(r);row={'t':1.,'selected':.06};r.enrich_measurement(row,{})
        values=r.record.coordinate_fields(row)
        for name in ('object_all_q_native','object_all_dq_native'):
            self.assertEqual(values[name].shape,(2,));self.assertTrue(np.isnan(values[name]).all())
            self.assertEqual(values[name+'_valid'],0)
        self.assertTrue(row['original']);self.assertEqual(values['joint_q'],.06)

    def test_npz_roundtrip_keeps_passive_motion_and_no_object_dtype(self):
        import tempfile
        r,calls,q,dq=fixture();instrument(r);rows=[]
        for tick in range(3):
            q[1]=tick*.001;row={'t':tick/240,'selected':q[0]}
            r.enrich_measurement(row,{});rows.append(r.record.coordinate_fields(row))
        arrays={k:np.asarray([row[k]for row in rows])for k in rows[0]}
        with tempfile.TemporaryFile() as f:
            np.savez_compressed(f,**arrays);f.seek(0)
            with np.load(f,allow_pickle=False) as z:
                np.testing.assert_allclose(z['object_all_q_native'][:,1],[0,.001,.002])
                self.assertEqual(z['object_all_q_native'].dtype,np.float64)


if __name__=='__main__':unittest.main()
