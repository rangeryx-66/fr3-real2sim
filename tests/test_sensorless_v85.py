import tempfile,json,hashlib,unittest
from pathlib import Path
import numpy as np
from agentic_interaction_v85.effort import virtual_work,kinematic_scale_ambiguity,load_allowed,FIELDS,contiguous,known_geometry


class SensorlessTests(unittest.TestCase):
    def test_virtual_work_recovers_generalized_effort_without_wrench_fit(self):
        rng=np.random.default_rng(5);J=rng.normal(size=(6,7));w=rng.normal(size=6);S=rng.normal(size=6)
        r=virtual_work(J,S,J.T@w)
        self.assertAlmostEqual(r['generalized_effort'],float(S@w),places=11)
        self.assertLess(r['compatibility_error'],1e-12)

    def test_common_world_rotation_preserves_virtual_work(self):
        rng=np.random.default_rng(8);J=rng.normal(size=(6,7));w=rng.normal(size=6);S=rng.normal(size=6)
        R=np.array([[0,-1,0],[1,0,0],[0,0,1]]);W=np.zeros((6,6));W[:3,:3]=R;W[3:,3:]=R
        a=virtual_work(J,S,J.T@w);b=virtual_work(W@J,W@S,J.T@w)
        self.assertAlmostEqual(a['generalized_effort'],b['generalized_effort'],places=11)

    def test_singular_pose_cannot_silently_output_plausible_force(self):
        with self.assertRaisesRegex(ValueError,'RANK_DEFICIENT'):
            virtual_work(np.ones((6,7)),np.ones(6),np.ones(7))

    def test_kinematics_cannot_fix_absolute_parameter_scale(self):
        v=np.array([-.1,.2,.3]);acc=np.array([.2,.4,-.5])
        u,other=kinematic_scale_ambiguity([.1,.2,.03],7,acc,v)
        np.testing.assert_allclose(other,7*u)
        with self.assertRaisesRegex(ValueError,'MUST_NOT_RECEIVE_EFFORT'):
            kinematic_scale_ambiguity([.1,.2,.03],7,acc,v,effort=u)

    def test_loader_never_reads_poison_wrench_or_native_object_q(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);t=p/'telemetry';t.mkdir()
            values={k:np.zeros((2,))for k in FIELDS}
            values['raw_ft']=np.array([object()],object);values['joint_q']=np.array([object()],object)
            f=t/'x.npz';np.savez(f,**values)
            index=dict(chunks=[dict(file=f.name,sha256=hashlib.sha256(f.read_bytes()).hexdigest())])
            (t/'index.json').write_text(json.dumps(index));a,_=load_allowed(p)
            self.assertEqual(set(a),set(FIELDS));self.assertNotIn('raw_ft',a);self.assertNotIn('joint_q',a)

    def test_corrupt_cache_fails_before_use(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);t=p/'telemetry';t.mkdir();f=t/'x.npz';f.write_bytes(b'bad')
            (t/'index.json').write_text(json.dumps(dict(chunks=[dict(file=f.name,sha256='0'*64)])))
            with self.assertRaisesRegex(ValueError,'HASH_MISMATCH'):load_allowed(p)

    def test_geometry_provider_never_opens_FT_configuration(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);expected=dict(joint_type='revolute',axis_world=[0,0,1],origin_world_m=[0,0,0])
            (p/'ARTICULATION_INTERFACE.json').write_text(json.dumps(dict(prior=expected)))
            (p/'identification_input.json').write_text('POISON F/T CALIBRATION')
            self.assertEqual(known_geometry(p),expected)

    def test_sampling_gap_and_role_boundaries_are_not_interpolated(self):
        a=dict(step=np.array([0,1,2,4,5,6]),t=np.array([0,1,2,4,5,6])/240,
               phase=np.array([0,0,0,0,0,0]),segment=np.array([1,1,2,2,2,2]))
        self.assertEqual([list(r)for r in contiguous(a)],[[0,1],[2],[3,4,5]])


if __name__=='__main__':unittest.main()
