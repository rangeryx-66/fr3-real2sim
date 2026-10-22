import copy, hashlib, json, tempfile, unittest
from pathlib import Path
import numpy as np
from interaction_runtime.prismatic_lifecycle_v75 import evidence_identity,attempt_indices,provenance,equivalent_joint_targets,finite_admission
from prismatic_estimator_audit_v75 import matched_mask,perturb_raw,momentum_fit,applicability
from run_prismatic_synthetic_v75 import raw_fixture,processed,fit_pieces,CONFIG


class EvidenceIdentityTest(unittest.TestCase):
    def test_actual_plot_of_failed_closure_is_current_and_no_contact_stays_missing(self):
        from render_prismatic_feedback_v75 import render
        from scipy.spatial.transform import Rotation
        from run_prismatic_synthetic_v75 import PRIOR
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'telemetry').mkdir();n=600;rows,q,v,_=raw_fixture('FIXTURE',duration=n/240)
            def pose(T):return np.r_[T[:3,3],Rotation.from_matrix(T[:3,:3]).as_quat()]
            sensor=np.array([pose(r['wrist_ft']['T_world_sensor'])for r in rows]);raw=np.array([r['wrist_ft']['raw_wrench_sensor']for r in rows])
            duplicate=lambda x:np.concatenate([x,x])
            arrays=dict(step=np.arange(2*n),t=np.arange(2*n)/240,phase=np.r_[np.zeros(n,int),np.ones(n,int)],
                segment=np.r_[np.ones(n,int),np.zeros(n,int)],ft_valid=np.ones(2*n),
                joint_q=duplicate(q),joint_dq=duplicate(v),sensor_pose=duplicate(sensor),tool_poses=duplicate(sensor[:,None,:]),
                tcp_pose=duplicate(sensor),command_TCP_pose=duplicate(sensor),raw_ft=duplicate(raw),
                pad_load=np.r_[np.full((n,2),3.),np.zeros((n,2))],
                relative_slip_m=np.zeros(2*n),safety_stop=np.zeros(2*n),reference_q=np.full(2*n,np.nan),
                aperture=np.full(2*n,.02),command_aperture=np.full(2*n,.019))
            f=p/'telemetry/00000000.npz';np.savez_compressed(f,**arrays)
            meta=dict(phases=['OPEN_CONTINUE','CLOSE'],body_properties=[dict(name='analytical_tool',mass_kg=.8,com_body_m=[.01,0,.07],inertia_body_kg_m2=np.diag([.001,.002,.0015]).tolist())])
            index=dict(metadata=meta,chunks=[dict(file=f.name,first_step=0,last_step=2*n-1,sha256=hashlib.sha256(f.read_bytes()).hexdigest())])
            (p/'telemetry/index.json').write_text(json.dumps(index));(p/'identification_input.json').write_text(json.dumps(dict(prior=PRIOR,bias=[0]*6)))
            result=render(p,evidence_identity('e',2,1,n,2*n))
            self.assertEqual(result['samples'],n);self.assertEqual(result['raw_normal_peaks_n'],[0.,0.])
            self.assertFalse(result['previous_attempt_fallback']);self.assertFalse(result['contact_geometry']['available'])
            self.assertTrue(Path(result['path']).exists());self.assertGreater(result['full_compensation_samples'],0)

    def test_failed_closure_never_borrows_preceding_successful_segment(self):
        arrays=dict(step=np.arange(20),segment=np.r_[np.ones(10),np.zeros(10)],
            attempt_id_v75=np.r_[np.ones(10),np.full(10,2)],
            observation_generation_v75=np.r_[np.zeros(10),np.ones(10)])
        identity=evidence_identity('episode',2,1,10,20)
        np.testing.assert_array_equal(attempt_indices(arrays,identity),np.arange(10,20))
        self.assertFalse(arrays['segment'][attempt_indices(arrays,identity)].any())
        identity['observation_generation']=2;self.assertEqual(len(attempt_indices(arrays,identity)),0)

    def test_invalid_or_empty_identity_is_rejected(self):
        for args in [('e',0,0,0,2),('e',1,0,3,3),('e',True,0,0,2)]:
            with self.assertRaises(ValueError):evidence_identity(*args)

    def test_hash_mismatch_is_unavailable_not_stale_fallback(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'telemetry').mkdir();(p/'telemetry/a.npz').write_bytes(b'raw')
            ix=np.arange(2);a=dict(step=ix,t=ix/240)
            index=dict(chunks=[dict(file='a.npz',first_step=0,last_step=1,sha256='wrong')])
            with self.assertRaisesRegex(ValueError,'integrity'):
                provenance(p,index,ix,a,evidence_identity('e',1,0,0,2))


class JointAndBudgetTest(unittest.TestCase):
    def test_relative_reference_lead_does_not_move_drawer_with_robot(self):
        from types import SimpleNamespace
        from interaction_runtime.prismatic_chain_v75 import relative_lead_check,rail_lead_offsets
        from interaction_runtime.policy import Policy
        policy=Policy.load(Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json')
        model=SimpleNamespace(margin=lambda q:1.,poses=lambda q,base,**kw:float(q[0]))
        scene=SimpleNamespace(check=lambda pose,moving,allow:(pose-moving[0,3]>=-.002,'moving panel'))
        solver=SimpleNamespace(solve=lambda target,base,seed,starts:np.array([target[0,3]]))
        g=dict(model=model,collision=scene,base=None,dt=1/240,v75_motion_prior=dict(axis_world=[1.,0.,0.]))
        goal=np.eye(4);goal[0,3]=.02;moving=goal.copy()
        self.assertTrue(scene.check(.02,moving,True)[0])  # the R1 co-moving check
        r=relative_lead_check(g,goal,np.array([.02]),moving,[dict(width=.02)],policy,solver,np.zeros(1))
        self.assertFalse(r['valid']);self.assertIn('NATIVE_COLLISION',r['reason'])
        self.assertAlmostEqual(rail_lead_offsets(policy,1/240)[1],-.0037)

    def test_contact_plot_focus_excludes_unloaded_route_but_keeps_contact_peaks(self):
        from render_prismatic_feedback_v75 import contact_focus
        phases=['PREGRASP','SLOW_CLOSE','FORCE_HOLD','OPEN_CONTINUE','RELEASE','SAFE_PARK_MOTION']
        a=dict(t=np.arange(600)/240,phase=np.repeat(np.arange(6),100),pad_load=np.zeros((600,2)))
        a['pad_load'][220:400]=1.;a['pad_load'][250]=7.
        first,last=contact_focus(a,np.arange(600),dict(phases=phases))
        self.assertLessEqual(first,220);self.assertGreater(last,399);self.assertLess(last,600)
        self.assertEqual(a['pad_load'][first:last].max(),7.)

    def test_session_preserves_protocol_and_safety_control_flow(self):
        import ast
        root=Path(__file__).resolve().parents[1]
        old=next(n for n in ast.parse((root/'interaction_runtime/prismatic_v73.py').read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='serve_v73')
        new=next(n for n in ast.parse((root/'interaction_runtime/prismatic_session_v75.py').read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='serve_v75')
        new.name=old.name
        for n in ast.walk(new):
            if isinstance(n,ast.Name)and n.id=='PrismaticEvidenceOperationsV75':n.id='PrismaticOperationsV73'
        self.assertEqual(ast.dump(old),ast.dump(new))

    def test_winding_uses_only_explicit_periodic_joints_and_original_limits(self):
        rows=equivalent_joint_targets([0,0],[6.1,0],[[-7,7],[-.2,.2]],[0],.05,[1,1])
        self.assertAlmostEqual(rows[0]['q'][0],2*np.pi);self.assertEqual(rows[0]['q'][1],0)
        self.assertAlmostEqual(np.cos(rows[0]['q'][0]),1)
        unwrapped=equivalent_joint_targets([0,0],[6.1,0],[[-7,7],[-.2,.2]],[],.05)
        self.assertEqual(unwrapped[0]['q'],[0.,0.])
        for row in rows:self.assertTrue(np.all(abs(np.asarray(row['q']))<np.array([6.95,.15])))

    def test_impossible_rest_is_not_declared_park(self):
        self.assertEqual(equivalent_joint_targets([0],[0],[[1,2]],[0],.05),[])

    def costs(self):
        return {k:dict(available=True,wall_s=10,sim_s=5,source='measured fixture / sha256')
                for k in ('approach','closure_hold','FIT','VALIDATION','exit')}

    def test_complete_validation_and_exit_must_fit_before_acquisition(self):
        r=dict(wall_s=35,sim_s=30,reserved_wall_s=10,reserved_sim_s=5)
        result=finite_admission(r,self.costs());self.assertFalse(result['admit'])
        self.assertEqual(result['required']['wall_s'],40)
        r['wall_s']=40;self.assertTrue(finite_admission(r,self.costs())['admit'])

    def test_unknown_costs_cannot_be_assumed_zero(self):
        c=self.costs();c.pop('VALIDATION')
        self.assertEqual(finite_admission(dict(wall_s=100,sim_s=100),c)['status'],'COST_UNKNOWN')


class CausalLoadTest(unittest.TestCase):
    def policy(self):
        from interaction_runtime.policy import Policy
        return Policy.load(Path(__file__).resolve().parents[1]/'configs/interaction_runtime_v1.json')

    def test_constant_normal_load_does_not_cause_predictive_stop(self):
        from interaction_runtime.prismatic_load_v75 import LoadTrend
        monitor=LoadTrend(self.policy())
        for i in range(240):self.assertIsNone(monitor.update(i/240,1/240,[1,1],[0,0,2,0,0,.2],.2))

    def test_rising_load_requests_bounded_stop_before_existing_hard_band(self):
        from interaction_runtime.prismatic_load_v75 import LoadTrend
        monitor=LoadTrend(self.policy());first=None
        for i in range(300):
            moment=.2+i/240*.5
            r=monitor.update(i/240,1/240,[1,1],[0,0,2,0,0,moment],.15)
            if r is not None:first=r;break
        self.assertIsNotNone(first);self.assertEqual(first['affected'],[2])
        self.assertLess(first['causal_window_mean'][2],.6)
        self.assertEqual(first['unchanged_sustained_limits'],[4.,8.,.6])
        self.assertTrue(first['hard_guard_authoritative'])


class RawIdentificationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw=raw_fixture('FIT_ACCEL',duration=5.)

    def test_raw_scenarios_are_immutable_and_independent_across_roles(self):
        rows,q,v,_=self.raw;original=np.array([r['wrist_ft']['raw_wrench_sensor']for r in rows])
        scenario=dict(noise_sd_sensor=[.01]*6)
        a,_,_=perturb_raw(rows,q,v,scenario)
        other=copy.deepcopy(rows)
        for row in other:row['identification_segment']='VALIDATION_DISTINCT'
        b,_,_=perturb_raw(other,q,v,scenario)
        np.testing.assert_array_equal(original,np.array([r['wrist_ft']['raw_wrench_sensor']for r in rows]))
        self.assertFalse(np.allclose(a[0]['wrist_ft']['raw_wrench_sensor'],b[0]['wrist_ft']['raw_wrench_sensor']))

    def test_quantization_is_not_gaussian_noise_and_delay_is_causal(self):
        rows,q,v,_=self.raw
        a,_,_=perturb_raw(rows,q,v,dict(quantum_sensor=[.4]*3+[.01]*3,delay_samples=2))
        raw=np.array([r['wrist_ft']['raw_wrench_sensor']for r in a])
        np.testing.assert_allclose(raw/[.4,.4,.4,.01,.01,.01],np.round(raw/[.4,.4,.4,.01,.01,.01]),atol=1e-12)
        np.testing.assert_array_equal(raw[0],raw[2])
        with self.assertRaises(ValueError):perturb_raw(rows,q,v,dict(delay_samples=.5))
        with self.assertRaises(ValueError):perturb_raw(rows,q,v,dict(noise_sd_sensor=[-.1]*6))

    def test_eligibility_count_control_preserves_time_strata_not_outcome(self):
        t=np.arange(100)/10;old=np.ones(100,bool);strict=old.copy();strict[::3]=False
        matched,strata=matched_mask(old,strict,t)
        self.assertEqual(matched.sum(),strict.sum())
        for row in strata:
            group=(np.floor(t).astype(int)==row['stratum'])
            self.assertEqual(matched[group].sum(),strict[group].sum())

    def test_integral_balance_recovers_known_mass_without_acceleration_fit(self):
        pieces=[processed(raw_fixture(role,sign,speed,period,duration=6.),role)
                for role,sign,speed,period in [('FIT_A',1,.0025,3.1),('FIT_B',-1,.002,2.5),('FIT_C',1,.0015,4.3)]]
        model,rows=fit_pieces(pieces);integral=momentum_fit(pieces,CONFIG,model)
        self.assertEqual(integral['status'],'estimated')
        np.testing.assert_allclose(integral['parameters'],[2,3,.3],rtol=.005,atol=1e-4)

    def test_asymmetric_law_is_reported_untrusted_without_truth_selection(self):
        pieces=[processed(raw_fixture(role,sign,speed,period,plant='asymmetric',duration=6.),role)
                for role,sign,speed,period in [('FIT_A',1,.0025,3.1),('FIT_B',-1,.002,2.5),('FIT_C',1,.0015,4.3)]]
        model,rows=fit_pieces(pieces)
        self.assertTrue(applicability(rows,model)['status'].startswith('untrusted'))


if __name__=='__main__':unittest.main()
