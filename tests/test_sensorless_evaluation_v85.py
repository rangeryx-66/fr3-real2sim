"""Read-only evaluation gates and integration checks; no simulator or real GT."""
import json,os,tempfile,unittest,hashlib
from pathlib import Path
import numpy as np
from agentic_interaction_v85.diagnostic import half_step_sensitivity,slice_piece
from agentic_interaction_v85.independent_parameter_evaluator import evaluate
from xarm_ft_v7_estimator import prediction


class EvaluationTests(unittest.TestCase):
    def fixture(self,d):
        p=Path(d);f=p/'MODELS.json';f.write_text('{}');out=p/'errors.json'
        return p,f,out,hashlib.sha256(f.read_bytes()).hexdigest()

    def test_changed_model_refused_before_any_truth_read(self):
        with tempfile.TemporaryDirectory() as d:
            p,f,o,h=self.fixture(d)
            with self.assertRaisesRegex(ValueError,'HASH_CHANGED'):evaluate(p,f,o,'0'*64)
            self.assertFalse(o.exists())

    def test_existing_evaluation_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            p,f,o,h=self.fixture(d);o.write_text('preserved')
            with self.assertRaisesRegex(ValueError,'PRESERVE_PREVIOUS'):evaluate(p,f,o,h)
            self.assertEqual(o.read_text(),'preserved')

    def test_online_actor_cannot_trigger_truth_read(self):
        with tempfile.TemporaryDirectory() as d:
            p,f,o,h=self.fixture(d);(p/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(actor_exited=False)))
            with self.assertRaisesRegex(ValueError,'ACTOR_STILL_ACTIVE'):evaluate(p,f,o,h)

    @unittest.skipUnless(Path('/proc').exists(),'Linux PID gate')
    def test_live_pid_overrides_false_closed_metadata(self):
        with tempfile.TemporaryDirectory() as d:
            p,f,o,h=self.fixture(d);(p/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(actor_exited=True)))
            (p/'PROCESS.json').write_text(json.dumps(dict(pid=os.getpid())))
            with self.assertRaisesRegex(ValueError,'STILL_ALIVE'):evaluate(p,f,o,h)

    def test_feature_times_remain_aligned_after_common_horizon_crop(self):
        t=np.arange(10.);source=dict(t=t,q=t,features={2:np.c_[t,t*2]})
        cut=slice_piece(source,np.array([4,5,6]))
        np.testing.assert_equal(cut['features'][2][:,0],cut['t'])
        short=slice_piece(cut,slice(None,2));np.testing.assert_equal(short['features'][2][:,0],short['t'])

    def test_numerical_half_step_converges_with_same_input(self):
        t=np.arange(101)*.01;mass=1.;b=.2;c=.1;u=.6;v0=.2
        vinf=(u-c)/b;v=vinf+(v0-vinf)*np.exp(-b*t/mass)
        q=vinf*t+(v0-vinf)*(mass/b)*(1-np.exp(-b*t/mass))
        piece=dict(t=t,q=q,native_qd=v,torque=np.full(len(t),u),qd=v,qdd=np.gradient(v,t),direction=np.ones(len(t)),processing=dict(boundary_exclusion_samples=0))
        model=dict(parameters=[mass,b,c],domain_native=[-10,10],support_intervals_native=[[-10,10]],intervals=0,units={},active_controls=[0])
        full=prediction(piece,model);half=half_step_sensitivity(piece,model)
        self.assertLess(half['coordinate_rmse'],full['coordinate_rmse']*.6)


if __name__=='__main__':unittest.main()
