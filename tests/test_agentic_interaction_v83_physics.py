import json
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agentic_interaction_v83.physics_id import fit_only,trace,register,write_new,evaluate
from xarm_ft_v7_estimator import prediction,samples


class FrozenPhysicsTests(unittest.TestCase):
    def piece(self):
        n=1200;t=np.arange(n)/240;v=np.full(n,-.01);q=1.5-.01*t
        return dict(t=t,q=q,qd=v,qdd=np.zeros(n),native_qd=v,direction=-np.ones(n),torque=-np.ones(n)*.025,valid=np.ones(n,bool),segments=np.repeat('VALIDATION_NEW',n),block_id='VALIDATION_NEW_01#0',processing=dict(boundary_exclusion_samples=121,window=121))

    def model(self,n):
        return dict(parameters=[.13,.3]+([.022]*11 if n else [.022]),domain_native=[0,1.8],intervals=n,active_controls=list(range(11))if n else[0],support_intervals_native=[[0,1.8]],units=dict(coordinate='rad'))

    def test_exact_production_prediction_metrics_constant_and_position(self):
        for n in [0,8]:
            d=self.piece();samples(d,[0,1.8],[8]);m=self.model(n);legacy=prediction(d,m);current=trace(d,m)
            self.assertAlmostEqual(float(np.sqrt(np.mean(current['q_error']**2))),legacy['coordinate_rmse'],places=13)
            self.assertAlmostEqual(current['q_error'][-1],legacy['final_coordinate_error'],places=13)

    def test_validation_data_cannot_enter_fit(self):
        with self.assertRaisesRegex(ValueError,'FIT_ONLY'):fit_only([self.piece()],{})

    def test_frozen_files_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory()as tmp:
            p=Path(tmp)/'model.json';write_new(p,dict(parameters=[1,2,3]))
            with self.assertRaises(FileExistsError):write_new(p,dict(parameters=[9,9,9]))
            self.assertEqual(json.loads(p.read_text())['parameters'],[1,2,3])

    def prepare(self,tmp):
        root=Path(tmp);episode=root/'fresh';(episode/'telemetry').mkdir(parents=True)
        (episode/'telemetry/index.json').write_text(json.dumps(dict(chunks=[dict(last_step=100)],segments=[])))
        (episode/'AGENT_SESSION_STATUS.json').write_text(json.dumps(dict(state='WAITING',unloaded=True,sequence=3)))
        model=dict(frozen_at_unix_s=time.time()-1,lineage=dict(training_episode_ids=['old'],development_episode_ids=['old']))
        files=[root/'candidate.json',root/'baseline.json']
        for f in files:f.write_text(json.dumps(model))
        return episode,files

    def test_register_requires_new_unexecuted_validation_at_safe_boundary(self):
        with tempfile.TemporaryDirectory()as tmp:
            episode,files=self.prepare(tmp);out=Path(tmp)/'registration.json'
            register(episode,*files,['VALIDATION_NEW'],out);self.assertEqual(json.loads(out.read_text())['prior_last_step'],100)
            (episode/'telemetry/index.json').write_text(json.dumps(dict(chunks=[],segments=[dict(role='VALIDATION_LATE')])) )
            with self.assertRaisesRegex(ValueError,'ALREADY_STARTED'):register(episode,*files,['VALIDATION_LATE'],Path(tmp)/'late.json')

    def test_register_rejects_reused_development_episode_or_loaded_robot(self):
        with tempfile.TemporaryDirectory()as tmp:
            episode,files=self.prepare(tmp)
            d=json.loads(files[0].read_text());d['lineage']['development_episode_ids']=[episode.name];files[0].write_text(json.dumps(d))
            with self.assertRaisesRegex(ValueError,'FRESH_EPISODE'):register(episode,*files,['VALIDATION_NEW'],Path(tmp)/'bad.json')
            (episode/'AGENT_SESSION_STATUS.json').write_text(json.dumps(dict(state='RUNNING',unloaded=False)))
            with self.assertRaisesRegex(ValueError,'UNLOADED'):register(episode,*files,['VALIDATION_NEW'],Path(tmp)/'loaded.json')

    def test_fresh_evaluation_never_refits_and_keeps_failed_validation_incomplete(self):
        with tempfile.TemporaryDirectory()as tmp:
            episode,files=self.prepare(tmp)
            for i,f in enumerate(files):
                payload=json.loads(f.read_text());payload['model']=self.model(8 if i==0 else 0);payload['model']['parameter_names']=['I','b','C'];f.write_text(json.dumps(payload))
            registration=Path(tmp)/'registered.json';register(episode,*files,['VALIDATION_NEW','VALIDATION_FAILED'],registration)
            (episode/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(actor_exited=True)))
            (episode/'telemetry/index.json').write_text(json.dumps(dict(segments=[dict(role='VALIDATION_NEW',first_step=101,result=dict(status='TARGET_REACHED',physical_transmission_valid=True)),dict(role='VALIDATION_FAILED',first_step=1401,result=dict(status='TRANSMISSION_FAILURE',physical_transmission_valid=False))])))
            d=self.piece();d['segments']=np.repeat('VALIDATION_NEW_01',len(d['t']))
            config=dict(domain_native=[0,1.8],intervals=[8]);raw=dict(t=d['t'],relative_slip_m=np.zeros(len(d['t'])),segment=np.zeros(len(d['t'])))
            with patch('agentic_interaction_v83.physics_id.fit',side_effect=AssertionError('validation attempted refit')),patch('agentic_interaction_v83.physics_id.load_segments',return_value=([d],dict(estimation=config,prior={}),[])),patch('diagnose_probing_v3.columns',return_value=(raw,{})):
                output=Path(tmp)/'evaluation.json';evaluate(episode,*files,registration,output)
            result=json.loads(output.read_text());self.assertFalse(result['all_declared_validation_completed']);self.assertFalse(result['refitting_performed']);self.assertTrue(result['independent_validation'])
            self.assertEqual(result['models']['candidate']['aggregate']['all']['samples'],958)


if __name__=='__main__':unittest.main()
