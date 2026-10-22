"""No loaded waiting and no validation admission from stale/one-way FIT."""
import hashlib,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from agentic_interaction_v81.operations import operations_class


class ReadinessTests(unittest.TestCase):
    def exercise(self,change=None,alter_model=False):
        class Owner:
            def interact(self,*args):return dict(executed=True,original_owner_called=True)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder);m=p/'model';m.mkdir()
            payload=json.dumps(dict(selected=dict(parameters=[1.,2.,.3]),configuration={})).encode()
            (m/'estimate.json').write_bytes(payload)
            ready=dict(model_sha256=hashlib.sha256(payload).hexdigest(),episode_id='new_scene',
                latest_completed_FIT_id=2,qualified_bidirectional=True,positive_bins=20,negative_bins=20,
                distinct_eligible_trajectories=2)
            ready.update(change or {});(m/'FIT_READINESS_V81.json').write_text(json.dumps(ready))
            if alter_model:(m/'estimate.json').write_text('{}')
            ops=operations_class(Owner)();ops.actions=[dict(role='VALIDATION_NEW',target_q=.03)]
            ops.r=SimpleNamespace(output=p,job=dict(episode_id='new_scene'),actual=lambda:.04,
                record=SimpleNamespace(segments=[dict(id=1,role='FIT_A',completed=True),dict(id=2,role='FIT_B',completed=True)]))
            result=ops.interact(.03)
            return result,list((p/'validation_model_seals').glob('*.json'))

    def test_same_scene_current_bidirectional_fit_allows_original_owner(self):
        result,seals=self.exercise();self.assertTrue(result['original_owner_called']);self.assertEqual(len(seals),1)

    def test_one_way_fit_defers_without_executing_or_waiting(self):
        result,seals=self.exercise(dict(negative_bins=0,qualified_bidirectional=False))
        self.assertEqual(result['status'],'VALIDATION_DEFERRED');self.assertFalse(result['executed']);self.assertEqual(seals,[])

    def test_stale_worker_result_not_used(self):
        for change in [dict(latest_completed_FIT_id=1),dict(episode_id='old_scene'),dict(distinct_eligible_trajectories=1)]:
            result,seals=self.exercise(change)
            self.assertEqual(result['status'],'VALIDATION_DEFERRED');self.assertEqual(seals,[])

    def test_producer_race_fails_closed_without_a_loaded_wait(self):
        result,seals=self.exercise(alter_model=True)
        self.assertFalse(result['executed']);self.assertEqual(seals,[])


if __name__=='__main__':unittest.main()
