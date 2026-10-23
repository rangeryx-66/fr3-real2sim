import json,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
from agentic_interaction_v81.debug_budget import apply,window,VALUES,PROFILE,FULL_RANGE_PROFILE
from agentic_interaction_v81.transport import publish,ProtocolError
from agentic_interaction_v81.session_status import write
from interaction_runtime.segments import EpisodeBudget


class DebugBudgetTests(unittest.TestCase):
    def spec(self):return dict(phase='engineering',debug_budget_profile=PROFILE,
        wall_budget_s=3600,sim_budget_s=1800,agent_wait_s=600)

    def test_exact_debug_values_and_finite_work_exit_separation(self):
        job={};apply(self.spec(),job)
        for k,v in VALUES.items():self.assertEqual(job[k],v)
        self.assertIn('deadline_shanghai',job)
        # Test elapsed accounting without the real absolute wall clock.
        job.pop('deadline_shanghai');now=[0.];b=EpisodeBudget(job,now=lambda:now[0])
        r=SimpleNamespace(job=job,episode_budget=b,tick=0,dt=1/240)
        self.assertEqual(window(r),(600.,dict(wall_s=3000.,sim_s=1680.,exit_reserved=True)))
        now[0]=2700;self.assertEqual(window(r)[0],300.)
        now[0]=3000;self.assertEqual(window(r)[0],0.)

    def test_no_silent_change_to_other_jobs_or_conflicting_debug_spec(self):
        job={'x':1};apply({},job);self.assertEqual(job,{'x':1})
        for change in [dict(wall_budget_s=1800),dict(agent_wait_s=180),dict(phase='independent_repetition')]:
            with self.subTest(change=change),self.assertRaises(ValueError):apply(dict(self.spec(),**change),{})

    def test_full_range_profile_has_separate_resources_not_new_physical_limits(self):
        job={'safety_policy':'frozen','controller':'frozen'}
        spec=dict(self.spec(),debug_budget_profile=FULL_RANGE_PROFILE,wall_budget_s=5400,sim_budget_s=2700)
        apply(spec,job);job.pop('deadline_shanghai')
        b=EpisodeBudget(job,now=lambda:0)
        self.assertEqual(b.remaining(0),dict(wall_s=4800.,sim_s=2580.,exit_reserved=True))
        self.assertEqual(job['agent_wait_timeout_s'],600.)
        self.assertEqual(job['safety_policy'],'frozen');self.assertEqual(job['controller'],'frozen')
        with self.assertRaises(ValueError):apply(dict(spec,sim_budget_s=1800),{})

    def test_historical_waiting_cannot_publish_while_running_or_closed(self):
        with tempfile.TemporaryDirectory()as d:
            root=Path(d);job=dict(episode_id='test',**VALUES)
            r=SimpleNamespace(job=job,output=root,base_generation=1,g={'observation_generations':[1,2]},
                tick=0,dt=1/240,episode_budget=EpisodeBudget(job))
            (root/'WAITING_FOR_AGENT.json').write_text(json.dumps(dict(sequence=3,timeout_s=600,deadline_unix_s=time.time()+600)))
            for state in ['RUNNING','COMPLETED','FINALIZING','CLOSED']:
                write(r,3,state)
                with self.subTest(state=state),self.assertRaisesRegex(ProtocolError,'BUSY_OR_CLOSED'):
                    publish(root/'agent_commands',3,dict(operation='inspect'))
            value=write(r,3,'WAITING',unloaded=True,deadline_unix_s=time.time()+600)
            self.assertEqual(value['observation_generation'],2)
            result=publish(root/'agent_commands',3,dict(operation='inspect'))
            self.assertEqual(result['acknowledgment'],'pending')


if __name__=='__main__':unittest.main()
