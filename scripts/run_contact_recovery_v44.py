"""Short engineering chain: archived failure fixture -> fresh Agent recovery.

No blind research episode. First fixture acquisition reproduces transmission;
only the subsequent fresh measured-geometry recovery can establish transfer.
"""
import sys,json,shutil,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v3 import RuntimeV3
from interaction_runtime.probing_v3 import RecorderV3
from interaction_runtime.probing_v2 import ProbingRuntime
from interaction_runtime.recovery_v44 import OperationsV44
from interaction_runtime.probing_v2 import serve
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet

class ContactRecorder(RecorderV3):
    """Bounded passive contact checkpoints; essential numbers remain 240 Hz."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.contact_checkpoints=[];self.last_contact_time=-1.;self.last_contact_phase=None
    def append(self,row):
        super().append(row)
        contacts=[x for x in row['contacts'] if x['force_n']>0]
        if contacts and len(self.contact_checkpoints)<2000 and (row['t']-self.last_contact_time>=.1 or row['phase']!=self.last_contact_phase):
            self.contact_checkpoints.append(dict(t=row['t'],phase=row['phase'],q=row['q'].copy(),T_tcp=row['T_tcp'].copy(),T_moving=row['T_moving_link'].copy(),contacts=copy.deepcopy(contacts)))
            self.last_contact_time=row['t'];self.last_contact_phase=row['phase']
    def close(self):
        result=super().close();atomic_json(self.folder.parent/'CONTACT_CHECKPOINTS.json',result_packet(dict(sample_semantics='event/10Hz native point reports, not pressure area or full friction wrench; detection before integration',samples=self.contact_checkpoints)));return result

class ContactRuntime(RuntimeV3):
    def __init__(self,job):ProbingRuntime.__init__(self,job,record_factory=ContactRecorder)


def run(r):
    fixture=Path(r.job['engineering_fixture']);ops=OperationsV44(r);g=r.g
    shutil.copytree(fixture/'agent_geometry',r.output/'agent_geometry');shutil.copy2(fixture/'visual_plan.json',r.output/'visual_plan.json')
    info=json.loads((fixture/'FIXTURE.json').read_text());g['agent_observations']=[dict(folder=info['initial_observation'],accepted_for_geometry=True,source='ARCHIVED_ENGINEERING_FIXTURE')]
    action=dict(role='FIT_ENGINEERING_FAILURE_REPRODUCTION',target_deg=8.,profile='constant',velocity_program='plateaus',reference_speed_m_s=.012,reference_acceleration_m_s2=.04,low_speed_fraction=.125,ramp_s=.45,plateau_s=.55,fit_intervals_s=[[.55,60.]],hold=True)
    check=ops.validate_execution()
    if not check['valid']:ops.close_workers();return dict(status='FIXTURE_ROUTE_REJECTED',check=check)
    first=ops.attempt(dict(operation='attempt_grasp_and_probe',segments=[action]))
    atomic_json(r.output/'ENGINEERING_FIRST_RESULT.json',result_packet(first))
    if not first.get('safe_exit',{}).get('safe'):ops.close_workers();return dict(status='FIRST_EXIT_UNAVAILABLE',first=first)
    # Real new wrist images at the actual resulting door state. Same actor,
    # same operation owner and acquisition count; no object/joint reset.
    observation=ops.reobserve();atomic_json(r.output/'INITIAL_OBSERVATION_READY.json',result_packet(observation))
    result=serve(r,operations_factory=lambda runtime:ops,initialize=False)
    guards=None
    if g.get('at_safe_park') and r.supervisor.latched is None:
        from interaction_runtime.guard_regression import run as verify_guards
        guards=verify_guards(r)
    return dict(status='ENGINEERING_CHAIN_FINISHED' if result.get('status')=='SESSION_COMPLETE' else 'ENGINEERING_INCOMPLETE',scope='one failure-fixture acquisition plus at most one fresh Agent recovery; not research validation accuracy',first=first,Agent=result,guards=guards)

if __name__=='__main__':
    sys.exit(main(ContactRuntime,run,extra_sources=('scripts/run_contact_recovery_v44.py','scripts/native_g1_execution_candidates.py','scripts/native_g1_panel_support.py','scripts/native_g1_agent_execution.py','scripts/native_g1_hybrid_contact.py','scripts/hybrid_panel_preflight.py','scripts/native_g1_visual_input.py','scripts/plan_shared_door_skill.py','scripts/xarm7_native_planning.py','scripts/native_g1_wrist_input.py','scripts/update_probing_v31_model.py','scripts/xarm_ft_v31_estimator.py')))
