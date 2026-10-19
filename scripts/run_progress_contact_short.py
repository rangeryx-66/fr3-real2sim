"""One deterministic contact-fixture regression, not a fresh Agent experiment."""
import sys,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v3 import RuntimeV3
from interaction_runtime.estimator_v31 import OperationsV31


def run(r):
    fixture=ROOT/'results/fast_runtime_v1_20261018/contact_fixture';audit=json.loads((fixture/'FIXTURE_RESULT.json').read_text())
    if not audit['ready']:return dict(status='FIXTURE_UNAVAILABLE')
    shutil.copytree(audit['geometry'],r.output/'agent_geometry');shutil.copy2(fixture/'visual_plan.json',r.output/'visual_plan.json')
    ops=OperationsV31(r);g=r.g;view=ROOT/'results/agent_pull_push_v12_20261018/episode/rgbd/generation_00_agent_01'
    g['agent_observations']=[dict(folder=str(view),accepted_for_geometry=True,source='existing automatically ranked measured-RGBD engineering fixture; not fresh perception')]
    actions=[dict(role=role,target_deg=target,profile='constant',velocity_program='constant',reference_speed_m_s=speed,reference_acceleration_m_s2=.04,max_sim_s=8.,fit_intervals_s=([[0.,8.]] if role.startswith('FIT') else []),hold=True)
        for role,target,speed in [('FIT_SHORT_OPEN',.8,.004),('FIT_SHORT_CLOSE',.3,.003),('VALIDATION_SHORT_REOPEN',1.1,.0035)]]
    try:
        check=ops.validate_execution()
        if not check['valid']:return dict(status='FIXTURE_ROUTE_UNAVAILABLE',check=check)
        result=ops.attempt(dict(operation='attempt_grasp_and_probe',segments=actions))
        return dict(status='SHORT_CONTACT_COMPLETE' if len(result['segments'])==3 and all(x['ok'] for x in result['segments']) and result['safe_exit']['safe'] else 'SHORT_CONTACT_STOP',attempt=result,declared_actions=actions,scope='CONTROL_ENGINEERING_FIXTURE; not new Agent/perception/identification accuracy validation')
    finally:ops.close_workers()

if __name__=='__main__':sys.exit(main(RuntimeV3,run,extra_sources=('scripts/run_progress_contact_short.py','scripts/update_probing_v31_model.py','scripts/xarm_ft_v31_estimator.py')))
