"""Bounded native-contact engineering regression; no Agent research episode.

Uses an explicitly archived automatically generated RGB-D fixture, not a new
perception result. One acquired grasp tests slow travel, interruption, reversal,
and contact degradation in sequence. Every outcome is retained.
"""
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
    g['agent_observations']=[dict(folder=str(view),accepted_for_geometry=True,source='automatically ranked measured RGB-D engineering fixture, not fresh Agent perception')]
    def action(role,target,speed,**extra):return dict(role=role,target_deg=target,profile='constant',velocity_program='constant',reference_speed_m_s=speed,reference_acceleration_m_s2=.04,hold=True,**extra)
    normal=bool(r.job.get('engineering_normal_profile',False))
    actions=[action('FIT_NATIVE_NORMAL_OPEN' if normal else 'FIT_HEALTHY_BEYOND_45',.8,.004 if normal else .0001,fit_intervals_s=[[0.,80.]]),
        action('FIT_DECLARED_ENGINEERING_INTERRUPT',1.8,.003,max_sim_s=2.2,fit_intervals_s=[[.1,2.2]]),
        action('VALIDATION_COMPLETE_REVERSAL',.3,.0035,continue_after=['SAFE_PARTIAL_PROGRESS']),
        action('FIT_NATIVE_TRANSMISSION_PROBE',13.,.012,fit_intervals_s=[[.6,40.]])]
    try:
        check=ops.validate_execution()
        if not check['valid']:return dict(status='FIXTURE_ROUTE_UNAVAILABLE',check=check)
        result=ops.attempt(dict(operation='attempt_grasp_and_probe',segments=actions))
        guards=None
        if normal and result.get('safe_exit',{}).get('safe'):
            from interaction_runtime.guard_regression import run as verify_guards
            guards=verify_guards(r)
        return dict(status='ENGINEERING_COMPLETE',guards=guards,attempt=result,declared_actions=actions,scope='NATIVE_CONTACT_CONTROL_ENGINEERING; not new perception or research identification accuracy validation')
    finally:ops.close_workers()

if __name__=='__main__':sys.exit(main(RuntimeV3,run,extra_sources=('scripts/run_adaptive_contact_v42.py','scripts/update_probing_v31_model.py','scripts/xarm_ft_v31_estimator.py','scripts/native_g1_wrist_input.py','scripts/agent_command_protocol_v12.py')))
