"""No Isaac: initial visual eligibility, current assembly/view route, handover replay."""
import inspect,json
from pathlib import Path
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/xarm_cabinet_torque_v11_20261017'


def main():
    job=json.loads((OUT/'job.json').read_text())
    from xarm_wrist_ft import prepare
    config=prepare(OUT/'offline_sensor')
    import xarm7_native_gripper as hand
    with patch.object(hand,'configuration',lambda:config):
        import xarm7_native_planning as planning
        with patch.object(planning,'configuration',lambda:config):
            source=json.loads((Path(job['source'])/'report.json').read_text());model=planning.NativeModel(None,Path(job['asset_root']),source)
    from cross_robot_adapter import RobotPinkIK
    from native_g1_wrist_views import calibration,survey_candidates,select_route
    export=json.loads(Path(job['collision_export']).read_text());selection=job['grasp_geometry_candidates'][0]
    allowed=[s['path'] for s in export['shapes'] if selection['mesh'].replace('-','') in s['path'].replace('-','')]
    scene=planning.scene_factory(ROOT,export,model,allowed);solver=RobotPinkIK(config);c,X=calibration(job['wrist_calibration'])
    base=source['robot_base_pose'];home=np.asarray(job['robot_start_q'])
    route,audit=select_route(model,solver,scene,base,home,[next(survey_candidates(model,c))],c,X)
    from verify_large_range_torque import verify
    handover=verify(ROOT/'results/xarm_hybrid_v0_20261017/episode',Path(job['pink_runner']))
    import native_g1_cabinet_torque_contact as contact
    sample=inspect.getsource(contact.handle_attempt)
    assert 'safe_release_retreat=hybrid.safe_exit' in sample and "'ok,status,_=pull(g)'" in sample
    import native_g1_contact_recovery as recovery
    namespace=dict(recovery.__dict__,pull=contact.pull,safe_release_retreat=contact.hybrid.safe_exit)
    text=inspect.getsource(recovery.attempt_grasp_and_probe)
    text=text.replace("    if any(r['short_success'] for r in g['contact_attempts']):return dict(executed=False,reason='first success: recovery prohibited by registration')\n",'').replace("ok,status,_=namespace['open_with_contact_feedback'](g)",'ok,status,_=pull(g)')
    exec(compile(text,'offline_callback_check','exec'),namespace)
    from cabinet_torque_v11 import fit
    from xarm_ft_v1_estimator import features
    t=np.linspace(0,20,1000);q=.2+.1*np.sin(t);v=.1*np.cos(t);acc=-.1*np.sin(t)
    samples=[dict(q=z,qd=w,qdd=a,direction=np.sign(w),tau=0.) for z,w,a in zip(q,v,acc)];truth=np.array([.3,.2,.025])
    for s,y in zip(samples,features(samples,'constant')@truth):s['tau']=y
    fitted=fit(samples,'constant');np.testing.assert_allclose(fitted['parameters'],truth,atol=1e-4)
    result=dict(initial_view_valid=route is not None,initial_route=route,audit=audit,registered_station=base,initial_q=home.tolist(),
        handle_eligibility='archived visual detection/grasp/recovery/parking only; no saved view or grasp loaded',
        collision_source='original closed native scene representation; complete actual state checked online',
        initial_view_selection='unchanged coarse cabinet survey initializer',handover=handover,callback_compiled=True,
        synthetic_model_test_not_physics=fitted,loaded_large_range_not_certified=True,physical_actors=0)
    (OUT/'OFFLINE_PREFLIGHT.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(initial_view_valid=route is not None,reason=audit[-1]['reason'],handover_mm_s=handover['new_first_step'])))

if __name__=='__main__':main()
