"""No Isaac: calibrated sign/frame and current-door view callback regression."""
import argparse, inspect, json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
import native_g1_hybrid_contact as hybrid


def verify():
    cases=[]
    gravity=np.array([0.,0.,-7.3715,.12,-.03,0.]);bias=np.array([.01,-.02,.03,.001,0.,-.002])
    contact=np.array([.1,-.2,.3,.01,-.02,.03])
    for xyz in [(0,0,0),(.3,-.7,.5),(-1.,.8,-.2)]:
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',xyz).as_matrix();T[:3,3]=[.7,-.2,.2]
        for wrench in [np.zeros(6),contact]:
            transmitted=wrench-gravity
            raw=np.r_[T[:3,:3].T@transmitted[:3],T[:3,:3].T@transmitted[3:]]+bias
            measured=hybrid.guard_wrench_world(dict(T_world_sensor=T,raw_wrench_sensor=raw,tool_gravity_wrench_world=gravity),bias)
            np.testing.assert_allclose(measured,wrench,atol=1e-12)
            cases.append(dict(rpy=list(xyz),known_test_wrench=wrench.tolist(),recovered=measured.tolist()))
    old=np.eye(4);current=np.eye(4);current[:3,:3]=Rotation.from_euler('z',10,degrees=True).as_matrix();current[0,3]=.02
    checker=lambda *args:(True,'unused offline spy')
    state=dict(collision=SimpleNamespace(check=checker,moving_reference=old),moving=lambda:current)
    def spy(g,target):
        np.testing.assert_array_equal(g['collision'].moving_reference,current)
        assert g['collision'].check is checker
        return dict(current_door_reference=True)
    with patch.object(hybrid.observation,'evaluate_view',spy):result=hybrid.evaluate_current_view(state,{})
    np.testing.assert_array_equal(state['collision'].moving_reference,old)
    owner=inspect.getsource(hybrid.run)
    for callback in ['panel_plan','push','fresh_reobserve','initial_observation','evaluate_current_view']:
        assert callback in owner and callable(getattr(hybrid,callback))
    compile(owner,'explicit_hybrid_owner_preflight','exec')
    return dict(Isaac_actors=0,physical_attempts=0,wrench_sign_frame_tests=cases,current_door_view_callback=result,
        original_reference_not_mutated=True,explicit_owner_compiles=True,
        does_not_certify_physical_contact=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path);a=p.parse_args();result=verify()
    if a.output:a.output.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
