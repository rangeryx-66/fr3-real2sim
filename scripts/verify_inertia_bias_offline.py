"""Analytic wrench/filter/integration consistency checks; no Isaac dependency."""
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from diagnose_inertia_bias_v3 import synthetic,sg2_kernel
from xarm_wrist_ft import wrench_at,generalized_effort


def verify():
    checks={};rng=np.random.default_rng(12);errors=[]
    for _ in range(30):
        T=np.eye(4);T[:3,:3]=Rotation.random(random_state=rng).as_matrix();T[:3,3]=rng.normal(size=3)
        contact=rng.normal(size=3);F=rng.normal(size=3);M=rng.normal(size=3);origin=rng.normal(size=3);axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
        local=np.r_[T[:3,:3].T@F,T[:3,:3].T@(M+np.cross(contact-T[:3,3],F))]
        expected=np.r_[F,M+np.cross(contact-origin,F)];errors.append(np.max(abs(wrench_at(local,T,origin)-expected)))
        value=generalized_effort(local,T,dict(axis_world=axis,origin_world_m=origin,joint_type='revolute'))
        errors.append(abs(value-axis@expected[3:]))
    assert max(errors)<1e-12
    checks['wrench_rotate_shift_project_max_error']=max(errors)
    values=synthetic();matched=[v for v in values if v['torque_filter']=='SG2_equivalent']
    checks['matched_analytic_max_relative_I_error']=max(abs(v['relative_I_error']) for v in matched)
    assert checks['matched_analytic_max_relative_I_error']<.001
    checks['kernels']={}
    for w in [61,121,181]:
        k=sg2_kernel(w,1/240);err=float(max(abs(k-k[::-1])));assert err<1e-10
        assert abs(k.sum()-1)<1e-12
        checks['kernels'][w]=dict(symmetry_error=err,unit_DC_gain=float(k.sum()))
    dt=1/240;t=np.arange(0,12,dt);q=.02*t+.003*np.sin(np.pi*t);v=.02+.003*np.pi*np.cos(np.pi*t);acc=-.003*np.pi**2*np.sin(np.pi*t)
    beta=np.array([.3,.4,.02]);J=.15;raw=(beta[0]+J)*acc+beta[1]*v+beta[2]*np.sign(v);H=J*v
    u=np.linspace(-1,1,121);w=np.pad((1-u*u)**3,(2,2));w/=w.sum();first=(np.roll(w,-1)-np.roll(w,1))/(2*dt);second=(np.roll(w,-1)-2*w+np.roll(w,1))/dt**2
    X=[];y=[];half=len(w)//2
    for c in range(181,len(q)-181,24):
        s=slice(c-half,c+half+1);X.append([second@q[s],-first@q[s],w@np.sign(v[s])]);y.append(w@raw[s]+first@H[s])
    fitted=np.linalg.lstsq(X,y,rcond=None)[0];assert np.max(abs((fitted-beta)/beta))<.001
    checks['weak_analytic_known_tool_momentum']=dict(known_parameters=beta.tolist(),recovered=fitted.tolist(),tool_hinge_inertia=J)
    checks.update(status='PASS: numerical identities only, not physical validation',Isaac_started=False)
    return checks


if __name__=='__main__':print(json.dumps(verify(),indent=2))
