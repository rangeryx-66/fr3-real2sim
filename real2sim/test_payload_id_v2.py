"""Pure numerical checks for the two-stage PayloadID module."""
from __future__ import annotations

import numpy as np

from .payload_id_v2 import robust_linear_fit, static_gravity_regressor, static_gravity_identify, dynamic_inertia_identify


def _record(n=180, dynamic=False):
    t=np.linspace(0,6,n); q=np.zeros((n,7)); dq=np.zeros_like(q); tau=np.zeros_like(q)
    T=np.repeat(np.eye(4)[None],n,axis=0); J=np.zeros((n,6,7))
    for i,ti in enumerate(t):
        r=.35*np.sin(2*np.pi*ti/6); p=.28*np.cos(2*np.pi*ti/6)
        from scipy.spatial.transform import Rotation
        T[i,:3,:3]=Rotation.from_euler('xy',[r,p]).as_matrix();
        J[i,:3,:7]=np.array([[.2,.1,0,.1,0,.05,0],[0,.2,.1,0,.1,0,.05],[.05,0,.2,.1,.1,.05,.1]])
        J[i,3:,:7]=np.array([[.1,.2,0,.1,.05,0,.1],[0,.1,.2,.05,.1,.1,0],[.2,0,.1,.1,0,.1,.05]])
    rec={'t':t,'q':q,'dq':dq,'tau':tau,'T_B_TCP':T,'jacobian_TCP':J,'guard_passed':np.array([True])}
    return rec


def test_robust_fit_rejects_outlier():
    x=np.array([[1.,0.],[0.,1.],[1.,1.],[2.,-1.]])
    y=x@np.array([2.,-1.]); y[-1]+=50
    result=robust_linear_fit(x,y)
    assert np.allclose(result['theta'],[2.,-1.],atol=.4)
    assert result['outlier_fraction']>0


def test_static_gravity_identification_synthetic():
    n=180; base=_record(n); payload=_record(n); Y=static_gravity_regressor(payload).reshape(n,7,4)
    theta=np.array([.45,.012,-.008,.006]); payload['tau']=(Y@theta).reshape(n,7); base['tau']=np.zeros((n,7))
    payload['static_hold']=np.ones(n,dtype=bool); payload['static_pose_id']=np.repeat(np.arange(9),20)[:n]
    out=static_gravity_identify(base,payload,min_postures=5)
    assert out['mass_kg']>0 and np.allclose(out['mass_kg'],theta[0],atol=.03)


def test_dynamic_rejects_missing_references():
    base=_record(); payload=_record(); payload['guard_passed']=np.array([False])
    out=dynamic_inertia_identify(base,payload,{'mass_kg':.4,'center_of_mass_m':[0,0,0]})
    assert out['accepted'] is False and out['source']=='UNOBSERVABLE'

