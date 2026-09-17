import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from real2sim.payload_id import identify, spatial_regressor
from real2sim.safety import UnsafePayload, require_free_space_stable


def test_free_space_gate():
    good={"success":True,"final_support":{"passed":True,"currently_clear":True,"category":"STABLE","flags":{"PICKED":True,"RETAINED":True,"CLEAR_TABLE":True}}}
    assert require_free_space_stable(good)["passed"]
    try:require_free_space_stable({"success":False})
    except UnsafePayload:pass
    else:raise AssertionError("unsafe grasp accepted")


def test_synthetic_payload_identification(tmp_path:Path):
    n=1000;t=np.arange(n)/240.;q=np.zeros((n,7));dq=np.zeros_like(q)
    T=np.repeat(np.eye(4)[None],n,axis=0)
    T[:,:3,3]=np.c_[.03*np.sin(t),.02*np.cos(.7*t),.4+.01*np.sin(1.3*t)]
    T[:,:3,:3]=Rotation.from_euler("xyz",np.c_[.2*np.sin(.9*t),.15*np.sin(1.1*t),.25*np.cos(.8*t)]).as_matrix()
    rng=np.random.default_rng(7);J0=rng.normal(size=(6,7));J1=rng.normal(size=(6,7))
    J=J0[None]+.1*np.sin(.4*t)[:,None,None]*J1[None];J[:,:3]*=.2;J[:,3:]*=.5
    common=dict(t=t,q=q,dq=dq,T_B_TCP=T,T_B_hand=T,jacobian_TCP=J,guard_passed=np.array([True]))
    base=tmp_path/'base.npz';payload=tmp_path/'payload.npz';np.savez(base,tau=np.zeros_like(q),mode=np.array(['baseline']),**common)
    record={**common,"tau":np.zeros_like(q)};Y=spatial_regressor(record)
    mass=.45;com=np.array([.01,-.008,.025]);Icom=np.diag([8e-4,7e-4,4e-4]);S=np.array([[0,-com[2],com[1]],[com[2],0,-com[0]],[-com[1],com[0],0]])
    Io=Icom+mass*S.T@S;theta=np.r_[mass,mass*com,Io[0,0],Io[0,1],Io[0,2],Io[1,1],Io[1,2],Io[2,2]]
    np.savez(payload,tau=(Y@theta).reshape(n,7),mode=np.array(['payload']),**common)
    result=identify(base,payload,tmp_path/'inertial.json')
    assert abs(result.mass-mass)<2e-3
    assert np.linalg.norm(np.asarray(result.center_of_mass)-com)<2e-3
