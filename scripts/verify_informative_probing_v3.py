"""Deterministic profiles, cached decoder, validation isolation; no Isaac episode."""
import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.probing_v3 import RuntimeV3
from interaction_runtime.probing_profiles_v3 import surrogate,information,speed_fraction
from interaction_runtime.policy import Policy
import verify_efficient_probing_v2 as previous
from update_probing_v3_model import update

def verify():
    previous.update=update;result=previous.verify()
    config=json.loads((ROOT/'configs/informative_probing_v3.json').read_text());r=object.__new__(RuntimeV3);r.policy=Policy.load(ROOT/'configs/interaction_runtime_v1.json');r.profile_started=0;r.dt=1/240
    for a in config['motion_menu'].values():
        r.probe_profile=a
        for tick in range(2400):
            r.tick=tick;m=r.motion_limits('LOADED');assert 0<m['speed_m_s']<=.012 and m['acceleration_m_s2']<=.04
        assert r.motion_limits('APPROACH')==r.policy.motion('APPROACH')
    # Archived measured robot transform, hinge prior only. No object dynamic truth.
    T=np.array([[-.0022103174,-.9999949506,-.0022832700,.3111125654],[-.0016988390,-.0022795173,.9999959589,-.3133443898],[-.9999961142,.0022141874,-.0016937920,.4307150398],[0,0,0,1.]])
    from scipy.spatial.transform import Rotation
    T[:3,:3]=Rotation.from_matrix(T[:3,:3]).as_matrix()
    axis=[2.579933e-7,8.977e-9,1.];axis=np.array(axis)/np.linalg.norm(axis);origin=[.7339114308,-.2777596443,.7459784718]
    outcomes=[]
    for lag in [0.,.05,.15]:
        chunks=[];q=0
        for label,target in [('plateaus',12),('plateaus',3),('steady',15)]:
            a=dict(config['motion_menu'][label],target_deg=target);d=surrogate(T,axis,origin,a,start_deg=q,lag_s=lag);chunks.append(d.pop('X'));outcomes.append(dict(profile=label,start=q,target=target,lag_s=lag,**d));q=target
        outcomes.append(dict(lag_s=lag,combined_train_information=information(np.concatenate(chunks))))
    result.update(V3_profile_envelopes=True,surrogate=outcomes,physical_episodes=0)
    return result
if __name__=='__main__':
    out=Path(sys.argv[1]);out.parent.mkdir(parents=True,exist_ok=True);r=verify();out.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
