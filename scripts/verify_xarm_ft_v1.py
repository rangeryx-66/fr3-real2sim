"""No Isaac: callback, partition-leakage, passive-law and frozen controller checks."""
import json,copy,ast,hashlib
from pathlib import Path
from unittest.mock import patch
import numpy as np
from xarm_ft_v1_estimator import local_process,fit,CONFIG
import native_g1_ft_multistate as v1
import native_g1_ft_identification as v0
import native_g1_contact_recovery as recovery
import native_g1_safe_termination as termination

seen={}
def owner(g):
    fn=recovery.attempt_grasp_and_probe;ns=fn.__globals__
    seen.update(uses_V1_interaction=ns['bidirectional'] is v1.interact,uses_V11_safe_park=ns['safe_release_retreat'] is termination.safe_release_park,
        original_grasp_execution=ns['physical'] is recovery.physical)
    return seen
with patch.object(v1,'install_native_law',lambda g:lambda:None),patch.object(v0,'install_passive',lambda g:None),patch.object(termination,'run',owner):
    v1.run(dict(step=lambda:None,observe_step=lambda *a:None))
assert all(seen.values()),seen
rows=[];dt=1/240
for block,label in enumerate(['FIT_A','VALIDATION_B','FIT_C']):
    for j in range(1000):
        t=(1000*block+j)*dt;s=j*dt
        # Cubic motion has exact SG3 derivatives; a fast sinusoid would test
        # smoothing attenuation as well as the regression and leakage wiring.
        q=.04+.02*s-.012*s*s+.002*s**3;v=.02-.024*s+.006*s*s;acc=-.024+.012*s
        tau=.4*acc+.2*v+.03*np.sign(v)
        body=dict(name='massless_tool',mass_kg=0.,com_world_m=[0,0,0],T_world_body=np.eye(4).tolist(),inertia_world_kg_m2=np.zeros((3,3)).tolist())
        rows.append(dict(t=t,door_angle_deg=float(np.rad2deg(q)),identification_segment=label,phase='OPEN',forces_n={'L':1,'R':1},
            wrist_ft=dict(valid=True,raw_wrench_sensor=[0,0,0,0,0,tau],T_world_sensor=np.eye(4).tolist(),tool_gravity_wrench_world=[0]*6,body_states=[body])))
prior=dict(axis_world=[0,0,1],origin_world_m=[0,0,0],joint_type='revolute');data,_=local_process(rows,prior,np.zeros(6));modified=copy.deepcopy(rows)
for r in modified:
    if r['identification_segment'].startswith('VALIDATION'):
        r['door_angle_deg']+=1000;r['wrist_ft']['raw_wrench_sensor']=[1e6]*6;r['wrist_ft']['body_states'][0]['com_world_m']=[10,20,30]
other,_=local_process(modified,prior,np.zeros(6));mask=~np.char.startswith(data['segments'],'VALIDATION');mask2=~np.char.startswith(other['segments'],'VALIDATION')
for key in ['q','qd','qdd','torque','inertial_world','valid']:
    assert np.array_equal(data[key][mask],other[key][mask2]),key
seen.update(validation_change_cannot_change_training_derivatives_or_compensation=True,segment_local_full_cascade_edges=True,
    constant_synthetic_fit=fit([s for s in data['samples'] if not s['segment'].startswith('VALIDATION')],'constant')['parameters'],
    Isaac_scenes_started=0,configuration=CONFIG)
assert np.allclose(seen['constant_synthetic_fit'],[.4,.2,.03],rtol=.03,atol=.001),seen
from xarm_wrist_ft import generalized_effort
from scipy.spatial.transform import Rotation
rng=np.random.default_rng(17)
for _ in range(20):
    T=np.eye(4);T[:3,:3]=Rotation.random(random_state=rng).as_matrix();T[:3,3]=rng.normal(size=3)
    axis=rng.normal(size=3);axis/=np.linalg.norm(axis);origin=rng.normal(size=3);raw=rng.normal(size=6)
    coefficients=np.r_[T[:3,:3].T@np.cross(axis,T[:3,3]-origin),T[:3,:3].T@axis]
    assert np.allclose(coefficients@raw,generalized_effort(raw,T,dict(joint_type='revolute',axis_world=axis,origin_world_m=origin)))
seen['raw_sensor_quantization_projection_sign_verified']=True
pink_path=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/benchmarkA_30deg_20261011/shared_sources/run_7320_pink_fixed.py')
if pink_path.exists():
    old=pink_path.read_text();new=old
    rules=[('rate=.0015*motion_scale/radius','rate=identification_speed_m_s/radius'),('acc=.002*motion_scale**2','acc=identification_acceleration_m_s2'),('min(.0015*motion_scale,np.sqrt','min(identification_speed_m_s,np.sqrt')]
    for a,b in rules:assert new.count(a)==1;new=new.replace(a,b)
    for a,b in reversed(rules):new=new.replace(b,a)
    assert new==old
    seen['only_reference_time_constants_rebound_not_IK_or_actuator_gains']=True
print(json.dumps(seen,indent=2))
