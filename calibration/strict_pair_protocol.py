"""Frozen strict capture acceptance, independent of estimator and object class."""
import numpy as np
from quasistatic_velocity_gate import configuration_velocity
POLICY={'q_pair_max_rad':.0015,'q_ref_max_rad':.003,'dq_max_rad_s':.002,'opening_max_m':.0002,'relative_translation_max_m':.003,'relative_rotation_max_rad':np.deg2rad(5),'settled_window_s':.5,'min_samples':96,'attempts_per_pose':3,'min_clean_poses':4,'settle_s':[1.,2.,3.],'torque_half_window_max_Nm':.05}

def window_quality(d,q_ref,opening_ref,*,payload):
 from scipy.spatial.transform import Rotation
 n=len(d['t']);keep=np.asarray(d['t'])>=d['t'][-1]-POLICY['settled_window_s'];q=np.asarray(d['q'])[keep];dq=np.asarray(d['dq'])[keep];tau=np.asarray(d['tau'])[keep];w=np.asarray(d['actual_opening_m'])[keep];t=np.asarray(d['t'])[keep]
 reasons=[]
 if len(t)<POLICY['min_samples']:reasons.append('SHORT_WINDOW')
 qerr=float(np.max(np.abs(q-np.asarray(q_ref))));raw_v=float(np.max(np.abs(dq)));
 try:
  dq_gate=configuration_velocity(t,q);v=float(np.max(np.abs(dq_gate)))
 except ValueError as exc:
  reasons.append(str(exc));dq_gate=np.full((1,7),np.nan);v=float('inf')
 openerr=float(np.max(np.abs(w-opening_ref)));half=len(tau)//2
 torque_drift=float(np.max(np.abs(np.mean(tau[:half],0)-np.mean(tau[half:],0)))) if half else float('inf')
 if qerr>=POLICY['q_ref_max_rad']:reasons.append('Q_REF_MISMATCH')
 if v>=POLICY['dq_max_rad_s']:reasons.append('NONZERO_DQ')
 if openerr>=POLICY['opening_max_m']:reasons.append('OPENING_MISMATCH')
 if torque_drift>=POLICY['torque_half_window_max_Nm']:reasons.append('TORQUE_TRANSIENT')
 if not np.isfinite(q).all() or not np.isfinite(dq).all() or not np.isfinite(tau).all():reasons.append('NONFINITE')
 drift=angle=0.
 if payload:
  T=np.asarray(d['T_TCP_object'])[keep];relative=np.linalg.inv(T[0])[None]@T;drift=float(np.linalg.norm(relative[:,:3,3],axis=1).max());angle=float(np.max(Rotation.from_matrix(relative[:,:3,:3]).magnitude()))
  if drift>POLICY['relative_translation_max_m'] or angle>POLICY['relative_rotation_max_rad'] or np.any(d['relative_motion_violation'][keep]):reasons.append('RELATIVE_SLIP')
 if not bool(np.asarray(d['guard_passed']).all()):reasons.append('CONTACT_LOSS')
 slope=(q[-1]-q[0])/max(float(t[-1]-t[0]),1e-9)
 return {'accepted':not reasons,'reasons':reasons,'samples':len(t),'q_mean':q.mean(0).tolist(),'q_ref_max_error_rad':qerr,'dq_mean':dq.mean(0).tolist(),'dq_max_rad_s':v,'dq_gate_source':'ACTUAL_Q_TIMED_FINITE_DIFFERENCE','dq_gate_mean':dq_gate.mean(0).tolist(),'dq_reported_max_rad_s':raw_v,'q_endpoint_velocity':slope.tolist(),'dq_vs_q_slope_max':float(np.max(np.abs(dq.mean(0)-slope))),'tau_mean':tau.mean(0).tolist(),'tau_std':tau.std(0).tolist(),'torque_half_window_difference_Nm':torque_drift,'opening_mean_m':float(w.mean()),'opening_max_error_m':openerr,'relative_translation_m':drift,'relative_rotation_rad':angle}
