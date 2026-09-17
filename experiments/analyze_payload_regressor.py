import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import sys
sys.path.insert(0,'/data1/home/rangeryx/fr3_moveit_grasp')
sys.path.insert(0,'/data1/home/rangeryx/fr3_moveit_grasp/src')
from real2sim.fr3_robot_calibration import filter_torque_official
from real2sim.fr3_robot_calibration import _official_regressor
from real2sim.fr3_robot_calibration import make_fr3_arm_urdf, _make_drake_plant, DEFAULT_FR3_URDF
from real2sim.payload_id_v2 import spatial_regressor

root=Path('/data1/home/rangeryx/fr3_moveit_grasp/results/fr3_official_payloadid_v2')
base=root/'paired_baselines/soup/attempt_00_seed1030/system_id_baseline_dynamic.npz'
pay=root/'payload_trials/soup/attempt_00_seed1030/system_id_payload_dynamic.npz'
for p in [base,pay]: print(p, p.exists())
b=np.load(base,allow_pickle=True); p=np.load(pay,allow_pickle=True)
print('keys',b.files); print('shapes', {k:b[k].shape for k in b.files}); print('payload guard',p['guard_passed'], 'attachment',p['attachment_mode'])
for k in ['q_ref','dq_ref','ddq_ref','q','dq','tau','T_B_TCP','jacobian_TCP']:
    print(k, b[k].shape, p[k].shape, np.max(np.abs(b[k][:min(len(b[k]),len(p[k]))]-p[k][:min(len(b[k]),len(p[k]))])) if b[k].ndim==p[k].ndim and b[k].shape[1:]==p[k].shape[1:] else '')
print('T tcp first',p['T_B_TCP'][0]); print('J first',p['jacobian_TCP'][0])
qref=p['q_ref']; print('qref range',qref.min(0),qref.max(0), 'dq',p['dq_ref'].min(0),p['dq_ref'].max(0),'ddq',p['ddq_ref'].min(0),p['ddq_ref'].max(0))

official=Path('/data1/home/rangeryx/fr3_moveit_grasp/third_party/scalable_real2sim_robot_payload_id_upstream_c52e31c')
tb=np.asarray(b['t']); tp=np.asarray(p['t'])
fb,_=filter_torque_official({k:b[k] for k in b.files if hasattr(b[k],'shape')},official)
fp,_=filter_torque_official({k:p[k] for k in p.files if hasattr(p[k],'shape')},official)
fbp=np.column_stack([np.interp(tp,tb,fb[:,j]) for j in range(7)])
delta=fp-fbp
n=len(tp)
rec={k:p[k] for k in ['t','T_B_TCP','jacobian_TCP','q_ref','dq_ref','ddq_ref']}
Y=spatial_regressor(rec).reshape(n*7,10)[:,:4]
for name, yy in [('filtered',delta.reshape(-1)),('raw',(p['tau']-np.column_stack([np.interp(tp,tb,b['tau'][:,j]) for j in range(7)])).reshape(-1))]:
    for s in [1.,-1.]:
        th=np.linalg.lstsq(Y,s*yy,rcond=1e-10)[0]
        com=th[1:]/th[0] if th[0]>0 else np.full(3,np.nan)
        print('FIT',name,'sign',s,'theta',th,'com',com,'rms',np.sqrt(np.mean((Y@th-s*yy)**2)))
true=np.array([-0.008427580168585969,-0.003606015530720242,0.03744343934298978])
print('true',true)
th=np.r_[0.45,0.45*true]
pred=(Y@th).reshape(n,7)
print('predtrue-vs-delta', 'corr',np.corrcoef(pred.reshape(-1),delta.reshape(-1))[0,1], 'rms_true',np.sqrt(np.mean((pred-delta)**2)), 'rms_neg',np.sqrt(np.mean((pred+delta)**2)), 'scale_delta',np.sqrt(np.mean(delta**2)))

# Jacobian convention audit against finite-difference measured TCP motion.
T=np.asarray(p['T_B_TCP']); J=np.asarray(p['jacobian_TCP']); dt=np.median(np.diff(tp))
vel_fd=np.gradient(T[:,:3,3],dt,axis=0)
om_fd=np.zeros((n,3))
for i in range(1,n-1):
    om_fd[i]=Rotation.from_matrix(T[i-1,:3,:3].T@T[i+1,:3,:3]).as_rotvec()/(2*dt)
om_fd[0]=om_fd[1]; om_fd[-1]=om_fd[-2]
jdq=np.einsum('nij,nj->ni',J,p['dq'])
print('J convention rms lin/angular',np.sqrt(np.mean((jdq[:,:3]-vel_fd)**2)),np.sqrt(np.mean((jdq[:,3:]-om_fd)**2)))
print('J swapped rms',np.sqrt(np.mean((jdq[:,:3]-om_fd)**2)),np.sqrt(np.mean((jdq[:,3:]-vel_fd)**2)))

# Compare Isaac Jacobian against the same FR3 arm model used by the official
# Drake regressor, evaluated at the analytic q_ref path.
try:
    from pydrake.all import JacobianWrtVariable
    du=make_fr3_arm_urdf(DEFAULT_FR3_URDF,Path('/tmp/fr3_payload_id_models/analyze_fr3.urdf'))
    dp,dm=_make_drake_plant(du); dc=dp.CreateDefaultContext()
    wf=dp.world_frame(); tf=dp.GetFrameByName('fr3_hand_tcp')
    Jd=[]
    for qi in p['q_ref']:
        dp.SetPositions(dc,qi)
        jl=dp.CalcJacobianTranslationalVelocity(dc,JacobianWrtVariable.kV,tf,np.zeros(3),wf,wf)
        ja=dp.CalcJacobianAngularVelocity(dc,JacobianWrtVariable.kV,tf,wf,wf)
        Jd.append(np.vstack([jl,ja]))
    Jd=np.asarray(Jd)
    print('Drake-vs-Isaac J rms',np.sqrt(np.mean((Jd-J)**2)),'trans',np.sqrt(np.mean((Jd[:,:3]-J[:,:3])**2)),'ang',np.sqrt(np.mean((Jd[:,3:]-J[:,3:])**2)))
    print('Jd first',Jd[0])
    Pd=[]; Rd=[]
    for qi in p['q_ref']:
        dp.SetPositions(dc,qi)
        X=dp.CalcRelativeTransform(dc,wf,tf)
        Pd.append(X.translation()); Rd.append(X.rotation().matrix())
    Pd=np.asarray(Pd); Rd=np.asarray(Rd)
    print('Drake-vs-Isaac pose rms',np.sqrt(np.mean((Pd-T[:,:3,3])**2)),np.sqrt(np.mean((Rd-T[:,:3,:3])**2)))
    print('P first',Pd[0],T[0,:3,3])
    recd=dict(rec); recd['jacobian_TCP']=Jd
    Yd=spatial_regressor(recd).reshape(n*7,10)[:,:4]
    for name, yy in [('filtered',delta.reshape(-1))]:
        th=np.linalg.lstsq(Yd,yy,rcond=1e-10)[0]
        print('FIT_DRAKE_J',name,'theta',th,'com',th[1:]/th[0],'rms',np.sqrt(np.mean((Yd@th-yy)**2)))
except Exception as exc:
    print('DRAKE_J_ERROR',repr(exc))

# Use the fitted official robot-alone parameters to predict the unloaded
# robot torque at the payload's exact analytic q_ref path.  This is the
# official payload-only subtraction contract and removes load-dependent
# tracking lag from a direct time-series difference.
fitj=json.load(open(str(root/'paired_baselines/soup/attempt_00_seed1030/robot_fit.json')))
regp=_official_regressor({k:p[k] for k in p.files if hasattr(p[k],'shape')},official)
mapping=np.asarray(fitj['base_mapping'],float)
theta_base=np.asarray(fitj['identified_base_parameters'],float)
Wbp=np.asarray(regp['W'])@mapping
model_robot=(Wbp@theta_base+np.asarray(regp['w0']).reshape(-1)).reshape(n,7)
for name, yy in [('model_robot',fp-model_robot)]:
    for s in [1.,-1.]:
        th=np.linalg.lstsq(Y,s*yy.reshape(-1),rcond=1e-10)[0]
        com=th[1:]/th[0] if th[0]>0 else np.full(3,np.nan)
        print('FIT',name,'sign',s,'theta',th,'com',com,'rms',np.sqrt(np.mean((Y@th-s*yy.reshape(-1))**2)))
        print('modeltrue rms',np.sqrt(np.mean((Y@np.r_[0.45,0.45*true]-s*yy.reshape(-1))**2)) if 'true' in globals() else '')

if 'Yd' in globals():
    for name, yy in [('model_robot', (fp-model_robot).reshape(-1))]:
        th=np.linalg.lstsq(Yd,yy,rcond=1e-10)[0]
        print('FIT_DRAKE_J',name,'theta',th,'com',th[1:]/th[0],'rms',np.sqrt(np.mean((Yd@th-yy)**2)))

stat=np.linalg.norm(p['dq_ref'],axis=1)<0.01
print('STATIC_ROWS',int(stat.sum()))
print('Y singular',np.linalg.svd(Y,compute_uv=False), 'colnorm',np.linalg.norm(Y,axis=0))
print('STATIC_DELTA_MEAN',np.mean(delta[stat],axis=0) if stat.any() else None)
print('STATIC_TRUE_MEAN',np.mean(pred[stat],axis=0) if stat.any() else None)
print('STATIC_MODELRES_MEAN',np.mean((fp-model_robot).reshape(n,7)[stat],axis=0) if 'model_robot' in globals() and stat.any() else None)
for name,YY,yy in [('static',Y[stat.repeat(7)],delta[stat].reshape(-1))]:
    th=np.linalg.lstsq(YY,yy,rcond=1e-10)[0]
    print('FIT_STATIC',name,th,th[1:]/th[0],np.sqrt(np.mean((YY@th-yy)**2)), 'rank',np.linalg.matrix_rank(YY))
