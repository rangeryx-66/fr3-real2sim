"""Pre-outcome V0 audit; archived observations only, no Isaac or hidden V1 law."""
import argparse,json,subprocess,sys
from pathlib import Path
import numpy as np
from xarm_ft_v1_estimator import CONFIG,resolution_sensitivity,analyze
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root;old=root/'results/xarm_wrist_ft_id_v0_20261017';out=root/'results/xarm_wrist_ft_v1_20261017/offline';out.mkdir(parents=True,exist_ok=True)
# Audit existing unloaded residual by phase, preserving all measured peaks.
d=np.load(old/'sensor_smoke_00/processed_wrench.npz',allow_pickle=True);audit=[]
known_prior=json.loads((old/'case_02/episode/identification/attempt_00/03/INPUT.json').read_text())['prior']
axis=np.asarray(known_prior['axis_world']);origin=np.asarray(known_prior['origin_world_m']);w=d['gravity_and_inertia_compensated_world']
hinge_tau=(w[:,3:]+np.cross(d['T_world_sensor'][:,:3,3]-origin,w[:,:3]))@axis
for phase in np.unique(d['phase']):
 if not str(phase).startswith('UNLOADED_MOTION'):continue
 ix=np.flatnonzero(d['phase']==phase)[121:-121];w=d['gravity_and_inertia_compensated_world'][ix];inertia=d['tool_inertial_world'][ix];g=d['gravity_only_compensated_world'][ix];freq=np.fft.rfftfreq(len(ix),1/240);power=np.abs(np.fft.rfft(w-w.mean(0),axis=0))**2
 means=[]
 for bin_id in np.unique(np.floor(d['t'][ix]/.1)):
  ids=ix[np.floor(d['t'][ix]/.1)==bin_id]
  if len(ids)>=20:means.append(float(hinge_tau[ids].mean()))
 audit.append(dict(phase=str(phase),records=len(ix),hinge_native_rms_nm=float(np.sqrt(np.mean(hinge_tau[ix]**2))),hinge_100ms_bin_rms_nm=float(np.sqrt(np.mean(np.asarray(means)**2))),hinge_100ms_bin_abs_P95_nm=float(np.quantile(abs(np.asarray(means)),.95)),
     gravity_only_rms=np.sqrt(np.mean(g*g,axis=0)).tolist(),inertial_term_rms=np.sqrt(np.mean(inertia*inertia,axis=0)).tolist(),compensated_rms=np.sqrt(np.mean(w*w,axis=0)).tolist(),
     compensated_max_abs=np.max(abs(w),axis=0).tolist(),dominant_residual_frequency_hz=freq[1+np.argmax(power[1:],axis=0)].tolist(),fraction_residual_power_above_60hz=(power[freq>60].sum(0)/power.sum(0)).tolist()))
(out/'SENSOR_RESIDUAL_AUDIT.json').write_text(json.dumps(dict(phases=audit,interpretation='dynamic residual is not explained solely by centered boundary leakage; high-frequency native solver/measurement residual remains unresolved; no actuator changes',not_hardware_precision=True),indent=2))
packet=out/'V0_SEGMENT_LOCAL';packet.mkdir(exist_ok=True);original=old/'case_02/episode/identification/attempt_00/03'
if not (packet/'measurements.pkl').exists():(packet/'measurements.pkl').symlink_to(original/'measurements.pkl')
s=json.loads((original/'INPUT.json').read_text());s['previous_packets']=[];(packet/'INPUT.json').write_text(json.dumps(s,indent=2))
subprocess.run([sys.executable,str(root/'scripts/run_xarm_ft_v1_estimation.py'),str(packet)],check=True)
data=dict(np.load(packet/'CUMULATIVE_DATA.npz',allow_pickle=True));data['samples']=json.loads((packet/'CUMULATIVE_SAMPLES.json').read_text())
model=json.loads((packet/'IDENTIFIED_MODEL.json').read_text());quant=resolution_sensitivity(data)
(out/'V0_RESOLUTION_SENSITIVITY.json').write_text(json.dumps(quant,indent=2));(out/'V0_SEGMENT_LOCAL_SUMMARY.json').write_text(json.dumps(model,indent=2))
print(json.dumps(dict(local_models={k:dict(parameters=v['parameters'],supported=v['supported'],heldout_torque_rmse_nm=v.get('heldout_torque_rmse_nm')) for k,v in model['models'].items()},resolution=quant),indent=2))
