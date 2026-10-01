"""Add measured probe evidence to an existing full cooked/raw replay result."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser(description=__doc__);p.add_argument('validation',type=Path);p.add_argument('observations',type=Path);p.add_argument('--ownership',type=Path,required=True);p.add_argument('--pull-plan',type=Path,required=True);a=p.parse_args();r=json.loads(a.validation.read_text());states=json.loads(a.observations.read_text())
if len(r.get('replay',[]))!=len(states):raise RuntimeError('replay does not cover the complete recorded trajectory')
loaded=[s for s in states if s['phase'] in ['CLOSURE_HOLD','PULL_DIAGNOSTIC','HOLD']];closed=[s for s in states if s['phase']=='CLOSURE_HOLD'];held=[s for s in states if s['phase']=='HOLD']
r['ownership_manifest_sha256']=hashlib.sha256(a.ownership.read_bytes()).hexdigest();r['observations_sha256']=hashlib.sha256(a.observations.read_bytes()).hexdigest();r['bilateral_pad_contact_all_hold_pull_samples']=bool(loaded) and all(min(s['ownership']['pad_forces_n'].values())>=.2 for s in loaded)
if closed and held:
 center=lambda s:np.mean([np.asarray(T)[:3,3] for T in s['finger_world_poses'].values()],axis=0)
 r['measured_mean_finger_origin_displacement_m']=float(np.linalg.norm(center(held[-1])-center(closed[-1])));r['measurement_rule']='mean actual finger rigid-body origins, closure hold to final pull hold; no commanded pose substitution';direction=np.asarray(json.loads(a.pull_plan.read_text())['waypoints'][0]['T'])[:3,0];direction=direction/np.linalg.norm(direction);delta=center(held[-1])-center(closed[-1]);r['measured_tangential_displacement_m']=float(delta@direction);r['off_axis_displacement_m']=float(np.linalg.norm(delta-(delta@direction)*direction));r['pull_plan_sha256']=hashlib.sha256(a.pull_plan.read_bytes()).hexdigest();r['physical_probe_reached_1mm']=r['measured_tangential_displacement_m']>=.001
else:r['physical_probe_reached_1mm']=False
r['physics_substeps_recorded']=len(states);r['complete_physics_substep_trace']=bool(states) and all(float(s.get('physics_dt_s',0))>0 for s in states);r['simulated_duration_s']=sum(float(s.get('physics_dt_s',0)) for s in states)
r['minimum_physics_joint_margin_rad']=min((s['margin_rad'] for s in states),default=0)
r['canonical_acceptance']=r['minimum_physics_joint_margin_rad']>.05 and r['complete_physics_substep_trace'] and r.get('unified_safe_all_samples',False) and r.get('native_contact_owner_mismatches',1)==0 and r['bilateral_pad_contact_all_hold_pull_samples'] and r['physical_probe_reached_1mm']
a.validation.write_text(json.dumps(r,indent=2));print({k:v for k,v in r.items() if k not in ['replay','cooked_pairs','raw_official_audit']})
