"""Explicit input-frame repair; preserve original online packet/models unchanged.

Uses the SAME frozen numeric estimator on the SAME measured wrench streams.
Does not import hidden dynamics, tune any fit, or alter the running actor.
"""
import argparse,json,pickle,os,subprocess,sys,hashlib
from pathlib import Path
import numpy as np
from xarm_ft_known_joint_prior import native_joint_prior

p=argparse.ArgumentParser();p.add_argument('packet');a=p.parse_args();source=Path(a.packet)
settings=json.loads((source/'INPUT.json').read_text())
with (source/'measurements.pkl').open('rb') as stream:rows=pickle.load(stream)
prior=native_joint_prior(settings['prior']['geometry_reference'],rows[0]['T_moving_link'])
output=source/'native_joint_prior';output.mkdir(exist_ok=True)
measurements=output/'measurements.pkl'
if not measurements.exists():os.link(source/'measurements.pkl',measurements)
(output/'INPUT.json').write_text(json.dumps(dict(prior=prior,bias=settings['bias']),indent=2))
audit=dict(original_prior=settings['prior'],corrected_prior=prior,only_kinematic_prior_input_changed=True,
    perpendicular_origin_difference_m=float(np.linalg.norm(np.cross(np.asarray(prior['origin_world_m'])-settings['prior']['origin_world_m'],prior['axis_world']))),
    estimator_source_sha256=hashlib.sha256(Path(__file__).with_name('xarm_ft_id_estimator.py').read_bytes()).hexdigest(),
    original_packet_preserved=True,original_model_not_validated_as_joint_dynamics=True,hidden_parameters_read=False,actor_modified=False)
(output/'PRIOR_INPUT_REPAIR.json').write_text(json.dumps(audit,indent=2))
subprocess.run([sys.executable,str(Path(__file__).with_name('run_xarm_ft_estimation.py')),str(output)],check=True)
print(json.dumps(dict(output=str(output),prior_origin_world_m=prior['origin_world_m'],perpendicular_origin_difference_m=audit['perpendicular_origin_difference_m'])))
