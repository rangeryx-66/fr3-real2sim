"""Post-episode measured-stream analysis, including unsuccessful final segments.

No object dynamics, GT effort or evaluator truth is imported. Original in-episode
updates remain intact. This is explicitly NOT an online result.
"""
import argparse,gzip,json,pickle
from pathlib import Path
from xarm_ft_known_joint_prior import native_joint_prior
from xarm_ft_id_estimator import process,estimate

p=argparse.ArgumentParser();p.add_argument('case',type=Path);a=p.parse_args();root=a.case/'episode'
if not (root/'EXPORT_COMPLETE.json').exists():raise ValueError('Exported scene required')
job=json.loads((a.case/'job.json').read_text());attempts=json.loads((root/'RECOVERY_EPISODE_RESULT.json').read_text())['attempts']
rows=[]
for part in sorted((root/'observations_chunks').glob('*.gz')):
 with gzip.open(part,'rb') as f:rows.extend(pickle.load(f))
for index,attempt in enumerate(attempts):
 selected=rows[attempt['first_row']:attempt['last_row']]
 if not any(v.get('identification_segment') for v in selected):continue
 prior=native_joint_prior(job['asset_root'],selected[0]['T_moving_link']);data=process(selected,prior,job['ft_calibration']['bias_sensor'])
 if data is None:continue
 out=root/'final_measurement_only_analysis'/('attempt_%02d'%index);model=estimate(data,out,prior)
 (out/'ANALYSIS_PROVENANCE.json').write_text(json.dumps(dict(source='private full-resolution observation chunks',first_row=attempt['first_row'],last_row=attempt['last_row'],
    classification='POSTHOC MEASUREMENT-ONLY, not an online update',includes_failed_segment=True,GT_dynamic_parameters_read=False),indent=2))
 print(index,len(selected),model.get('parameters'),model.get('heldout_torque_rmse_nm'))
