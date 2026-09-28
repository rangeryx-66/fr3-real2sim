"""Verify fresh FR3 and R1 replay used the original benchmark inputs byte for byte."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/r1a7_fr3_exact_ab'
reference=json.loads((ROOT/'config/r1a7_fr3_exact_reference.json').read_text())
run=Path(json.loads((OUT/'fr3_orchestration.json').read_text())['trial_directory'])

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

rows=[]
for i,ref in enumerate(reference,1):
    fr3=json.loads((run/f'trial_{i:02d}.json').read_text())
    raw=json.loads((OUT/'raw'/f'trial_{i:02d}.json').read_text())
    adapted=json.loads((OUT/'adapted'/f'trial_{i:02d}.json').read_text())
    row=dict(trial=i,fr3_category=fr3['category'],raw_category=raw['category'],adapted_category=adapted['category'],
             original_cloud_matches=sha(ref['cloud'])==ref['cloud_sha256'],
             fresh_fr3_cloud_matches=sha(fr3['capture']['path'])==ref['cloud_sha256'],
             fresh_fr3_grasps_match=sha(run/f'trial_{i:02d}_grasps.json')==ref['grasps_sha256'],
             r1_raw_matches=raw.get('reference_input',{}).get('cloud_sha256')==ref['cloud_sha256'] and raw.get('reference_input',{}).get('grasps_sha256')==ref['grasps_sha256'],
             r1_adapted_matches=adapted.get('reference_input',{}).get('cloud_sha256')==ref['cloud_sha256'] and adapted.get('reference_input',{}).get('grasps_sha256')==ref['grasps_sha256'])
    rows.append(row)
report=dict(all_matched=all(all(value for key,value in row.items() if key.endswith('_matches')) for row in rows),rows=rows,
            point_cloud_sha256=reference[0]['cloud_sha256'],anygrasp_output_sha256=reference[0]['grasps_sha256'])
(OUT/'input_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report),flush=True)
