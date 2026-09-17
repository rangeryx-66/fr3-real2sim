"""Retry only static captures blocked by unused dynamic-protocol preparation."""
from pathlib import Path
import json,time
R=Path(__file__).resolve().parents[1];O=R/'results/fr3_qcomp_cross_object'
while not (O/'complete.json').exists():time.sleep(30)
d=json.load(open(O/'complete.json'));valid={x['name'] for x in json.load(open(O/'evaluation_cases.json'))};pending={}
for row in d['rows']:
 if row['object'] in valid:continue
 log=Path(row['path'])/'payload/pipeline.log'
 if log.exists() and 'no FR3 Fourier candidate satisfies calibration limits' in log.read_text():pending.setdefault(row['object'],[]).append(row['seed'])
objects=list(pending.items());(O/'infrastructure_retry_plan.json').write_text(json.dumps({'reason':'STATIC_CAPTURE_BLOCKED_BY_UNUSED_DYNAMIC_PREPARATION','objects':objects,'original_records_preserved':True},indent=2))
if objects:
 p=Path(__file__).with_name('run_cross_object_validation.py');s=p.read_text().replace("O=R/'results/fr3_qcomp_cross_object'","O=R/'results/fr3_qcomp_cross_object_infrastructure_retry'")
 start=s.index('objects=');end=s.index('\nfrozen=',start);s=s[:start]+'objects='+repr(objects)+s[end:]
 exec(compile(s,str(p),'exec'),{'__file__':str(p),'__name__':'__main__'})
(O/'infrastructure_retry_finished.json').write_text(json.dumps({'objects':objects}))
