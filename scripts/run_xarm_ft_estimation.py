"""Numeric worker: observes only saved robot sensor streams and kinematic prior."""
from pathlib import Path
import sys,json,pickle
from xarm_ft_id_estimator import process,estimate
p=Path(sys.argv[1]);settings=json.loads((p/'INPUT.json').read_text())
with (p/'measurements.pkl').open('rb') as f:rows=pickle.load(f)
data=process(rows,settings['prior'],settings['bias'])
if data is None:(p/'IDENTIFIED_MODEL.json').write_text(json.dumps(dict(status='insufficient sensor samples')))
else:estimate(data,p,settings['prior'])
