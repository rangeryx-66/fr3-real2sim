import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from known_model_diagnostic.two_station import plan
p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--export',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--wall-s',type=float,default=600);a=p.parse_args()
print(json.dumps(plan(ROOT,json.loads(a.job.read_text()),json.loads(a.export.read_text()),a.output,a.wall_s)),flush=True)
