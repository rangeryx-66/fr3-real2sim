import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from known_model_diagnostic.estimated_input import compare
p=argparse.ArgumentParser();p.add_argument('successful_output',type=Path);p.add_argument('destination',type=Path);a=p.parse_args();print(json.dumps(compare(ROOT,a.successful_output,a.destination)),flush=True)
