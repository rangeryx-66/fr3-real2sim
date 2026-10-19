"""Build a union-preserving owned contact partition from official geometry."""
import argparse,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.contact_ownership import build_manifest
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source-cooked-export',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.parent.mkdir(parents=True,exist_ok=True);r=build_manifest(ROOT,a.output,a.source_cooked_export)
print({n:{'pieces':len(s['pieces']),'volume_delta':s['partition_volume_m3']-s['source_hull_volume_m3']} for n,s in r['fingers'].items()})
