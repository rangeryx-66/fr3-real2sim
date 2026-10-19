"""Bounded frozen-shared planner check on four saved RGB-D cases, no Isaac."""
import argparse,json,shutil,time
from pathlib import Path
from types import SimpleNamespace
from xarm7_native_planning import NativeModel
from revolute10_fixed import authored_handle_colliders
from native_g1_visual_input import plan_observed
p=argparse.ArgumentParser();p.add_argument('--asset',required=True);a=p.parse_args();ROOT=Path(__file__).resolve().parents[1];old=ROOT/'results/native_g1_visual_input_20261014';B=ROOT/'results/native_g1_visual_reliability_20261015';asset=a.asset
job=json.loads((old/asset/'job.json').read_text());source=json.loads((Path(job['source'])/'report.json').read_text());model=NativeModel(None,job['asset_root'],source);prior=old/asset/('physical_01' if asset!='45600' else 'shadow_01');out=B/'offline'/asset;out.mkdir(parents=True,exist_ok=True);shutil.copyfile(prior/'cooked_initial.json',out/'cooked_initial.json');export=json.loads((out/'cooked_initial.json').read_text());allowed=authored_handle_colliders(export,job['grasp_geometry_candidates'][0]);folder=B/'diagnostics/observed'/('45671_retry' if asset=='45671' else asset)
try:
 c,d=plan_observed(dict(job=job,model=model,allowed=allowed,a=SimpleNamespace(output=out),ROOT=ROOT),folder)
 print('VISUAL_INPUT_OFFLINE_OK',asset,c['planned_reached_state'],flush=True)
except Exception as e:
 (out/'offline_failure.json').write_text(json.dumps(dict(error=repr(e)),indent=2));raise
