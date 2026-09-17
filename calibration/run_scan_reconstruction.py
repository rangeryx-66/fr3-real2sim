"""Restartable scan quality -> BundleSDF -> texture -> evaluation workflow."""
from __future__ import annotations
import argparse,json,time
from pathlib import Path
from real2sim.scan_quality import prepare
from real2sim.bundlesdf_adapter import run
from real2sim.texture_bake import bake
from real2sim.pass_alignment import audit
from real2sim.scan_metrics import evaluate


def main():
    p=argparse.ArgumentParser();p.add_argument('--raw-scan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--official-root',type=Path,default=Path('/data1/home/rangeryx/scalable-real2sim'));p.add_argument('--gt-mesh',type=Path);p.add_argument('--optimized',action='store_true');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    t=time.monotonic();quality=prepare(a.raw_scan,a.output/'dataset');tq=time.monotonic()
    recon=run(a.output/'dataset',a.output/'bundlesdf',a.official_root,a.optimized);tr=time.monotonic()
    texture=bake(Path(recon['products']['mesh']),a.output/'dataset',a.output/'bundlesdf',a.output/'textured');tt=time.monotonic()
    alignment=audit(a.output/'dataset',a.output/'bundlesdf');(a.output/'pass_alignment.json').write_text(json.dumps(alignment,indent=2))
    summary={'quality':quality,'reconstruction':recon,'texture':texture,'pass_alignment':alignment,'time_seconds':{'preprocess':tq-t,'bundlesdf':tr-tq,'texture':tt-tr,'total':tt-t}}
    if a.gt_mesh:summary['evaluation']=evaluate(Path(texture['products']['textured_mesh']),a.gt_mesh,a.output/'dataset',a.output/'bundlesdf',a.output/'textured/texture_report.json')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
