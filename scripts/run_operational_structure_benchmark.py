"""Calibrate DEV prediction scales -> DEV/regression -> freeze -> fresh TEST."""
import argparse,json,sys,concurrent.futures
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,default=ROOT/'configs/operational_structure.yaml')
    p.add_argument('--stage',choices=['calibrate','dev','controls','controls-structure','freeze','benchmark','summary','full'],default='full');a=p.parse_args()
    from operational_structure.benchmark import OperationalBenchmark
    b=OperationalBenchmark(json.loads(a.config.read_text()))
    try:
        if a.stage=='full':
            b.controls() # verified native regression reuse, or a fresh legacy run
            with concurrent.futures.ThreadPoolExecutor(max_workers=2) as p:
                dev=p.submit(b.dev);controls=p.submit(b.controls,True)
                dev.result();controls.result()
            b.freeze();b.batch()
            return
        if a.stage in ('dev','full'):b.dev()
        if a.stage in ('controls','full'):b.controls()
        if a.stage in ('controls-structure','full'):b.controls(full_structure=True)
        if a.stage in ('freeze','full'):b.freeze()
        if a.stage in ('benchmark','full'):b.batch()
    finally:
        if a.stage in ('summary','benchmark','full'):
            from operational_structure.reporting import report
            report(b)


if __name__=='__main__':main()
