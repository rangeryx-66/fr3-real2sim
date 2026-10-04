"""One bounded DEV -> freeze -> fresh TEST articulation experiment."""
import argparse,json,sys,concurrent.futures,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'configs/active_structure.yaml')
    p.add_argument('--stage',choices=['dev','controls','freeze','benchmark','summary','full'],default='full')
    a=p.parse_args();c=json.loads(a.config.read_text())
    from active_structure.benchmark import ActiveBenchmark
    b=ActiveBenchmark(c)
    if a.stage=='full':
        # Stage A qualifies actual bounded refinement entry, not a favorable
        # final DEV outcome. Finish those fixed budgets after the method freeze.
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            dev=pool.submit(ActiveBenchmark(c).dev)
            controls=pool.submit(ActiveBenchmark(c).controls)
            controls.result()
            while True:
                try:b.freeze_method();break
                except RuntimeError as error:
                    if not str(error).startswith('DEV_REFINEMENT_ENTRY_NOT_VALIDATED') or dev.done() or b.expired():raise
                    time.sleep(10.)
            b.freeze();b.batch();dev.result()
        from active_structure.reporting import report
        report(b);return
    if a.stage=='dev':b.dev()
    if a.stage in ('controls','full'):b.controls()
    if a.stage in ('freeze','full'):b.freeze()
    if a.stage in ('benchmark','full'):b.batch()
    if a.stage in ('summary','benchmark','full'):
        from active_structure.reporting import report
        report(b)

if __name__=='__main__':main()
