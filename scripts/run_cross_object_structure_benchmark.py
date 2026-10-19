"""One command: freeze assets -> controls -> unknown interaction -> held-out -> report."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'configs/cross_object_structure.yaml')
    p.add_argument('--stage',choices=['freeze','controls','controls-structure','heldout-controls','benchmark','summary','full'],default='full')
    p.add_argument('--prediction-gpus',type=int,nargs='+',help='Optional compute allocation for post-episode control predictions')
    a=p.parse_args();c=json.loads(a.config.read_text())
    from cross_object_structure.benchmark import StructureBenchmark
    from cross_object_structure.reporting import report
    b=StructureBenchmark(c)
    if a.stage in ('freeze','full'):b.freeze()
    if a.stage in ('controls','full'):
        controls=b.controls()
        if not all(r['success'] for r in controls.values()):
            report(b);raise SystemExit('REGRESSION_CONTROL_FAILED_MAIN_TEST_NOT_STARTED')
    if a.stage in ('benchmark','full'):
        controls=json.loads((b.out/'regression_controls.json').read_text())
        if not all(r['success'] for r in controls.values()):raise SystemExit('REGRESSION_GATE_REQUIRED')
        b.batch()
    # Supplemental controls exercise the new refinement/held-out scheduler even
    # when all unseen episodes stop before it. They never change main-set gates.
    if a.stage in ('controls-structure','full'):b.controls(full_structure=True)
    if a.stage in ('heldout-controls','full'):b.control_predictions(a.prediction_gpus)
    if a.stage in ('summary','full','benchmark','controls-structure','heldout-controls'):report(b)

if __name__=='__main__':main()
