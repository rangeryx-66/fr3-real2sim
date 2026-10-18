"""V3 physical Skill unchanged; explicit production V3.1 estimator entry point.

Do not run for archived-data validation. Numeric worker replay needs no Isaac.
"""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from run_efficient_probing_v2 import main
from interaction_runtime.probing_v3 import RuntimeV3
from interaction_runtime.estimator_v31 import serve_v31

ESTIMATOR_SOURCES=('scripts/update_probing_v31_model.py','scripts/xarm_ft_v31_estimator.py','scripts/xarm_ft_v1_estimator.py',
    'scripts/xarm_wrist_ft.py','scripts/diagnose_probing_v3.py','scripts/update_probing_v2_model.py',
    'scripts/run_informative_probing_v31.py','scripts/route_probing_approach_v3.py','configs/informative_probing_v3.json')

if __name__=='__main__':sys.exit(main(RuntimeV3,serve_v31,extra_sources=ESTIMATOR_SOURCES))
