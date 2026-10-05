"""Single command for frozen native ablations and reporting-only paper evidence."""
from pathlib import Path
import sys,subprocess,argparse
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/paper_structure.json');a=p.parse_args()
subprocess.run([sys.executable,str(ROOT/'scripts/run_paper_structure_benchmark.py'),'--config',str(a.config),'--stage','full'],check=True)
from report_paper_evidence import report
print(report(a.config))
