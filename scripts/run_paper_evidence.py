"""Single command for frozen native ablations and reporting-only paper evidence."""
from pathlib import Path
import sys,subprocess,argparse,os,time,json
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/paper_structure.json');a=p.parse_args()
# Join an already running frozen batch; never start duplicate native writers.
c=json.loads(a.config.read_text());deadline=datetime.fromisoformat(c['deadline_shanghai']).timestamp()
def active_batch():
    for d in Path('/proc').glob('[0-9]*'):
        try:
            argv=(d/'cmdline').read_bytes().replace(b'\x00',b' ').decode()
            if 'run_paper_structure_benchmark.py' in argv and (d/'cwd').resolve()==ROOT:return True
        except (FileNotFoundError,PermissionError,OSError):pass
    return False
if active_batch():print('Joining existing frozen batch; no duplicate native execution.',flush=True)
while active_batch():
    if time.time()>=deadline:raise SystemExit('CUTOFF_05_00: summarize completed native results separately')
    time.sleep(10)
subprocess.run([sys.executable,str(ROOT/'scripts/run_paper_structure_benchmark.py'),'--config',str(a.config),'--stage','full'],check=True)
from report_paper_evidence import report
print(report(a.config))
