"""Two sequential bounded native checks; never runs an Agent research episode."""
import argparse,json,subprocess,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--prefix',required=True);p.add_argument('--gpu',type=int,default=3);a=p.parse_args()
    root=ROOT/'results/fast_runtime_v1_20261018';checks=[]
    for test,expected in [('free','FREE_SPACE_COMPLETE'),('guards','NATIVE_GUARD_REGRESSION_COMPLETE')]:
        label=a.prefix+'_'+test
        subprocess.run([sys.executable,str(ROOT/'scripts/launch_fast_runtime_short.py'),'--label',label,'--test',test,'--gpu',str(a.gpu)],check=True)
        folder=root/label;result=json.loads((folder/'TEST_RESULT.json').read_text());completion=json.loads((folder/'PROCESS_COMPLETION.json').read_text())
        checks.append(dict(test=test,status=result['status'],process=completion))
        (root/(a.prefix+'_SEQUENTIAL_CHECKS.json')).write_text(json.dumps(checks,indent=2))
        if result['status']!=expected or not completion['actor_exited']:return 1
    return 0
if __name__=='__main__':sys.exit(main())
