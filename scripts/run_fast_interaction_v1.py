"""Versioned short engineering runtime. Never launches a full Agent episode by default."""
import argparse, hashlib, json, os, sys, time, traceback, shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))

def provenance(job,out):
    files=sorted((ROOT/'interaction_runtime').glob('*.py'))+[ROOT/'configs/interaction_runtime_v1.json',Path(__file__)]
    result=dict(reference='eb80bf3',sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        active_closure_sha256=hashlib.sha256(Path(job['active_closure_source']).read_bytes()).hexdigest(),native_physics_hz=240,
        active_owner='interaction_runtime.runtime.Runtime.step; no legacy step/owner wrapper',mode='separate simulation runtime, not a frozen baseline modification')
    for source in files:
        target=out/'RUNTIME_SOURCES'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    out.joinpath('EXECUTABLE_PROVENANCE.json').write_text(json.dumps(result,indent=2));return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path);p.add_argument('--mode',choices=['FAST_DEV','EVALUATION'],default='FAST_DEV');p.add_argument('--test',choices=['free','session','fixture','guards'],default='free');p.add_argument('--cycles',type=int,default=4);a=p.parse_args()
    job=json.loads(a.job.read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True);provenance(job,out)
    from interaction_runtime.runtime import Runtime
    r=None;result=dict(status='INITIALIZING',physical_contact_test=False)
    try:
        r=Runtime(job,a.mode)
        if a.test=='free':
            import numpy as np
            home=r.qtarget[r.arm].copy();moved=home.copy();moved[0]+=.12;moved[2]-=.08
            r.g['phase']='SETTLE';r.hold(.3)
            for i in range(a.cycles):
                r.move_path([moved],'PREGRASP');r.move_path([home],'SAFE_PARK_MOTION')
            r.hold(.3);result.update(status='FREE_SPACE_COMPLETE',cycles=a.cycles,final_q=np.asarray(r.robot.get_joint_positions()).tolist())
        elif a.test=='guards':
            from interaction_runtime.guard_regression import run
            result=run(r)
        elif a.test=='fixture':
            from interaction_runtime.fixture import serve_fixture
            result=serve_fixture(r)
        else:
            from interaction_runtime.session import serve
            result=serve(r)
    except BaseException as e:
        result.update(status='FAILED',error=repr(e),traceback=traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:
        out.joinpath('TEST_RESULT.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        if r is not None:r.close()
    return 0 if result['status'] not in ['FAILED','INITIALIZING'] else 1
if __name__=='__main__':sys.exit(main())
