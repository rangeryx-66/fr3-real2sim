"""Independent real-contact observer check; never counts as capture success."""
from pathlib import Path
import sys,json,ast
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'scripts'),str(ROOT)]
from run_wrist_reconstruction_episode import source

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);a=p.parse_args()
    job=json.loads(a.job.read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    text=source();old='from wrist_reconstruction.session import run as run_skill'
    if text.count(old)!=1:raise RuntimeError('VALIDATION_HOOK_CHANGED')
    text=text.replace(old,'from wrist_reconstruction.release_validation import run as run_skill')
    ast.parse(text);(out/'expanded_validation_program.py').write_text(text)
    sys.argv=[sys.argv[0],'--job',str(a.job)]
    exec(compile(text,str(out/'expanded_validation_program.py'),'exec'),{'__name__':'__main__','__file__':__file__})
