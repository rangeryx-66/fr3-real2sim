"""Official-scale ArtGS with explicit coarse review before joint optimization."""
from pathlib import Path
import sys,json,ast
ROOT=Path(__file__).resolve().parents[1]


def source():
    text=(ROOT/'scripts/run_artgs_backend.py').read_text()
    def replace(a,b):
        nonlocal text
        if text.count(a)!=1:raise RuntimeError('QUALITY_BACKEND_HOOK_CHANGED:'+a[:60])
        text=text.replace(a,b)
    replace("args.resolution=1;args.init_from_pcd=True;", "args.resolution=2 if name=='coarse_gs' else 8 if name=='pred' else 1;args.coarse_name='coarse_gs';args.opacity_reg_weight=.1;args.densify_grad_threshold=.001 if name!='coarse_gs' else args.densify_grad_threshold;args.init_from_pcd=True;")
    replace("  if a.stage in ['joint','full']:\n   import train as joint", "  if a.stage in ['joint','full']:\n   review=outputs/'coarse_review.json'\n   if not review.exists() or not json.loads(review.read_text()).get('full_optimization_allowed',False):raise RuntimeError('COARSE_REVIEW_REQUIRED_BEFORE_FULL')\n   import train as joint")
    replace("actual_iterations={'coarse':a.coarse_iterations,'predict':a.predict_iterations,'joint':a.joint_iterations}", "requested_iterations={'coarse':a.coarse_iterations,'predict':a.predict_iterations,'joint':a.joint_iterations},actual_iterations={'coarse':a.coarse_iterations if 'coarse_gs' in completed else 0,'predict':a.predict_iterations if 'pred' in completed else 0,'joint':a.joint_iterations if 'artgs' in completed else 0}")
    ast.parse(text);return text

if __name__=='__main__':
    text=source();exec(compile(text,str(ROOT/'scripts/run_artgs_backend.py'),'exec'),globals())
