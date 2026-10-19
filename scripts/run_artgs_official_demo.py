"""Run author checkpoint on its own data; never reuse it for target assets."""
import argparse,ast,json,sys,os,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--backend',type=Path,default=ROOT/'third_party/artgs-official');p.add_argument('--demo',type=Path,default=ROOT/'third_party/artgs_demo');p.add_argument('--output',type=Path,default=ROOT/'results/articulated_system_20261005/artgs_official_demo');a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);sys.path[:0]=[str(a.backend.resolve()),str(ROOT/'scripts')];os.chdir(a.backend)
import torch,torchvision,numpy as np
from arguments import ModelParams,PipelineParams,OptimizationParams
from argparse import ArgumentParser
from scene import Scene,GaussianModel,DeformModel
from gaussian_renderer import render
parser=ArgumentParser();lp=ModelParams(parser);pp=PipelineParams(parser);op=OptimizationParams(parser);args=parser.parse_args([])
ck=a.demo.resolve()/'ArtGS_ckpt/artgs/storage_45503';data=a.demo.resolve()/'ArtGS_raw_data/artgs/sapien/storage_45503'
node=ast.parse((ck/'cfg_args').read_text(),mode='eval').body
if not isinstance(node,ast.Call) or not isinstance(node.func,ast.Name) or node.func.id!='Namespace':raise ValueError('INVALID_OFFICIAL_CONFIG')
for kw in node.keywords:
 if hasattr(args,kw.arg):setattr(args,kw.arg,ast.literal_eval(kw.value))
args.source_path=str(data);args.model_path=str(ck);args.num_slots=len(args.joint_types.split(','));args.resolution=2
G=GaussianModel(args.sh_degree);scene=Scene(lp.extract(args),G,load_iteration='best');D=DeformModel(lp.extract(args));D.load_weights(str(ck),iteration='best')
with torch.no_grad():
 values=D.step(G,is_training=False);background=torch.zeros(3,device='cuda')
 for state,views in enumerate([scene.getTrainCameras_start(),scene.getTrainCameras_end()]):
  v=values[state];image=render(views[0],G,pp.extract(args),background,v['d_xyz'],v['d_rotation'])['render'].clamp(0,1);torchvision.utils.save_image(image,str(out/f'state_{state}.png'))
 joints=D.deform.get_joint_param(D.deform.joint_types[1:])
 def serial(x):return x.tolist() if hasattr(x,'tolist') else float(x)
 (out/'official_demo_result.json').write_text(json.dumps({'official_checkpoint_scene':'storage_45503','official_source_commit':'7c1f41be2cb8b96abca13c6a9668dcf7e06d8c2a','checkpoint_sha256':hashlib.sha256((ck/'deform/iteration_best/deform.pth').read_bytes()).hexdigest(),'gaussians':len(G.get_xyz),'inferred_types':D.deform.joint_types,'motion':joints,'checkpoint_used_for_7320_or_45746':False,'two_actual_native_renders':True},default=serial,indent=2))
print('OFFICIAL_ARTGS_DEMO_COMPLETED',flush=True)
