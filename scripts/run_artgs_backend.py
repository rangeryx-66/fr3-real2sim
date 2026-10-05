"""Bounded official ArtGS coarse -> type prediction -> joint reconstruction."""
import argparse,json,os,sys,time,hashlib,traceback,importlib.metadata,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
 p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--backend',type=Path,default=ROOT/'third_party/artgs-official');p.add_argument('--stage',choices=['coarse','predict','joint','export','full'],default='full');p.add_argument('--coarse-iterations',type=int,default=3000);p.add_argument('--predict-iterations',type=int,default=3000);p.add_argument('--joint-iterations',type=int,default=5000);p.add_argument('--wall-s',type=int,default=10800);p.add_argument('--type-from-interaction',action='store_true');a=p.parse_args()
 source=a.source.resolve();backend=a.backend.resolve();sys.path.insert(0,str(backend))
 outputs=Path(str(source).replace('/data/','/outputs/'));outputs.mkdir(parents=True,exist_ok=True)
 runtime=outputs/'official_runtime';runtime.mkdir(exist_ok=True)
 if not (runtime/'arguments').exists():shutil.copytree(backend/'arguments',runtime/'arguments')
 # Official stages read/write relative arguments/*.json. Isolate these per
 # reconstruction, so parallel assets cannot overwrite each other's type.
 os.chdir(runtime)
 import torch
 from argparse import ArgumentParser
 from arguments import ModelParams,OptimizationParams,PipelineParams
 from pytorch_lightning import seed_everything
 seed_everything(61)
 outputs=Path(str(source).replace('/data/','/outputs/'));outputs.mkdir(parents=True,exist_ok=True)
 status=outputs/('backend_status_interaction_prior.json' if a.type_from_interaction else 'backend_status.json');start=time.time();completed=[]
 joint_name='artgs_interaction_prior' if a.type_from_interaction else 'artgs';reconstruction_name='reconstruction_interaction_prior' if a.type_from_interaction else 'reconstruction'
 def selected_type():
  if not a.type_from_interaction:return json.loads((runtime/'arguments/joint_types_cgs.json').read_text())['capture']['sensor'][source.name]
  audit=json.loads((source/'input_provenance.json').read_text());doc=json.loads((Path(audit['capture_root'])/'multistate_capture.json').read_text());estimate=doc['states'][audit['states'][1]].get('articulation_estimate') or {}
  if not estimate:
   memory=Path(audit['capture_root'])/'structured_memory.json'
   estimate=json.loads(memory.read_text()).get('estimated_articulation',{}) if memory.exists() else {}
  kind=estimate.get('joint_type')
  if kind not in ('revolute','prismatic'):raise RuntimeError('NO_OBSERVED_INTERACTION_TYPE_PRIOR')
  return 's,'+{'revolute':'r','prismatic':'p'}[kind]
 def save(state,**kw):status.write_text(json.dumps(dict(backend='official ArtGS',source=str(source),state=state,stages_completed=completed,elapsed_s=time.time()-start,**kw),indent=2))
 (outputs/('backend_provenance_interaction_prior.json' if a.type_from_interaction else 'backend_provenance.json')).write_text(json.dumps({'official_repository':'https://github.com/YuLiu-LY/ArtGS','official_commit':'7c1f41be2cb8b96abca13c6a9668dcf7e06d8c2a','mode':'scene-specific optimization, not general pretrained inference','joint_type_prior_from_measured_interaction':a.type_from_interaction,'official_code_hashes':{str(f.relative_to(backend)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [backend/'train_coarse.py',backend/'train_predict.py',backend/'train.py',backend/'scene/artgs.py']},'input_provenance_sha256':hashlib.sha256((source/'input_provenance.json').read_bytes()).hexdigest(),'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'iterations':{'coarse':a.coarse_iterations,'predict':a.predict_iterations,'joint':a.joint_iterations},'environment':{name:importlib.metadata.version(name) for name in ['torch','pytorch-lightning','tinycudann','pytorch3d','diff-gaussian-rasterization']},'GT_mesh_or_joint_input':False},indent=2))
 def params(name,iterations):
  q=ArgumentParser();lp=ModelParams(q);op=OptimizationParams(q);pp=PipelineParams(q);args=q.parse_args([])
  args.source_path=str(source);args.model_path=str(outputs/name);args.dataset='capture';args.subset='sensor';args.scene_name=source.name;args.num_slots=2;args.iterations=iterations;args.resolution=1;args.init_from_pcd=True;args.vis_cano=False;args.vis_center=False;args.random_bg_color=True;args.eval=True
  return args,lp,op,pp
 # Official scripts store scene metadata here; add only input schema fields,
 # not reference joints or reconstructed answers.
 for file,value in [('num_slots.json',2),('larger_motion_state.json',None),('joint_types_cgs.json',None)]:
  path=runtime/'arguments'/file;d=json.loads(path.read_text());d.setdefault('capture',{}).setdefault('sensor',{})
  if value is not None:d['capture']['sensor'][source.name]=value
  path.write_text(json.dumps(d,indent=2))
 def train(module,name,n,saving=False):
  args,lp,op,pp=params(name,n);module.args=args
  kw=dict(args=args,dataset=lp.extract(args),opt=op.extract(args),pipe=pp.extract(args))
  if saving:kw['saving_iterations']=[n]
  trainer=module.Trainer(**kw)
  for i in range(n):
   if time.time()-start>a.wall_s:raise TimeoutError('BACKEND_WALL_BUDGET')
   trainer.train_step()
   if i%250==0:save('RUNNING',stage=name,iteration=i)
  completed.append(name);save('STAGE_COMPLETE')
  return trainer
 try:
  save('RUNNING')
  if a.stage in ['coarse','full']:
   import train_coarse
   trainer=train(train_coarse,'coarse_gs',a.coarse_iterations)
   # Official next-stage path is hardcoded iteration_10000. Alias the actual
   # bounded checkpoint; provenance retains the true iteration count.
   ck=outputs/'coarse_gs'/'point_cloud';alias=ck/'iteration_10000'
   if not alias.exists():alias.symlink_to(f'iteration_{a.coarse_iterations}',target_is_directory=True)
   del trainer;torch.cuda.empty_cache()
  if a.stage in ['predict','full']:
   import train_predict
   trainer=train(train_predict,'pred',a.predict_iterations);del trainer;torch.cuda.empty_cache()
  if a.stage in ['joint','full']:
   import train as joint
   args,lp,op,pp=params(joint_name,a.joint_iterations)
   args.joint_types=selected_type()
   args.num_slots=len(args.joint_types.split(','));args.use_art_type_prior=True;args.type_source='EE-only interaction identification (type only; axis/geometry learned by ArtGS)' if a.type_from_interaction else 'official automatic train_predict';joint.args=args
   trainer=joint.Trainer(args,lp.extract(args),op.extract(args),pp.extract(args),[a.joint_iterations])
   for i in range(a.joint_iterations):
    if time.time()-start>a.wall_s:raise TimeoutError('BACKEND_WALL_BUDGET')
    trainer.train_step()
    if i%250==0:save('RUNNING',stage=joint_name,iteration=i)
   completed.append('artgs');save('STAGE_COMPLETE');export(trainer,outputs/reconstruction_name,args,pp.extract(args));completed.append('export')
  elif a.stage=='export':
   import train as joint
   args,lp,op,pp=params(joint_name,a.joint_iterations);args.joint_types=selected_type();args.type_source='EE-only interaction identification (type only; axis/geometry learned by ArtGS)' if a.type_from_interaction else 'official automatic train_predict';args.use_art_type_prior=True;args.num_slots=len(args.joint_types.split(','));joint.args=args
   trainer=joint.Trainer(args,lp.extract(args),op.extract(args),pp.extract(args),[a.joint_iterations]);export(trainer,outputs/reconstruction_name,args,pp.extract(args));completed.append('export')
  save('COMPLETE',actual_iterations={'coarse':a.coarse_iterations,'predict':a.predict_iterations,'joint':a.joint_iterations},GT_mesh_input=False,GT_axis_input=False)
 except BaseException as e:save('FAILED',error=str(e),traceback=traceback.format_exc());raise


def export(trainer,out,args,pipe):
 import torch,numpy as np,open3d as o3d
 from gaussian_renderer import render
 from utils.mesh_utils import GaussianExtractor
 out.mkdir(parents=True,exist_ok=True)
 d=trainer.deform.step(trainer.gaussians,is_training=False);types=trainer.deform.deform.joint_types[1:]
 joints=trainer.deform.deform.get_joint_param(types)
 def serial(x):
  if isinstance(x,np.ndarray):return x.tolist()
  if isinstance(x,np.generic):return x.item()
  raise TypeError(type(x))
 (out/'motion_inferred.json').write_text(json.dumps({'joint_types':types,'joints':joints,'source':'official learned ArtGS dual-quaternion model','GT_input':False,'joint_type_source':getattr(args,'type_source','official automatic train_predict')},default=serial,indent=2))
 bg=torch.zeros(3,device='cuda');mask=trainer.deform.deform.get_mask(trainer.gaussians.get_xyz,is_training=False)
 with torch.no_grad():
  for part in range(len(types)+1):
   rgbs=[];depths=[];views=trainer.scene.getTrainCameras_start()
   for view in views:
    values=d[0];pkg=render(view,trainer.gaussians,pipe,bg,values['d_xyz'],values['d_rotation'],vis_mask=(mask.argmax(-1)==part));rgbs.append(pkg['render'].clamp(0,1));depths.append(pkg['depth'])
   mesh=GaussianExtractor(views,rgbs,depths,depth_trunc=5).extract_mesh()
   o3d.io.write_triangle_mesh(str(out/f'part_{part}.ply'),mesh)
   if len(mesh.triangles)==0:raise RuntimeError(f'EMPTY_RECONSTRUCTED_MESH_PART_{part}')
  metrics=[]
  for state,views in enumerate([trainer.scene.getTestCameras_start(),trainer.scene.getTestCameras_end()]):
   for i,view in enumerate(views):
    v=d[state];pkg=render(view,trainer.gaussians,pipe,bg,v['d_xyz'],v['d_rotation']);image=pkg['render'].clamp(0,1);target=view.original_image.cuda();metrics.append({'state':state,'view':i,'rgb_rmse':float(torch.sqrt(((image-target)**2).mean()))})
    import torchvision
    torchvision.utils.save_image(image,str(out/f'heldout_state_{state}_view_{i}.png'))
  (out/'heldout_view_metrics.json').write_text(json.dumps(metrics,indent=2))
if __name__=='__main__':main()
