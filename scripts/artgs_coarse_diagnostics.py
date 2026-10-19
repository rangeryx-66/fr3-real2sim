"""Save actual canonical GS, center/cluster plots and coarse held-out renders."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);a=p.parse_args();source=a.source.resolve();out=Path(str(source).replace('/data/','/outputs/'));backend=ROOT/'third_party/artgs-official';sys.path.insert(0,str(backend))
    import torch,torchvision,matplotlib
    matplotlib.use('Agg');import matplotlib.pyplot as plt
    from plyfile import PlyData
    folder=out/'coarse_gs/point_cloud/iteration_10000';dest=out/'coarse_diagnostics';dest.mkdir(parents=True,exist_ok=True)
    summary={'source':str(source),'GT_input':False,'full_optimization_allowed':False,'review_status':'REQUIRES_IMAGE_GEOMETRY_REVIEW'}
    import train_coarse
    from argparse import ArgumentParser
    from arguments import ModelParams,OptimizationParams,PipelineParams
    from gaussian_renderer import render
    q=ArgumentParser();lp=ModelParams(q);op=OptimizationParams(q);pp=PipelineParams(q);args=q.parse_args([]);args.source_path=str(source);args.model_path=str(out/'coarse_gs');args.dataset='capture';args.subset='sensor';args.scene_name=source.name;args.num_slots=2;args.iterations=10000;args.resolution=2;args.init_from_pcd=True;args.eval=True;train_coarse.args=args
    trainer=train_coarse.Trainer(args,lp.extract(args),op.extract(args),pp.extract(args));metrics=[]
    for state in (0,1):
        ply=PlyData.read(folder/f'point_cloud_{state}.ply')['vertex'];P=np.column_stack([ply[k] for k in ('x','y','z')]);C=np.clip(np.column_stack([ply[k] for k in ('f_dc_0','f_dc_1','f_dc_2')])*.2820947918+.5,0,1)
        i=np.arange(0,len(P),max(1,len(P)//12000));fig=plt.figure(figsize=(10,7));axis=fig.add_subplot(111,projection='3d');axis.scatter(*P[i].T,c=C[i],s=1);axis.set_title(f'actual coarse state {state} / {len(P)} Gaussians');fig.savefig(dest/f'coarse_cloud_state_{state}.png');plt.close(fig)
        g=trainer.gaussians[state];g.load_ply(str(folder/f'point_cloud_{state}.ply'))
        with torch.no_grad():
            views=trainer.scene.getTestCameras_start() if state==0 else trainer.scene.getTestCameras_end()
            for view_id,view in enumerate(views):
                values=render(view,g,pp.extract(args),torch.zeros(3,device='cuda'));rgb=values['render'].clamp(0,1);depth=values['depth'];torchvision.utils.save_image(torch.cat([view.original_image.cuda(),rgb],dim=2),str(dest/f'coarse_state_{state}_view_{view_id}.png'));np.save(dest/f'coarse_depth_state_{state}_view_{view_id}.npy',depth.cpu().numpy());metrics.append({'state':state,'view':view_id,'rgb_rmse':float(torch.sqrt(((rgb-view.original_image.cuda())**2).mean()))})
    static=np.load(folder/'xyz_static.npy');dynamic=np.load(folder/'xyz_dynamic.npy');centers=np.load(folder/'center_info.npy')
    fig=plt.figure(figsize=(10,7));axis=fig.add_subplot(111,projection='3d')
    for P,color,label in [(static,'gray','static'),(dynamic,'orange','moving')]:
        idx=np.arange(0,len(P),max(1,len(P)//12000));axis.scatter(*P[idx].T,c=color,s=1,label=label)
    axis.scatter(*centers[:,:3].T,c=['blue','red'],s=100,marker='x');axis.legend();axis.set_title('official static/moving initialization + part centers');fig.savefig(dest/'part_centers_clustering.png');plt.close(fig)
    summary['part_centers_normalized']=centers.tolist();summary['cluster_counts']={'static':len(static),'moving':len(dynamic)}
    summary['heldout_metrics']=metrics;summary['mean_rgb_rmse']=float(np.mean([r['rgb_rmse'] for r in metrics]));(dest/'diagnostics.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
if __name__=='__main__':main()
