"""Static asset preview only. No grasp controller or benchmark attempt."""
from piper_mobile_execute import *
from types import SimpleNamespace
from interactive_twin.plant import bootstrap_job
from interactive_twin_refinement.assembly import install_setup_capture
p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);args=p.parse_args();job=json.loads(args.job.read_text())
a=SimpleNamespace(source=Path(job['source']),asset_root=Path(job['asset_root']),output=Path(job['output']),gpu=job['gpu'],ownership=None)
a.output.mkdir(exist_ok=True,parents=True);base=json.loads((a.source/'report.json').read_text())['robot_base_pose']
install_setup_capture();scene=bootstrap_job(a,base,job);world=scene['world'];app=scene['app'];world.pause(); passive_warmup_q=scene['articulation'].get_joint_positions().tolist(); scene['articulation'].set_joint_positions(np.zeros(len(passive_warmup_q))); scene['articulation'].set_joint_velocities(np.zeros(len(passive_warmup_q))); world.play(); world.step(render=False,update_fabric=True); world.pause()
try:
 from pxr import UsdGeom,Usd
 import cv2
 stage=scene['stage'];UsdGeom.Imageable(stage.GetPrimAtPath('/World/Piper')).MakeInvisible()
 cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy],useExtentsHint=False)
 box=cache.ComputeWorldBound(stage.GetPrimAtPath(scene['asset_path'])).ComputeAlignedRange()
 lo=np.asarray(box.GetMin());hi=np.asarray(box.GetMax());center=(lo+hi)/2;extent=hi-lo
 front=np.asarray(scene['asset_rotation'])@np.array([0.,0.,1.]);front/=np.linalg.norm(front)
 side=np.asarray(scene['asset_rotation'])@np.array([1.,0.,0.]);side/=np.linalg.norm(side)
 camera=scene['overview'][1];camera.set_clipping_range(.01,20.)
 for name,lateral in [('front',0.),('oblique',.4)]:
  direction=front+lateral*side+np.array([0.,0.,.18]);direction/=np.linalg.norm(direction)
  eye=center+direction*max(.4,float(np.max(extent))*1.8)
  forward=(center-eye)/np.linalg.norm(center-eye);right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
  R=np.column_stack((right,np.cross(forward,right),forward));camera.set_world_pose(position=eye,orientation=np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros')
  for _ in range(20):world.render()
  rgba=np.asarray(camera.get_rgba());cv2.imwrite(str(a.output/(name+'_closed.png')),cv2.cvtColor(rgba[:,:,:3],cv2.COLOR_RGB2BGR))
 (a.output/'closed_preview_metadata.json').write_text(json.dumps({'asset_id':scene['asset_id'],'purpose':'STATIC_ASSET_PREVIEW_ONLY; not a physical attempt','bounds':[lo.tolist(),hi.tolist()],'passive_warmup_joint':passive_warmup_q,'closed_pose_for_static_visualization_only':True,'actual_joint_at_render':scene['articulation'].get_joint_positions().tolist(),'original_urdf_sha256':hashlib.sha256((a.asset_root/'urdf'/(str(scene['asset_id'])+'.urdf')).read_bytes()).hexdigest()},indent=2))
finally:app.close()
