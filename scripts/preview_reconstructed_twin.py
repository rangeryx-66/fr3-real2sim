"""Independent Isaac URDF import/assembly visualization; not contact success."""
import argparse,json,os,sys,subprocess,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--twin',type=Path,required=True);p.add_argument('--gpu',type=int,default=4);a=p.parse_args();out=a.twin.resolve();os.environ['CUDA_VISIBLE_DEVICES']=str(a.gpu)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':720})
import numpy as np,cv2,omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.sensors.camera import Camera
from scipy.spatial.transform import Rotation
from pxr import UsdLux,Gf
world=World(stage_units_in_meters=1.)
light=UsdLux.DomeLight.Define(world.stage,'/World/light');light.CreateIntensityAttr(1300.)
from isaacsim.asset.importer.urdf import URDFImporter,URDFImporterConfig
from isaacsim.core.utils.stage import add_reference_to_stage
usd=URDFImporter(URDFImporterConfig(urdf_path=str(out/'reconstructed.urdf'),usd_path=str(out/'isaac_import'),fix_base=True,allow_self_collision=True,merge_fixed_joints=False,joint_drive_type='force',joint_target_type='position')).import_urdf()
if not usd:raise RuntimeError('RECONSTRUCTED_URDF_IMPORT_FAILED')
path='/World/reconstructed';add_reference_to_stage(str(usd),path)
robot=world.scene.add(SingleArticulation(prim_path=path,name='reconstructed'));world.reset()
update=json.loads((out/'twin_update.json').read_text());limit=update['inferred_joint']['motion_between_input_states']
# Root mesh is local, with no dataset/reference mesh loaded in this scene.
bounds=np.array(update['parts'][0]['bounds_link_m']);center=bounds.mean(0);radius=max(.6,float(np.linalg.norm(bounds[1]-bounds[0]))*1.6)
eye=center+np.array([radius,-radius,.7*radius]);forward=(center-eye)/np.linalg.norm(center-eye);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);R=np.column_stack((right,np.cross(forward,right),forward))
c=Camera(prim_path='/World/camera',resolution=(960,720),frequency=24);c.initialize();c.set_world_pose(eye,np.roll(Rotation.from_matrix(R).as_quat(),1),camera_axes='ros')
for _ in range(32):world.step(render=True)
ffmpeg=shutil.which('ffmpeg');v=subprocess.Popen([ffmpeg,'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','24','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(out/'reconstructed_preview.mp4')],stdin=subprocess.PIPE)
try:
 for i in range(192):
  phase=i/191;value=limit*(1-abs(2*phase-1));robot.set_joint_positions(np.array([value]));robot.set_joint_velocities(np.zeros(1));world.step(render=True)
  image=np.asarray(c.get_rgba())[:,:,:3].copy();cv2.putText(image,'INFERRED ARTGS TWIN / JOINT-DRIVEN IMPORT CHECK',(20,35),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,210,0),2);v.stdin.write(image.tobytes())
  if i in [0,96]:cv2.imwrite(str(out/f'isaac_import_{i}.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
 v.stdin.close();v.wait(timeout=30)
 (out/'import_validation.json').write_text(json.dumps({'URDF_imported':True,'joints':list(robot.dof_names),'joint_driven_visualization':True,'robot_contact_success':False,'reference_asset_loaded':False,'assembly_frames_checked':'root-normalized shared frame and moving link recentered at inferred axis','preview':'reconstructed_preview.mp4'},indent=2))
finally:app.close()
