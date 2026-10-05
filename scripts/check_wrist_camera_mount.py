"""Rendered mount QA at an observed robot posture; diagnostic, not capture input."""
import argparse,json,os,sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'scripts'),str(ROOT)]

def main():
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--view-meta',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--gpu',type=int,default=5);a=p.parse_args()
    job=json.loads(a.job.read_text());meta=json.loads(a.view_meta.read_text());out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);(out/'error.txt').unlink(missing_ok=True)
    args=SimpleNamespace(source=Path(job['source']),asset_root=Path(job['asset_root']),output=out,gpu=a.gpu,ownership=None)
    job['robot_start_q']=meta['robot_q']
    from interactive_twin.plant import bootstrap_job
    scene=bootstrap_job(args,meta['robot_base_pose'],job)
    from wrist_reconstruction.scene import normal_background
    normal_background(scene['stage'])
    from isaacsim.sensors.camera import Camera
    from articulated_interaction_skill.capture import matrix
    from wrist_reconstruction.capture import snapshot
    import cv2
    E=matrix(*scene['tcp'].get_world_pose())
    camera=Camera('/World/mount_diagnostic',resolution=(1280,720),frequency=30);camera.initialize();camera.set_horizontal_aperture(12.8);camera.set_vertical_aperture(7.2);camera.set_focal_length(9.25);camera.set_clipping_range(.05,5.)
    camera.add_distance_to_image_plane_to_frame();camera.add_instance_id_segmentation_to_frame()
    rows=[];(out/'progress.txt').write_text('before play\n');scene['world'].play();(out/'progress.txt').write_text('after play\n')
    try:
        for index,y in enumerate([-.075,-.1,-.125]):
            X=np.eye(4);X[:3,3]=[0,y,-.1];T=E@X;camera.set_world_pose(T[:3,3],np.roll(Rotation.from_matrix(T[:3,:3]).as_quat(),1),camera_axes='ros')
            for frame in range(24):
                scene['world'].step(render=True)
                (out/'progress.txt').write_text(f'mount {index} frame {frame}\n')
            rgb,depth,seg=snapshot(camera);ids=[int(k) for k,v in seg['info']['idToLabels'].items() if '/World/Piper/' in str(v)]
            fraction=float(np.isin(seg['data'],ids).mean());cv2.imwrite(str(out/f'mount_{index}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
            rows.append({'T_tcp_camera_optical':X.tolist(),'robot_pixel_ratio':fraction,'image':f'mount_{index}.png'})
        (out/'mount_qa.json').write_text(json.dumps({'diagnostic_only':True,'robot_posture_source':'observed robot q; initialization, not physical manipulation','counts_as_robot_capture':False,'GT_hinge_input':False,'sensor_K':camera.get_intrinsics_matrix().tolist(),'rows':rows},indent=2))
        print(json.dumps(rows),flush=True)
    except BaseException:
        import traceback
        (out/'error.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
    finally:scene['app'].close()

if __name__=='__main__':main()
