"""Three empty-arm poses, four seconds each; no payload fitting."""
from pathlib import Path
import argparse,json,numpy as np,rclpy
import plant
from unseen_backend import UnseenBackend
from settling_gate import GateThresholds
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args,_=p.parse_known_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
rclpy.init();backend=UnseenBackend(out,GateThresholds())
try:
 src=Path('/data1/home/rangeryx/fr3_moveit_grasp/results/fr3_strict_pair_validation/mug/manifest.json');d=json.load(open(src));poses=[d['poses'][i] for i in [0,3,7]]
 backend.target_policy(['fr3_leftfinger','fr3_rightfinger','fr3_hand'])
 records=[]
 for entry in poses:
  q=entry['q_ref'];state=backend.measured();names=[f'fr3_joint{i}' for i in range(1,8)]
  for name,value in zip(names,q):state.joint_state.position[state.joint_state.name.index(name)]=value
  backend.validate(state)
  move=plant.command({'op':'trajectory','names':names,'points':[{'t':4.,'q':q}]},timeout=180)
  plant.settle(2.)
  start=plant.command({'op':'dq_audit_start'})
  if not start.get('ok'):raise RuntimeError(start)
  plant.settle(4.)
  path=out/f"pose_{entry['pose_id']:02d}.npz";stop=plant.command({'op':'dq_audit_stop','path':str(path.resolve())})
  if not stop.get('ok'):raise RuntimeError(stop)
  records.append({'pose_id':entry['pose_id'],'q_ref':q,'movement':move,'record':stop})
  (out/'progress.json').write_text(json.dumps(records,indent=2))
 (out/'complete.json').write_text(json.dumps(records,indent=2))
finally:rclpy.shutdown()
