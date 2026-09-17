"""After-cohort collision-only replay: fixed attachment versus measured target pose."""
import os,json,gzip,copy,argparse
from pathlib import Path
import numpy as np
import rclpy
p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=1025);a=p.parse_args()
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';r=json.loads((RUN/f'B_seed_{a.seed:04d}.json').read_text());os.environ['FR3_ARENA_TARGET']=r['target_class']
assert (RUN/'COMPLETE.json').exists(),'do not touch PlanningScene during formal cohort'
from arena_backend import ArenaBackend,collision_geometry
from clutter_backend import transform,TCP,TOUCH
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import AttachedCollisionObject
import plant
d=json.load(gzip.open(RUN/f'trace_seed_{a.seed:04d}.json.gz','rt'));records=d['records'];first=records[0]
snapshot=dict(names=d['names'],q=first['q'],box=r['initial_target']['position'],box_quat=r['initial_target']['quaternion_wxyz'],clutter=dict(obstacles=r['initial_layout']))
plant.state=lambda *args,**kwargs:copy.deepcopy(snapshot)
def forbidden(*args,**kwargs):raise RuntimeError('read-only diagnostic')
plant.command=plant.submit=forbidden
rclpy.init();n=ArenaBackend(str(ROOT/'results/arena_preflight'),geometry_filter=True,lift_gain_scale=2.)
n.reset_scene();X=np.array(r['T_TCP_target']);H0=transform(first['tcp'],first['tcp_quat']);n.attach(H0,H0@X);n.support_contact(False)
def check(v,relative):
    req=GetStateValidity.Request();req.group_name='fr3_arm';req.robot_state.joint_state.name=d['names'];req.robot_state.joint_state.position=v['q'];req.robot_state.is_diff=True
    att=AttachedCollisionObject();att.link_name=TCP;att.touch_links=TOUCH;att.object=collision_geometry('box',None,relative,TCP);req.robot_state.attached_collision_objects=[att]
    result=n.call('check_state_validity',req)
    return dict(valid=result.valid,contacts=[(c.contact_body_1,c.contact_body_2) for c in result.contacts])
rows=[]
try:
    for v in [x for x in records if x['phase']=='LIFT'][::12]:
        H=transform(v['tcp'],v['tcp_quat']);O=transform(v['box'],v['box_quat']);actual=np.linalg.inv(H)@O;delta=np.linalg.inv(X)@actual
        rows.append(dict(t=v['t'],fixed_attachment=check(v,X),measured_attachment=check(v,actual),relative_translation_m=float(np.linalg.norm(delta[:3,3])),relative_rotation_deg=float(np.rad2deg(Rotation.from_matrix(delta[:3,:3]).magnitude())),actual_T_TCP_target=actual.tolist()))
finally:n.reset_scene();n.destroy_node();rclpy.shutdown()
hits=[x for x in rows if x['fixed_attachment']['valid'] and any('box' in pair and any('clutter_' in name for name in pair) for pair in x['measured_attachment']['contacts'])]
result=dict(seed=a.seed,plant_commands=0,physics_replays=0,sample_dt=12/240,initial_attachment=X.tolist(),fixed_valid_measured_target_collision_samples=len(hits),max_relative_translation_m=max(x['relative_translation_m'] for x in rows),max_relative_rotation_deg=max(x['relative_rotation_deg'] for x in rows),rows=rows)
out=ROOT/'results/arena_complex_summary';(out/f'attachment_replay_seed_{a.seed}.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
