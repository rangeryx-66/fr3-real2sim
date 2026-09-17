"""After-cohort read-only ACM counterfactual on recorded planned/actual states."""
import os,json,gzip,copy,argparse
from pathlib import Path
import numpy as np
import rclpy
p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=1006);a=p.parse_args()
ROOT=Path(__file__).resolve().parents[1];RUN=ROOT/'results/arena_complex40';r=json.loads((RUN/f'B_seed_{a.seed:04d}.json').read_text());os.environ['FR3_ARENA_TARGET']=r['target_class']
assert (RUN/'COMPLETE.json').exists(),'must not change PlanningScene during formal cohort'
from arena_backend import ArenaBackend
import plant
from moveit_msgs.srv import GetPlanningScene,GetStateValidity,ApplyPlanningScene
from moveit_msgs.msg import PlanningSceneComponents,RobotState
records=json.load(gzip.open(RUN/f'trace_seed_{a.seed:04d}.json.gz','rt'))
first=next(v for v in records['records'] if v['phase']=='PREGRASP' and max(v['forces'])>.1)
start=next(v for v in records['records'] if v['phase']=='PREGRASP')
snapshot=dict(names=records['names'],q=start['q'],box=r['initial_target']['position'],box_quat=r['initial_target']['quaternion_wxyz'],clutter=dict(obstacles=r['initial_layout']))
plant.state=lambda *args,**kwargs:copy.deepcopy(snapshot)
def forbidden(*args,**kwargs):raise RuntimeError('Read-only diagnostic prohibits plant commands')
plant.command=plant.submit=forbidden
rclpy.init();n=ArenaBackend(str(ROOT/'results/arena_preflight'),geometry_filter=True,lift_gain_scale=2.)
n.reset_scene()
get=GetPlanningScene.Request();get.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
original=n.call('get_planning_scene',get).scene.allowed_collision_matrix
changed=copy.deepcopy(original);i=changed.entry_names.index('box')
for name in ['fr3_leftfinger','fr3_rightfinger']:
    j=changed.entry_names.index(name);changed.entry_values[i].enabled[j]=False;changed.entry_values[j].enabled[i]=False
def set_acm(acm):
    req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.allowed_collision_matrix=acm;n.apply(req)
def check(q):
    req=GetStateValidity.Request();req.group_name='fr3_arm';req.robot_state.joint_state.name=records['names'];req.robot_state.joint_state.position=q
    v=n.call('check_state_validity',req);return dict(valid=v.valid,contacts=[(c.contact_body_1,c.contact_body_2) for c in v.contacts])
result=dict(seed=a.seed,first_physical_contact={k:first[k] for k in ['t','phase','forces','box','tcp']},actual_contact_state_original=check(first['q']))
try:
    set_acm(changed);result['actual_contact_state_finger_target_forbidden']=check(first['q'])
    traj=next(t for t in r['executions'] if t['stage']=='PREGRASP');hits=[]
    # Interpolate the saved global trajectory at the original physics interval.
    ts=np.array([v['t'] for v in traj['points']]);qs=np.array([v['q'] for v in traj['points']]);names=traj['joint_names'];indices=[records['names'].index(name) for name in names]
    for t in np.arange(0,ts[-1]+1e-8,1/60):
        q=np.array(start['q']);q[indices]=[np.interp(t,ts,qs[:,j]) for j in range(len(names))];v=check(q.tolist())
        if any('box' in pair and any('finger' in name for name in pair) for pair in v['contacts']):
            hits.append(dict(trajectory_time=t,q=q.tolist(),**v))
    result['planned_finger_target_contacts_with_ACM_forbidden']=hits;result['plant_commands_sent']=0;result['physics_replays']=0
    if hits:
        set_acm(original);result['first_planned_hit_under_original_ACM']=check(hits[0]['q'])
finally:set_acm(original);n.reset_scene();n.destroy_node();rclpy.shutdown()
out=ROOT/'results/arena_complex_summary';(out/f'pregrasp_acm_seed_{a.seed}.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
