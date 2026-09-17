"""Post-batch read-only collision audit of the recorded seed-18 pregrasp path."""
import copy
import json
import numpy as np
import rclpy
from clutter_backend import ClutterBackend, ROOT, GROUP
from moveit_msgs.srv import GetStateValidity
import plant

rclpy.init();node=ClutterBackend(ROOT/'results/clutter_path_diagnostic')
plant.command(dict(op='reset',seed=18));plant.settle(1.)
node.mode='B';node.initial=plant.state()
episode=json.loads((ROOT/'results/clutter_ab_20_v1/B_seed_0018.json').read_text())
traj=episode['executions'][0];points=traj['points'];samples=[]
for i in range(len(points)-1):
    a,b=points[i],points[i+1];qa,qb=np.array(a['q']),np.array(b['q'])
    count=max(1,int(np.ceil(np.max(np.abs(qb-qa))/.002)))
    for j in range(count):
        f=j/count;samples.append((a['t']*(1-f)+b['t']*f,qa*(1-f)+qb*f))
samples.append((points[-1]['t'],np.array(points[-1]['q'])))
results=[]
for margin in [0.,.001,.002]:
    node.margin=margin;node.reset_scene();start=node.measured();collisions=[]
    for t,q in samples:
        state=copy.deepcopy(start)
        for name,value in zip(traj['joint_names'],q):state.joint_state.position[state.joint_state.name.index(name)]=float(value)
        req=GetStateValidity.Request();req.robot_state=state;req.group_name=GROUP
        r=node.call('check_state_validity',req)
        if not r.valid:collisions.append(dict(t=t,contacts=[[c.contact_body_1,c.contact_body_2] for c in r.contacts]))
    results.append(dict(margin_m=margin,samples=len(samples),colliding_samples=len(collisions),first_collisions=collisions[:10]))
    print(json.dumps(results[-1]),flush=True)
out=dict(scope='Post-batch planning-only diagnostic, same saved path; no execution and excluded from formal A/B',max_joint_interpolation_step_rad=.002,results=results)
(node.output/'path_audit.json').write_text(json.dumps(out,indent=2));node.margin=0;node.reset_scene()
node.destroy_node();rclpy.shutdown()
