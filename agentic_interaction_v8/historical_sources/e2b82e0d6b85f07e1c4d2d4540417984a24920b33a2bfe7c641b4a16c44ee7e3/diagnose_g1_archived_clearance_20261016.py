"""Post-hoc cooked-geometry distances at recorded states, never an executable gate."""
import argparse,gzip,json,pickle
from pathlib import Path
import numpy as np
import fcl,trimesh
from piper_mobile_demo.owned_scene import Shape,meshes
from xarm7_native_planning import PAD_FRAMES

def read(p):return json.loads(p.read_text())
def one(ep,t):
    d=ep/'physics_steps_chunks'
    for c in read(d/'index.json')['chunks']:
        rr=pickle.load(gzip.open(d/c['file'],'rb'))
        if rr[-1]['t']+1/240<t:continue
        return min(rr,key=lambda x:abs(x['t']-t))
    return rr[-1]
def separation(a,b):
    r=fcl.CollisionResult();fcl.collide(a.object,b.object,fcl.CollisionRequest(num_max_contacts=16,enable_contact=True),r)
    if r.is_collision:
        return -max(float(x.penetration_depth) for x in r.contacts)*1000
    return float(fcl.distance(a.object,b.object,fcl.DistanceRequest(),fcl.DistanceResult()))*1000

def evaluate(root):
    ep=root/'episode';r=read(ep/'report.json');zero=r['grasp_acquisition']['t'];data=read(ep/'cooked_initial.json')
    M0=np.asarray(one(ep,0)['T_moving_link']);Mi=np.linalg.inv(M0)
    allowed=read(ep/'sensor_metadata.json')['allowed_handle_colliders'];solids=[];others=[];fingers={}
    for s in data['shapes']:
        if not s.get('convexes'):continue
        if s['path'] in allowed:
            solids.extend(Shape(m,Mi@np.asarray(s['world_transform']),True,s['path'],'handle') for m in meshes(s))
        elif s['path'].startswith('/World/articulated_pose'):
            others.extend(Shape(m,Mi@np.asarray(s['world_transform']),True,s['path'],'context') for m in meshes(s))
        body=s.get('rigid_body_path','').split('/')[-1]
        if body in PAD_FRAMES:
            name='gripper_link1' if body=='left_finger' else 'gripper_link2'
            T=np.linalg.inv(PAD_FRAMES[body])@np.linalg.inv(s['rigid_body_world_transform'])@np.asarray(s['world_transform'])
            fingers[name]=[Shape(m,T,True,s['path'],name) for m in meshes(s)]
    # Context belongs to moving panel and fixed cabinet; at hold before motion
    # D is effectively M0. Report this approximation for later opening states.
    result=[]
    for dt in [0,.2,.2291666667,.2541666667,.3,.5,1.,1.325]:
        s=one(ep,zero+dt);D=np.asarray(s['T_moving_link']);row=dict(t=s['t']-zero,fingers={})
        for name,shapes in fingers.items():
            for z in shapes:z.place(np.asarray(s['finger_world_poses'][name]))
            for z in solids+others:z.place(D)
            ds=[separation(a,b) for a in shapes for b in solids]
            context=[(separation(a,b),b.path) for a in shapes for b in others]
            v=min(context,default=(None,None))
            row['fingers'][name]=dict(cooked_handle_signed_clearance_mm=min(ds),nearest_context_mm=v[0],nearest_context_path=v[1])
        result.append(row)
    return dict(states=result,context_motion_note='All context transformed with moving body: exact at closed hold to numerical motion; do not interpret later moving/fixed cabinet clearance as a certified path.',distance_semantics='FCL separation if disjoint; negative max FCL contact penetration if intersecting. Not PhysX solver separation/rest/contact offsets.')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--success',type=Path,required=True);p.add_argument('--failed',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    out={k:evaluate(v) for k,v in [('success',a.success),('failed',a.failed)]};a.output.write_text(json.dumps(out,indent=2));print('ARCHIVED_CLEARANCE_COMPLETE')
