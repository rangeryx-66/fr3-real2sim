"""Bounded installation / handle-grasp kinematic margin screen, no execution.

Collision and MoveIt path validation remain mandatory in the live preflight.
"""
import argparse,json,sys,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'src'))
from urdf_chain import KinematicChain
from articulated_demo.kinematics import URDFChain


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--variants',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed-report',type=Path)
    p.add_argument('--xy-offsets-m',type=float,nargs='+',default=[-.015,0.,.015])
    p.add_argument('--yaw-offsets-deg',type=float,nargs='+',default=[-5.,0.,5.])
    p.add_argument('--endpoint-only',action='store_true')
    p.add_argument('--base-poses-file',type=Path,help='JSON array of [x,y,z,yaw_deg]; keep cabinet frozen')
    p.add_argument('--multi-start',type=int,default=1)
    p.add_argument('--min-all-joint-margin',type=float,default=.015)
    p.add_argument('--reverse',action='store_true',help='Trace from 22 degrees back to closed, then verify live forward planning')
    p.add_argument('--asset-root',type=Path,default=Path('/data1/home/rangeryx/datasets/physx_mobility/prepared/47686_v1'))
    args=p.parse_args()
    source=json.loads((args.source/'report.json').read_text())
    items=json.loads(args.variants.read_text())['candidates']
    manifest=json.loads((args.asset_root/'manifest.json').read_text())
    cabinet=URDFChain(args.asset_root/'urdf'/f"{manifest['asset_id']}.urdf")
    names=tuple(f'J{i}' for i in range(1,8));model=ROOT/'config/r1a7_dex1.urdf'
    joints={j.get('name'):j for j in ET.parse(model).findall('joint')}
    bounds=np.asarray([[float(joints[n].find('limit').get(k)) for k in ('lower','upper')] for n in names])
    chain=KinematicChain(model,'r1a7_world','r1a7_tcp',names)
    install=source['asset_installation'];x,y,yaw=install['x_m'],install['y_m'],install['yaw_deg']
    poses=[(x+dx,y+dy,yaw+da) for dx in args.xy_offsets_m
        for dy in args.xy_offsets_m for da in args.yaw_offsets_deg]
    def world_root(pose):
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',pose[2],degrees=True).as_matrix()@np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
        T[:3,3]=[pose[0],pose[1],install['fixture_height_m']-manifest['scale_source_to_meters']*manifest['static_source_bounds'][0][1]]
        return T
    moving=manifest.get('moving_link','abstract_2_1');joint=manifest['joint_name']
    source_moving=world_root((x,y,yaw))@cabinet.root_to_link(moving,{joint:source['planning_joint_q_rad']})
    seed_report=json.loads(args.seed_report.read_text()) if args.seed_report else source
    records=seed_report.get('candidate_results',seed_report.get('candidates',[]))
    seed=next((c['grasp_joint_q'] for c in records if c.get('grasp_joint_q')),source['home_q'])
    base_poses=json.loads(args.base_poses_file.read_text()) if args.base_poses_file else [None]
    cases=[((x,y,yaw),base) for base in base_poses] if args.base_poses_file else [(pose,None) for pose in poses]
    rng=np.random.default_rng(20260930);rows=[]
    for placement,base_pose in cases:
        if base_pose is not None:
            mount=np.eye(4);mount[:3,:3]=Rotation.from_euler('z',base_pose[3],degrees=True).as_matrix();mount[:3,3]=base_pose[:3]
            if chain.chain[0].name!='r1a7_world_mount':raise ValueError('unexpected world mount')
            chain.chain[0].origin[:]=mount
        for item in items:
            relative=np.linalg.inv(source_moving)@np.asarray(item['T_B_TCP'])
            matching=next((r for r in records if r.get('raw_rank',r.get('rank'))==item['raw_rank'] and r.get('variant')==item['variant'] and r.get('grasp_joint_q')),None)
            q=np.asarray(matching['grasp_joint_q'] if matching else seed).copy();trace=[]
            angles=[22.] if args.endpoint_only else [0.,4.,8.,12.,16.,20.,22.]
            if args.reverse:angles=angles[::-1]
            for angle in angles:
                target=world_root(placement)@cabinet.root_to_link(moving,{joint:math.radians(angle)})@relative
                rot=Rotation.from_matrix(target[:3,:3]);previous=q.copy()
                def error(qr):
                    T=chain.forward(dict(zip(names,qr)))
                    return np.r_[T[:3,3]-target[:3,3],.1*(rot.inv()*Rotation.from_matrix(T[:3,:3])).as_rotvec()]
                def objective(z):return -z[7]+.01*np.sum((z[:7]-previous)**2)
                def margin_constraint(z):return np.r_[z[4:7]-bounds[4:,0]-z[7],bounds[4:,1]-z[4:7]-z[7]]
                initial=np.r_[q,max(0.,np.minimum(q[4:]-bounds[4:,0],bounds[4:,1]-q[4:]).min())]
                low=bounds[:,0]+args.min_all_joint_margin;high=bounds[:,1]-args.min_all_joint_margin
                starts=[initial,*[np.r_[rng.uniform(low,high),.05] for _ in range(args.multi_start-1)]]
                fits=[]
                for start in starts:
                    start[:7]=np.clip(start[:7],low,high)
                    f=minimize(objective,start,method='SLSQP',
                        bounds=[*zip(low,high),(0.,1.5)],
                        constraints=[{'type':'eq','fun':lambda z:error(z[:7])},
                                     {'type':'ineq','fun':margin_constraint}],
                        options={'maxiter':100,'ftol':1e-8})
                    res=error(f.x[:7]);exact=np.linalg.norm(res[:3])<1e-4 and np.linalg.norm(res[3:])/.1<.003
                    fits.append((exact,f.x[7],-np.linalg.norm(res),f))
                fit=max(fits,key=lambda f:f[:3])[-1]
                q=fit.x[:7];res=error(q);margin=np.minimum(q-bounds[:,0],bounds[:,1]-q)
                exact=np.linalg.norm(res[:3])<1e-4 and np.linalg.norm(res[3:])/.1<.003
                trace.append({'angle_deg':angle,'exact':bool(exact),'q':q.tolist(),
                    'focus_margin_rad':float(margin[4:].min()),'all_joint_margin_rad':float(margin.min()),
                    'joint_margins_rad':dict(zip(names,margin.tolist()))})
                if not exact:break
            full=not args.endpoint_only and len(trace)==7 and all(t['exact'] for t in trace)
            row={'placement':placement,'raw_rank':item['raw_rank'],'variant':item['variant'],
                'robot_base_pose':base_pose,'multi_start':args.multi_start,
                'endpoint_only':args.endpoint_only,'reverse':args.reverse,
                'endpoint_exact':trace[0]['exact'] if args.endpoint_only else None,
                'endpoint_margin_rad':trace[0]['focus_margin_rad'] if args.endpoint_only and trace[0]['exact'] else None,
                'full_kinematic_arc':full,'arc_min_focus_margin_rad':min(t['focus_margin_rad'] for t in trace) if full else None,
                'trace':trace};rows.append(row)
            print(placement,item['variant'],full,row['arc_min_focus_margin_rad'],flush=True)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps({'kind':'exact URDF kinematic margin screen; no collision or MoveIt plan claim',
                'grid':poses,'rows':rows},indent=2))


if __name__=='__main__':main()
