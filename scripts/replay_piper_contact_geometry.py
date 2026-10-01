"""Replay every recorded closure/pull state against unchanged exported convexes and raw meshes."""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from validate_piper_cooked_contact import mesh_from,strict_pair
import trimesh
p=argparse.ArgumentParser(description=__doc__);p.add_argument('export',type=Path);p.add_argument('observations',type=Path);p.add_argument('output',type=Path);p.add_argument('--target-body',default='l_1');a=p.parse_args()
shapes=json.loads(a.export.read_text())['shapes'];fingers=[s for s in shapes if s.get('rigid_body_path','').split('/')[-1] in ['gripper_link1','gripper_link2'] and s.get('raw_points')];targets=[s for s in shapes if s.get('rigid_body_path','').split('/')[-1]==a.target_body or s['path'].split('/')[-1]==a.target_body]
if len(fingers)!=2 or not targets:raise RuntimeError('incomplete collision export')
prepared=[]
for f in fingers:
 name=f['rigid_body_path'].split('/')[-1];official=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{name}.stl',force='mesh');inner=official.vertices[:,2].min();pads=official.triangles[np.max(abs(official.triangles[:,:,2]-inner),axis=1)<1e-6];local=np.linalg.inv(f['rigid_body_world_transform'])@np.asarray(f['world_transform'])
 for t in targets:
  T=np.asarray(t['world_transform']);carrier=np.asarray(t.get('rigid_body_world_transform',T));target_local=np.linalg.inv(carrier)@T
  if t.get('analytic',{}).get('shape')=='box':raw=[trimesh.creation.box([float(t['analytic']['size'])]*3)];cooked=raw
  else:raw=mesh_from(t,False);cooked=mesh_from(t)
  for mode,fs,ts in [('physx_cooked',mesh_from(f),cooked),('raw_finger_raw_target',mesh_from(f,False),raw)]:
   for A in fs:
    for B in ts:prepared.append((name,mode,A,B,local,T,target_local,pads))
rows=json.loads(a.observations.read_text());out=[];counts={};first={}
for i,row in enumerate(rows):
 if row['phase'] not in ['CLOSE','CLOSURE_HOLD','PULL_DIAGNOSTIC','HOLD','OPEN_DOOR']:continue
 failures=[];intersections={};worst={}
 for name,mode,A,B,local,T,target_local,pads in prepared:
  finger=np.asarray(row['finger_world_poses'][name]);TA=finger@local;TB=np.asarray(row['T_moving_link'])@target_local if 'T_moving_link' in row else T
  check=strict_pair(A,TA,B,TB,pads,finger)
  intersections[mode]=intersections.get(mode,0)+check['fcl_contact_count']
  if not check['strict_valid']:
   bad=check['nonpad_witnesses'];failures.append({'mode':mode,'finger':name,'nonpad_count':len(bad),'unclassified':check['unclassified_contacts'],'max_distance_from_pad_m':max((x['distance_from_original_pad_m'] for x in bad),default=0.),'max_abs_local_z_m':max((abs(x['point_finger'][2]) for x in bad),default=0.)})
   counts[mode]=counts.get(mode,0)+1;first.setdefault(mode,{'index':i,'t':row['t'],'phase':row['phase'],'failure':failures[-1]})
 out.append({'index':i,'t':row['t'],'phase':row['phase'],'aperture_m':row.get('aperture_m',row.get('actual_aperture_m')),'intersections':intersections,'failures':failures})
 if len(out)%120==0:print('audited',len(out),flush=True)
summary={'samples_checked':len(out),'target_shapes':len(targets),'finger_shapes':len(fingers),'first_failure':first,'invalid_samples_by_mode':{m:sum(any(f['mode']==m for f in row['failures']) for row in out) for m in ['physx_cooked','raw_finger_raw_target']},'maximum_nonpad_distance_m':{m:max((f['max_distance_from_pad_m'] for row in out for f in row['failures'] if f['mode']==m),default=0.) for m in ['physx_cooked','raw_finger_raw_target']},'maximum_nonpad_normal_depth_m':{m:max((f['max_abs_local_z_m'] for row in out for f in row['failures'] if f['mode']==m),default=0.) for m in ['physx_cooked','raw_finger_raw_target']},'collision_tolerance_m':1e-6,'mesh_size_modified':False}
a.output.write_text(json.dumps({'summary':summary,'states':out},indent=2));print(json.dumps(summary,indent=2))
