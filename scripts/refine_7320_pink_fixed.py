"""Offline quarter-degree refinement of the7320 fixed-station candidates."""
from pathlib import Path
import sys,json,numpy as np,time
r=Path(__file__).resolve().parents[1];sys.path[:0]=[str(r),str(r/'scripts')]
from piper_pink_fixed import PinkIK
from piper_mobile_demo.model import Model
from interactive_twin_recovery.mobile import scene_at
from wrist_reconstruction.geometry import calibration
from wrist_reconstruction.planner import camera_clearance
root=r/'results/7320_pink_fixed_20261010';job=json.loads((root/'search_job.json').read_text());m=Model(r/'config/piper.urdf',job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()));old=json.loads(Path(job['whole_task_plan']).read_text());export=json.loads(Path(job['pink_collision_export']).read_text());T0=np.array(old['T_grasp']);j=m.manifest['joint_name'];link=m.manifest['moving_link'];F0=m.asset_T@m.asset.root_to_link(link,{j:0.});cal=calibration(job['camera_calibration']);out=root/'planning_consistent';out.mkdir(exist_ok=True)
a=[]
for name in ['planning','planning_refined','planning_fine']:a+=json.loads((root/name/'station_audit.json').read_text())
a.sort(key=lambda x:x['range_deg'],reverse=True);seen=set();pool=[]
for e in a:
 key=tuple(round(x,7) for x in e['base'])+(e.get('branch',0),)
 if key in seen:continue
 seen.add(key)
 if e['range_deg']>=54:pool.append(e)
 if len(pool)>=100:break
records=[];best=[];started=time.time()
for e in pool:
 ik=PinkIK(m.urdf,m.home);b=e['base'];sc=scene_at(r,export,m,old['base'],b);q=ik.solve(T0,b,starts=4);path=[];why='INITIAL_IK'
 if q is not None and e.get('branch',0):
  alt=q.copy();alt[3]+=np.pi if q[3]<0 else -np.pi;alt[4]*=-1;alt[5]+=np.pi if q[5]<0 else -np.pi;q=ik.solve(T0,b,alt)
 initial=None if q is None else q.copy()
 if q is not None:
  for deg in np.arange(0,90.001,.25):
   s=np.deg2rad(deg);D=m.asset_T@m.asset.root_to_link(link,{j:s})@np.linalg.inv(F0);T=D@T0;nq=ik.solve(T,b,q)
   if nq is None:why='PINK_TRACKING_LIMIT';break
   if m.margin(nq)<=.05:why='EXISTING_JOINT_MARGIN';break
   P=m.poses(nq,b,width=.024);ok,why=sc.check(P,D@sc.moving_reference,True)
   if ok:ok,why=camera_clearance(sc,P,cal)
   if not ok:break
   path.append({'state':float(s),'q':nq.tolist(),'T_tcp':T.tolist(),'margin_rad':m.margin(nq)});q=nq
 score=np.rad2deg(path[-1]['state']) if path else 0.;record={'base':b,'branch':e.get('branch',0),'range_deg':float(score),'blocker':why,'initial_q':None if initial is None else initial.tolist()};records.append(record)
 if score>=60:
  best.append((record,path));best.sort(key=lambda a:a[0]['range_deg'],reverse=True)
  (out/'best_path.json').write_text(json.dumps({'summary':best[0][0],'path':best[0][1]},indent=2))
 (out/'progress.json').write_text(json.dumps({'checked':len(records),'pool':len(pool),'best_deg':max(x['range_deg'] for x in records),'wall_s':time.time()-started}))
 print('FINE',len(records),round(score,2),b,flush=True)
(out/'station_audit.json').write_text(json.dumps(records,indent=2));print('DONE', (out/'progress.json').read_text(),flush=True)
