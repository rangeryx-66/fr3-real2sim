import sys,json,numpy as np
from pathlib import Path
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from piper_mobile_demo.model import Model
from piper_mobile_demo.owned_scene import meshes
from revolute10_fixed import authored_handle_colliders
from rank_prepared_handle_assets import load,hits
out=ROOT/'results/oracle_door_development_20261011';fig,axes=plt.subplots(3,2,figsize=(12,12));summary=[]
for row,i in enumerate(['45176','45203','45415']):
 job=json.loads((out/i/'job.json').read_text());src=json.loads((Path(job['source'])/'report.json').read_text());model=Model(ROOT/'config/piper.urdf',job['asset_root'],src);s=job['grasp_geometry_candidates'][0];e=json.loads(Path(job['collision_export']).read_text());allowed=authored_handle_colliders(e,s)
 _,pieces=load(model.asset_urdf);visual=next(p[3] for p in pieces if p[1]=='visual' and p[2]==s['mesh']);visual.apply_transform(model.asset_T)
 native=[]
 for ent in e['shapes']:
  if ent['path'] in allowed:
   for m in meshes(ent):m.apply_transform(np.array(ent['world_transform']));native.append(m)
 anchor=(model.asset_T@np.r_[next(x for x in s['sections'] if x['fraction']==0)['anchor_root_m'],1])[:3];n=model.asset_T[:3,:3]@s['outward_normal_root'];axis=model.asset_T[:3,:3]@s['axis_root'];close=np.cross(n,axis)
 cooked=np.vstack([m.vertices for m in native]);geom=json.loads((out/i/'planning/closing_geometry_audit.json').read_text())[0]
 record={'asset_id':i,'handle':s['mesh'],'visual_world_bounds':visual.bounds.tolist(),'cooked_world_bounds':[cooked.min(0).tolist(),cooked.max(0).tolist()],'anchor_world':anchor.tolist(),'anchor_to_cooked_center_m':float(np.linalg.norm(anchor-(cooked.min(0)+cooked.max(0))/2)),'visual_hits':hits(visual,anchor,n),'cooked_hits':[hits(m,anchor,n) for m in native],'sections':s['sections']};summary.append(record)
 for col,projection in enumerate([np.column_stack((axis,close)),np.column_stack((axis,n))]):
  ax=axes[row,col]
  for m in native:ax.triplot(*(m.vertices@projection).T,m.faces,color='tab:orange',lw=.25,alpha=.6)
  ax.triplot(*(visual.vertices@projection).T,visual.faces,color='tab:blue',lw=.25,alpha=.5)
  ax.plot(*(anchor@projection),'rx');ax.set_aspect('equal');ax.set_title(i+' '+s['mesh']+(' axis/closing' if col==0 else ' axis/depth'))
 print(json.dumps(record),flush=True)
fig.suptitle('Oracle handle geometry: blue visual / orange native convex hull / red candidate anchor');fig.tight_layout();fig.savefig(out/'handle_geometry_audit.png',dpi=130);(out/'HANDLE_COLLISION_AUDIT.json').write_text(json.dumps(summary,indent=2))
