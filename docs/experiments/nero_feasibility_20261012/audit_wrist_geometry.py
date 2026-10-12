"""Raw official triangle geometry versus convex hulls, signed pair clearances."""
from pathlib import Path
import json,xml.etree.ElementTree as ET,numpy as np,trimesh,fcl,hashlib
from scipy.spatial import cKDTree
from piper_mobile_demo.model import geometry,fcl_geometry,obj
from articulated_demo.kinematics import URDFChain
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/nero_feasibility_20261012';c=json.loads((O/'NERO_MODEL.json').read_text());root=ET.parse(c['urdf']).getroot();chain=URDFChain(c['urdf']);raw={};vis={}
for n in ['link5','link6','link7','flange_link']:
 l=root.find(f"link[@name='{n}']");raw[n]=geometry(l.find('collision'),Path(c['urdf']).parent)
 try:vis[n]=geometry(l.find('visual'),Path(c['urdf']).parent)
 except Exception as e:vis[n]=None
samples=[{'source':'official_home','q':c['home_q']}]
old=json.loads((ROOT/'results/cross_robot_20261012/PURE_IK_DIAGNOSTIC.json').read_text())
for r in old:
 if r['robot']=='nero':samples += [{'source':r['asset']+'_'+str(p['angle_deg']),'q':p['q']} for p in r['path'][::max(1,len(r['path'])//3)]]
for q6 in [-.65,0,.85]:
 for q7 in [-1.4,0,1.4]:
  q=np.array(c['home_q'],float);q[5]=q6;q[6]=q7;samples.append({'source':'wrist_grid','q':q.tolist()})
records=[];shapeinfo={};pairs=[('link5','link7'),('link5','flange_link')]
for n,m in raw.items():
 a=vis[n];shapeinfo[n]={'vertices':len(m.vertices),'faces':len(m.faces),'watertight':bool(m.is_watertight),'bounds_m':m.bounds.tolist(),'raw_volume_m3':float(m.volume),'hull_volume_m3':float(m.convex_hull.volume),'visual_available':a is not None,'visual_vertices':len(a.vertices) if a is not None else None,'visual_to_collision_nearest_vertex_max_m':float(cKDTree(m.vertices).query(a.vertices)[0].max()) if a is not None else None}
print(shapeinfo,flush=True)
for s in samples:
 print('CONFIG',s['source'],flush=True)
 P={n:chain.root_to_link(n,dict(zip(c['arm_joint_names'],s['q']))) for n in raw}
 for a,b in pairs:
  A=obj(fcl_geometry(raw[a]),P[a]);B=obj(fcl_geometry(raw[b]),P[b]);co=fcl.CollisionResult();fcl.collide(A,B,fcl.CollisionRequest(num_max_contacts=64,enable_contact=True),co);dr=fcl.DistanceResult();gap=float(fcl.distance(A,B,fcl.DistanceRequest(enable_nearest_points=True),dr))
  H1=obj(fcl_geometry(raw[a].convex_hull),P[a]);H2=obj(fcl_geometry(raw[b].convex_hull),P[b]);cr=fcl.CollisionResult();fcl.collide(H1,H2,fcl.CollisionRequest(num_max_contacts=64,enable_contact=True),cr)
  # Signed volumetric clearance: outside positive, inside negative. Preserve
  # availability and watertight qualifications; do not call a BVH sentinel -1m.
  signed=None;issue=None
  try:
   ma=raw[a].copy();ma.apply_transform(P[a]);mb=raw[b].copy();mb.apply_transform(P[b]);pointsA=np.vstack([ma.vertices,ma.triangles_center]);pointsB=np.vstack([mb.vertices,mb.triangles_center]);pointsA=pointsA[np.linspace(0,len(pointsA)-1,min(512,len(pointsA)),dtype=int)];pointsB=pointsB[np.linspace(0,len(pointsB)-1,min(512,len(pointsB)),dtype=int)];dA=-trimesh.proximity.signed_distance(mb,pointsA);dB=-trimesh.proximity.signed_distance(ma,pointsB);signed=float(min(dA.min(),dB.min()))
  except Exception as e:issue=repr(e)
  records.append({'source':s['source'],'q':s['q'],'pair':[a,b],'raw_triangle_intersections':len(co.contacts),'raw_surface_clearance_m':gap if gap>=0 else None,'raw_FCL_collision_sentinel':gap if gap<0 else None,'signed_sample_clearance_m':signed,'signed_samples_per_mesh_max':512,'signed_meshes_watertight':bool(raw[a].is_watertight and raw[b].is_watertight),'signed_issue':issue,'hull_contact_count':len(cr.contacts),'hull_max_contact_penetration_m':max((float(x.penetration_depth) for x in cr.contacts),default=0.),'nearest_points_world':[[float(z) for z in x] for x in dr.nearest_points]})
(O/'WRIST_COLLISION_AUDIT_PROGRESS.json').write_text(json.dumps(records,indent=2))
fig=plt.figure(figsize=(12,7))
for i,s in enumerate([samples[0],samples[1],samples[-1]]):
 ax=fig.add_subplot(2,3,i+1,projection='3d');P={n:chain.root_to_link(n,dict(zip(c['arm_joint_names'],s['q']))) for n in raw};points=[]
 for n,color in [('link5','tab:blue'),('link6','gray'),('link7','orange'),('flange_link','red')]:
  m=raw[n].copy();m.apply_transform(P[n]);points.extend(m.vertices);ax.add_collection3d(Poly3DCollection(m.triangles,facecolor=color,alpha=.55,linewidth=0))
 pts=np.array(points);lo,hi=pts.min(0),pts.max(0);mid=(lo+hi)/2;span=max(hi-lo)/2;ax.set_xlim(mid[0]-span,mid[0]+span);ax.set_ylim(mid[1]-span,mid[1]+span);ax.set_zlim(mid[2]-span,mid[2]+span);ax.view_init(25,40);ax.set_title('Raw '+s['source'])
 ax=fig.add_subplot(2,3,i+4,projection='3d')
 for n,color in [('link5','tab:blue'),('link7','orange'),('flange_link','red')]:
  m=raw[n].convex_hull.copy();m.apply_transform(P[n]);ax.add_collection3d(Poly3DCollection(m.triangles,facecolor=color,alpha=.35,linewidth=0))
 ax.set_xlim(mid[0]-span,mid[0]+span);ax.set_ylim(mid[1]-span,mid[1]+span);ax.set_zlim(mid[2]-span,mid[2]+span);ax.view_init(25,40);ax.set_title('Convex hull proxy')
fig.suptitle('NERO wrist assembly: link5 blue / link6 gray / link7 orange / flange red\nOfficial collision triangles, same URDF transforms; no collision exemption applied');fig.tight_layout(rect=[0,0,1,.91]);fig.savefig(O/'WRIST_COLLISION_COMPARISON.png',dpi=160)
result={'classification':'GEOMETRY_DIAGNOSTIC_ONLY','official_source_commit':c['source_commit'],'URDF_sha256':hashlib.sha256(Path(c['urdf']).read_bytes()).hexdigest(),'shapes':shapeinfo,'configurations':len(samples),'records':records,'no_colliders_changed':True};(O/'WRIST_COLLISION_AUDIT.json').write_text(json.dumps(result,indent=2));print(shapeinfo);print([(p,sum(x['raw_triangle_intersections']>0 for x in records if x['pair']==list(p)),min(x['raw_surface_clearance_m'] for x in records if x['pair']==list(p) and x['raw_surface_clearance_m'] is not None)) for p in pairs])
