"""First native non-pad impulse: exact raw/cooked distances and contact plot.

Diagnostic only. No pose search, parameter tuning or contact-acceptance changes.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np,trimesh,fcl
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from piper_mobile_demo.owned_scene import meshes,Shape

def minimum(source,targets):
 best=None
 for target in targets:
  result=fcl.CollisionResult();hit=fcl.collide(source.object,target.object,fcl.CollisionRequest(enable_contact=True,num_max_contacts=64),result)>0
  if hit:
   depth=max([float(c.penetration_depth) for c in result.contacts]+[0.]);distance=0.;nearest=None
  else:
   r=fcl.DistanceResult();distance=float(fcl.distance(source.object,target.object,fcl.DistanceRequest(enable_nearest_points=True),r));depth=0.;nearest=[np.asarray(x).tolist() for x in r.nearest_points]
  value={'surface_to_proxy_solid_distance_m':distance,'intersects_proxy_solid':hit,'FCL_penetration_depth_m':depth,'target':target.path,'nearest_points_world_m':nearest}
  if best is None or (hit,depth,-distance)>(best['intersects_proxy_solid'],best['FCL_penetration_depth_m'],-best['surface_to_proxy_solid_distance_m']):best=value
 return best

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,required=True);a=p.parse_args();data=json.loads((a.trial/'cooked_initial.json').read_text());steps=json.loads((a.trial/'physics_steps.json').read_text());obs=json.loads((a.trial/'observations.json').read_text());truth=json.loads((a.trial/'evaluation_gt.json').read_text());allowed=json.loads((a.trial/'association.json').read_text())['allowed_pad_targets'];ownership=json.loads((ROOT/'config/piper_contact_ownership.json').read_text())
 index,step,contact=next((i,s,c) for i,s in enumerate(steps) for c in s['contacts'] if c['owner']!='pad' and c['force_n']>0)
 if 'contact_point_world_m' not in contact or 'native_normal_world' not in contact:raise RuntimeError('REPLAY_WITH_NATIVE_CONTACT_POINT_AND_NORMAL_REQUIRED')
 entries={e['path']:e for e in data['shapes']};active=entries[contact['collider']];target_entry=entries[contact['target']];H=np.asarray(step['diagnostic_target_world_pose_pre']);F=np.asarray(step['finger_world_poses'][contact['finger']]);point=np.asarray(contact['contact_point_world_m']);normal=np.asarray(contact['native_normal_world'])
 def transform_entry(e,post=False):
  body=np.asarray(e['rigid_body_world_transform']);relative=np.linalg.inv(body)@np.asarray(e['world_transform'])
  if e.get('finger'):B=np.asarray((obs[index] if post else step)['finger_world_poses'][e['finger']])
  else:B=np.asarray(truth[index]['T_moving_link']) if post else H
  return B@relative
 def shape(e,cooked=True,post=False):
  m=trimesh.util.concatenate(meshes(e,cooked));m.apply_transform(transform_entry(e,post));return Shape(m,np.eye(4),cooked,e['path'],e.get('finger') or 'handle',e.get('owner'))
 targets=[shape(entries[x]) for x in allowed];posttargets=[shape(entries[x],post=True) for x in allowed]
 raw_results={};pad_results={};cooked_pad_results={};post_raw_results={};post_pad_results={};pad_shapes=[];raw_shapes=[]
 for finger,spec in ownership['fingers'].items():
  mesh=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{finger}.stl',force='mesh');mask=np.array([i in set(spec['official_pad_face_ids']) for i in range(len(mesh.faces))]);B=np.asarray(step['finger_world_poses'][finger]);raw=mesh.copy();raw.update_faces(~mask);raw.remove_unreferenced_vertices();raw.apply_transform(B);pad=mesh.copy();pad.update_faces(mask);pad.remove_unreferenced_vertices();pad.apply_transform(B)
  rawshape=Shape(raw,np.eye(4),False,'official_nonpad_'+finger,finger);padshape=Shape(pad,np.eye(4),False,'official_pad_surface_'+finger,finger);raw_shapes.append(rawshape);pad_shapes.append(padshape);raw_results[finger]=minimum(rawshape,targets);pad_results[finger]=minimum(padshape,targets)
  postB=np.asarray(obs[index]['finger_world_poses'][finger]);delta=postB@np.linalg.inv(B);rp=raw.copy();pp=pad.copy();rp.apply_transform(delta);pp.apply_transform(delta);post_raw_results[finger]=minimum(Shape(rp,np.eye(4),False,'official_nonpad_post',finger),posttargets);post_pad_results[finger]=minimum(Shape(pp,np.eye(4),False,'official_pad_post',finger),posttargets)
  pad_entry=next(e for e in data['shapes'] if e.get('finger')==finger and e.get('owner')=='pad');cooked_pad_results[finger]=minimum(shape(pad_entry),targets)
 cooked=shape(active);authored=shape(active,False);cooked_result=minimum(cooked,targets);authored_result=minimum(authored,targets);post_cooked=minimum(shape(active,post=True),posttargets)
 def offsets(e):
  native=np.asarray(e.get('runtime_body_shapes',{}).get('contact_offsets',[])).ravel();native=native[np.isfinite(native)&(native>=0)];unique=np.unique(native)
  return {'contactOffset_authored_or_schema':e['offsets']['contactOffset'],'restOffset_authored_or_schema':e['offsets']['restOffset'],'native_body_unique_contact_offsets_m':unique.tolist(),'native_body_unique_rest_offsets_m':np.unique(e.get('runtime_body_shapes',{}).get('rest_offsets',[])).tolist(),'exact_shape_offset_identified':bool(len(unique)==1),'actual_contact_offset_m':float(unique[0]) if len(unique)==1 else None}
 offA=offsets(active);offB=offsets(target_entry);shell_range=[offA['actual_contact_offset_m']+x for x in offB['native_body_unique_contact_offsets_m']] if offA['actual_contact_offset_m'] is not None else None;shell=(offA['actual_contact_offset_m']+offB['actual_contact_offset_m']) if offA['actual_contact_offset_m'] is not None and offB['actual_contact_offset_m'] is not None else None
 own=raw_results[contact['finger']]
 if own['intersects_proxy_solid']:classification='A_OFFICIAL_NONPAD_SURFACE_INTERSECTS_PROXY'
 elif not cooked_result['intersects_proxy_solid'] and not post_cooked['intersects_proxy_solid']:classification='B_POSITIVE_GEOMETRIC_GAP_WITH_NATIVE_RESPONSE'
 else:classification='COOKING_OR_INTEGRATION_CONTACT_WITH_OFFICIAL_RAW_GAP: not a pure contactOffset-only conclusion'
 history=[(i,c) for i,st in enumerate(steps[:index+1]) for c in st['contacts'] if c['collider']==contact['collider'] and c['target']==contact['target']]
 def local_bounds(e,cooked):
  m=trimesh.util.concatenate(meshes(e,cooked));m.apply_transform(np.linalg.inv(np.asarray(e['rigid_body_world_transform']))@np.asarray(e['world_transform']));return m.bounds.tolist()
 pad_entry=next(e for e in data['shapes'] if e.get('finger')==contact['finger'] and e.get('owner')=='pad')
 result={'first_nonpad_physics_index':index,'phase':step['phase'],'physics_time_s':step['t'],'physics_dt_s':step['physics_dt_s'],'pre_integration_q':step['q'],'native_contact':contact,'nonpad_official_raw_surface_to_handle':raw_results,'official_pad_surface_to_handle':pad_results,'active_nonpad_authored_collision_to_handle':authored_result,'active_nonpad_cooked_collision_to_handle_pre':cooked_result,'active_nonpad_cooked_collision_to_handle_post':post_cooked,'nonpad_offsets':offA,'handle_offsets':offB,'sum_actual_contact_offsets_m':shell,'classification':classification,'acceptance_modified':False,'metal_definition':'non-pad finger ownership, not a material claim','reference':'pre-integration robot poses and diagnostic native target pose; post pose also reported to bracket one timestep','contact_normal_reference':contact['normal_reference'],'raw_surface_scope':'original official STL minus official pad faces; distance to the unchanged semantic proxy solid','native_impulse_world_ns':contact['impulse_world_ns'],'impulse_norm_ns':float(np.linalg.norm(contact['impulse_world_ns'])),'reported_force_n':contact['force_n'],'reported_separation_m':contact['separation_m']}
 result.update(official_nonpad_post_distances=post_raw_results,official_pad_post_distances=post_pad_results,cooked_pad_distances_pre=cooked_pad_results,sum_contact_offset_range_m=shell_range,active_nonpad_local_raw_bounds_m=local_bounds(active,False),active_nonpad_local_cooked_bounds_m=local_bounds(active,True),active_pad_local_raw_bounds_m=local_bounds(pad_entry,False),active_pad_local_cooked_bounds_m=local_bounds(pad_entry,True),proximity_history={'first_report_physics_index':history[0][0],'first_separation_m':history[0][1]['separation_m'],'first_force_n':history[0][1]['force_n'],'zero_impulse_substeps_before_first_impulse':len(set(i for i,c in history if c['force_n']==0))},response_assessment='representation-induced near-contact impulse with positive official raw gap; NOT proof of physical non-pad surface contact' if not own['intersects_proxy_solid'] else 'official non-pad surface contact with impulse',potential_or_representation_events=0 if own['intersects_proxy_solid'] else 1,confirmed_loaded_forbidden_events=1 if own['intersects_proxy_solid'] else 0,pure_contactOffset_only_explanation=False,diagnostic_state='paused before further closure; no new grasp search or control tuning')
 (a.trial/'first_nonpad_diagnostic.json').write_text(json.dumps(result,indent=2))
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from mpl_toolkits.mplot3d.art3d import Poly3DCollection
 inv=np.linalg.inv(F)
 def local(v):return (np.asarray(v)@inv[:3,:3].T+inv[:3,3])*1000
 def polygons(ax,s,color,alpha):
  # Use exact FCL input vertices/faces from their original mesh below.
  vertices=s.vertices
  from scipy.spatial import ConvexHull
  faces=ConvexHull(vertices).simplices;poly=Poly3DCollection(local(vertices)[faces],facecolor=color,edgecolor=color,alpha=alpha,linewidth=.3);ax.add_collection3d(poly)
 fig=plt.figure(figsize=(14,7));cpoint=local(point[None])[0];nlocal=inv[:3,:3]@normal
 for pos,zoom in [(121,False),(122,True)]:
  ax=fig.add_subplot(pos,projection='3d')
  for e in data['shapes']:
   if e.get('owner')=='pad':polygons(ax,shape(e),'green',.35)
  # Official non-pad finger surface context; active native collider in bright red.
  for finger,spec in ownership['fingers'].items():
   mesh=trimesh.load(ROOT/'third_party/agilex_piper/description/meshes'/f'{finger}.stl',force='mesh');mesh.update_faces(np.array([i not in set(spec['official_pad_face_ids']) for i in range(len(mesh.faces))]));mesh.apply_transform(np.asarray(step['finger_world_poses'][finger]));ax.add_collection3d(Poly3DCollection(local(mesh.vertices)[mesh.faces],facecolor='red',edgecolor='red',alpha=.08,linewidth=.1))
  polygons(ax,cooked,'red',.8)
  for t in targets:polygons(ax,t,'royalblue',.18)
  ax.scatter(*cpoint,c='black',marker='*',s=90);ax.quiver(*cpoint,*nlocal,length=6,color='black',linewidth=2)
  bounds=local(np.vstack([x.vertices for x in targets]+[cooked.vertices]));lo,hi=(cpoint-12,cpoint+12) if zoom else (bounds.min(0)-12,bounds.max(0)+12)
  ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),zlim=(lo[2],hi[2]),xlabel='finger-local X (mm)',ylabel='finger-local Y (mm)',zlabel='finger-local Z (mm)',title='Contact detail' if zoom else 'Semantic handle / official finger context');ax.set_box_aspect(hi-lo);ax.view_init(elev=22,azim=140)
 fig.suptitle('First native non-pad impulse | green: pad collider | red: non-pad | blue: handle | black: contact / normal')
 fig.text(.05,.02,f"Official non-pad gap: {own['surface_to_proxy_solid_distance_m']*1000:.6f} mm; native separation: {contact['separation_m']*1000:.6f} mm; force: {contact['force_n']:.6f} N\n{classification}",fontsize=10);fig.tight_layout(rect=[0,.08,1,.94]);fig.savefig(a.trial/'first_nonpad_geometry.png',dpi=180);plt.close(fig)
 # Actual contact-plane slice, exposing sub-mm cooking protrusion.
 from scipy.spatial import ConvexHull
 def slice_polygon(s):
  vertices=local(s.vertices);faces=ConvexHull(vertices).simplices;edges=set()
  for face in faces:
   for i,j in zip(face,np.roll(face,-1)):edges.add(tuple(sorted((int(i),int(j)))))
  points=[];level=cpoint[1]
  for i,j in edges:
   a0,b0=vertices[i],vertices[j];da,db=a0[1]-level,b0[1]-level
   if abs(da)<1e-10:points.append(a0[[0,2]])
   if da*db<0:points.append((a0+(b0-a0)*da/(da-db))[[0,2]])
  if len(points)<3:return None
  points=np.unique(np.asarray(points),axis=0)
  if len(points)<3:return None
  return points[ConvexHull(points).vertices]
 fig,ax=plt.subplots(figsize=(10,6))
 for source,color,label,alpha,style in [(shape(pad_entry),'green','Cooked pad collider',.2,'-'),(authored,'red','Authored non-pad collider',0.,'--'),(cooked,'red','Cooked non-pad collider',.35,'-')]+[(t,'royalblue','Handle proxy',.2,'-') for t in targets]:
  poly=slice_polygon(source)
  if poly is None:continue
  ax.fill(poly[:,0],poly[:,1],color=color,alpha=alpha);closed=np.vstack((poly,poly[0]));ax.plot(closed[:,0],closed[:,1],color=color,linestyle=style,label=label)
 ax.scatter(cpoint[0],cpoint[2],c='black',marker='*',s=100,label='PhysX contact point');ax.arrow(cpoint[0],cpoint[2],.15*nlocal[0],.15*nlocal[2],head_width=.025,color='black',length_includes_head=True)
 ax.set(xlim=(cpoint[0]-1,cpoint[0]+1),ylim=(cpoint[2]-.4,cpoint[2]+.6),xlabel='finger-local X (mm)',ylabel='finger-local Z (mm)',title='True contact-plane slice: raw non-pad vs cooked protrusion');ax.grid(alpha=.2);handles,labels=ax.get_legend_handles_labels();unique=dict(zip(labels,handles));ax.legend(unique.values(),unique.keys(),fontsize=9);fig.tight_layout();fig.savefig(a.trial/'first_nonpad_cross_section.png',dpi=200);plt.close(fig)
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
