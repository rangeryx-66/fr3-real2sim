from pathlib import Path
import json,hashlib,xml.etree.ElementTree as ET
import numpy as np,fcl
from piper_mobile_demo.model import geometry,fcl_geometry,obj
from articulated_demo.kinematics import URDFChain
R=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=R/'results/nero_actuator_stability_20261012';P=R/'results/nero_feasibility_20261012';c=json.loads((P/'NERO_MODEL.json').read_text());u=Path(c['sim_urdf']);root=ET.parse(u).getroot();chain=URDFChain(u);meshes={n:geometry(root.find(f"link[@name='{n}']/collision"),u.parent) for n in ['link6','flange_link']};samples=[('initial',c['home_q'])]
for asset in ['45134','45443','45176','45415']:
 plan=json.loads((P/'offline'/asset/'candidate.json').read_text());samples += [(asset+'_path_'+str(i),row['q']) for i,row in enumerate(plan['path'])]
rows=[]
for name,q in samples:
 A,B=[obj(fcl_geometry(meshes[n]),chain.root_to_link(n,dict(zip(c['arm_joint_names'],q)))) for n in ['link6','flange_link']];co=fcl.CollisionResult();fcl.collide(A,B,fcl.CollisionRequest(num_max_contacts=100,enable_contact=True),co);d=fcl.DistanceResult();gap=fcl.distance(A,B,fcl.DistanceRequest(enable_nearest_points=True),d)
 rows.append({'source':name,'q':q,'raw_triangle_contact_count':len(co.contacts),'exact_raw_surface_gap_m':float(gap) if gap>=0 else None})
r={'pair':['link6','flange_link'],'urdf_sha256':hashlib.sha256(u.read_bytes()).hexdigest(),'official_connection_chain':[{'name':j.get('name'),'type':j.get('type'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link')} for j in root.findall('joint') if j.find('child').get('link') in ['link7','flange_link']], 'records':rows,'all_raw_meshes_clear':all(x['raw_triangle_contact_count']==0 and x['exact_raw_surface_gap_m'] is not None and x['exact_raw_surface_gap_m']>0 for x in rows),'minimum_surface_gap_m':min((x['exact_raw_surface_gap_m'] for x in rows if x['exact_raw_surface_gap_m'] is not None),default=None),'native_observation':'run01 link6/flange convex overlap 37.59 mm; impulse 55.13 Ns; joint7 projected reaction 48.32 Nm with zero external torque'}
(O/'MEASURED_CONTACT_GEOMETRY_AUDIT.json').write_text(json.dumps(r,indent=2));print({k:v for k,v in r.items() if k!='records'})
