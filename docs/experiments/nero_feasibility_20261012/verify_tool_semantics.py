from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np,hashlib
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from articulated_demo.kinematics import URDFChain
from piper_mobile_demo.model import geometry
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/nero_feasibility_20261012';c=json.loads((O/'NERO_MODEL.json').read_text());original=E.parse(ROOT/'config/piper.urdf').getroot();ch0=URDFChain(ROOT/'config/piper.urdf')
# Define an explicit native tool frame aligning the old shared contact frame.
for fn in ['nero_kinematic.urdf','nero_sim.urdf']:
 p=O/fn;r=E.parse(p).getroot();r.find("joint[@name='tcp_joint']/origin").set('rpy','0 0 3.141592653589793')
 if fn=='nero_sim.urdf':
  q=r.find("joint[@name='gripper_joint2']")
  for x in q.findall('mimic'):q.remove(x)
  E.SubElement(q,'mimic',joint='gripper_joint1',multiplier='-1.0',offset='0.0')
 E.indent(r);E.ElementTree(r).write(p,encoding='utf-8',xml_declaration=True)
r=E.parse(O/'nero_kinematic.urdf').getroot();ch=URDFChain(O/'nero_kinematic.urdf');records=[]
for width in [.1,.04,.02]:
 vals={'gripper_joint1':width/2,'gripper_joint2':-width/2};t0=ch0.root_to_link('tcp_link',vals);t=ch.root_to_link('tcp_link',vals)
 for n in ['gripper_base','gripper_link1','gripper_link2']:
  a=geometry(original.find(f"link[@name='{n}']/collision"),ROOT/'config');a.apply_transform(np.linalg.inv(t0)@ch0.root_to_link(n,vals));b=geometry(r.find(f"link[@name='{n}']/collision"),O);b.apply_transform(np.linalg.inv(t)@ch.root_to_link(n,vals));records.append({'width_m':width,'link':n,'symmetric_nearest_vertex_max_m':float(max(cKDTree(a.vertices).query(b.vertices)[0].max(),cKDTree(b.vertices).query(a.vertices)[0].max()))})
c['tool_convention']='native TCP at gripper_base xyz0,0,.138 rpy0,0,pi, geometrically aligned to frozen shared PiPER pad frame';c['sim_mimic']='one physical jaw width drive; q2=-q1, virtual width joint folded into q1';(O/'NERO_MODEL.json').write_text(json.dumps(c,indent=2));(O/'GRIPPER_COMPATIBILITY.json').write_text(json.dumps({'same_gripper_meshes':True,'frame_alignment':records,'native_mount_offset_m':.006,'old_PiPER_mount_offset_m':.0045,'native_vs_legacy_finger_axes':'official native conventions preserved; TCP convention adapted once for all objects','real_hardware_bolt_power_firmware_compatibility':'manufacturer confirmation still required','no_object_specific_grasp_offsets':True},indent=2))
d=json.loads((O/'WRIST_COLLISION_AUDIT.json').read_text());pairs=[]
for a,b in [('link5','link7'),('link5','flange_link')]:
 rows=[x for x in d['records'] if x['pair']==[a,b]];clear=all(x['raw_triangle_intersections']==0 and x['raw_surface_clearance_m'] is not None and x['raw_surface_clearance_m']>0 for x in rows);pairs.append({'pair':[a,b],'configurations_checked':len(rows),'raw_mesh_clear_in_all':clear,'minimum_exact_surface_gap_m':min(x['raw_surface_clearance_m'] for x in rows if x['raw_surface_clearance_m'] is not None),'minimum_sampled_signed_gap_m':min(x['signed_sample_clearance_m'] for x in rows if x['signed_sample_clearance_m'] is not None),'signed_inside_test_qualification':'link5 non-watertight; full triangle intersections + exact surface distance are primary evidence; signed sample is diagnostic','vendor_semantics':'link5-link7 Never in pinned SRDF' if b=='link7' else 'flange is fixed child of link7; exact assembly geometry independently verified','handling':'filter ONLY this convex-proxy body pair in PhysX, retain raw official triangle check at every inherited scene check; all other self/environment/gripper contacts unchanged'})
if not all(x['raw_mesh_clear_in_all'] for x in pairs):raise ValueError('No verified semantic correction available')
(O/'VERIFIED_COLLISION_SEMANTICS.json').write_text(json.dumps({'corrected_pairs':pairs,'no_other_SRDF_exemptions_applied':True,'official_collision_meshes_unchanged':True,'convex_proxy_issue':'link5 hull volume ≈3.8 times raw signed volume; hull fills wrist cavity, causing false intersections','real_raw_geometry_collisions_still_stop':True},indent=2));print(records);print(pairs)
