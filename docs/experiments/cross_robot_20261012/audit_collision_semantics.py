"""Record manufacturer SRDF pairs as a diagnostic; do not disable colliders."""
from pathlib import Path
import json,xml.etree.ElementTree as ET,hashlib,sys,subprocess
O=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/cross_robot_20261012');S=Path('/data1/home/rangeryx/cross_robot_sources_20261012');sys.path.insert(0,str(S/'python_deps'));import xacro
files={'nero':Path('/data1/home/rangeryx/piper_official_src/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.srdf.xacro'),'xarm7':S/'xarm_ros/xarm7_moveit_config/config/xarm7.srdf'};res={}
for r,p in files.items():
 root=ET.fromstring(xacro.process_file(str(p),mappings={'arm_type':'nero','effector_type':'none'}).toxml()) if r=='nero' else ET.parse(p).getroot();pairs=[x.attrib for x in root.findall('disable_collisions')];res[r]={'source':str(p),'source_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'vendor_pairs':pairs,'applied_to_current_study':False,'source_status':'manufacturer MoveIt planning mask; not an automatically safe exemption for the changed same-gripper convex-hull composition','interpretation':'failures involving these pairs are planner/collision-semantic incompatibility, not evidence that the physical arm cannot move'}
(O/'COLLISION_SEMANTICS_DIAGNOSTIC.json').write_text(json.dumps(res,indent=2));print({k:len(v['vendor_pairs']) for k,v in res.items()})
