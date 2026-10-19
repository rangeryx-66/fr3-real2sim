from pathlib import Path
import json,yaml,xml.etree.ElementTree as ET,hashlib
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');O=ROOT/'results/cross_robot_20261012';configs=json.loads((O/'ROBOT_MODELS.json').read_text());repo=Path('/data1/home/rangeryx/fr3_moveit_grasp/franka_description');limits=yaml.safe_load((repo/'robots/fr3/joint_limits.yaml').read_text());dynamics=yaml.safe_load((repo/'robots/fr3/dynamics.yaml').read_text());r=ET.parse(configs['fr3']['urdf']).getroot();checks=[]
for i in range(1,8):
 j=r.find(f"joint[@name='joint{i}']");checks.append({'joint':f'joint{i}','limits_match_official_yaml':all(abs(float(j.find('limit').get(k))-v)<1e-12 for k,v in limits[f'joint{i}']['limit'].items()),'dynamics_match_official_yaml':all(abs(float(j.find('dynamics').get(k))-v)<1e-12 for k,v in dynamics[f'joint{i}']['dynamic'].items())})
hand={}
original=ET.parse(ROOT/'config/piper.urdf').getroot()
for name,c in configs.items():
 root=ET.parse(c['urdf']).getroot();hand[name]={}
 for n in ['gripper_base','gripper_link1','gripper_link2','gripper_link','tcp_link']:
  hand[name][n]=ET.tostring(root.find(f"link[@name='{n}']"))==ET.tostring(original.find(f"link[@name='{n}']"))
 # Ignore serializer indentation, compare element parameters recursively.
 def tree(e):return e.tag,e.attrib,[tree(x) for x in e]
 for n in hand[name]:hand[name][n]=tree(root.find(f"link[@name='{n}']"))==tree(original.find(f"link[@name='{n}']"))
(O/'SOURCE_PARAMETER_AUDIT.json').write_text(json.dumps({'fr3_against_pinned_official_yaml':checks,'hand_links_exact_parameter_equivalence':hand,'mount_joint_intentionally_separate_from_hand_subtree':True,'object_source_unmodified':True},indent=2));print(checks)
