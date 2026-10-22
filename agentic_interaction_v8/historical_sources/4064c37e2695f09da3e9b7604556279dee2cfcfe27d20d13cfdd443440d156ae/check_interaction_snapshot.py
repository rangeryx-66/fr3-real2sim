"""Deep-penetration guard from the current native collision snapshot.

No object joint type, axis, origin, limits or joint angle is read. This is a
simulation collision audit, not a perception sensor or a trajectory generator.
"""
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from articulated_interaction.collision import InteractionScene

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for n in ['cooked','association','calibration','output']:p.add_argument('--'+n,type=Path,required=True)
 a=p.parse_args();data=json.loads(a.cooked.read_text());cal=json.loads(a.calibration.read_text());allowed=json.loads(a.association.read_text())['allowed_pad_targets']
 owner=InteractionScene(ROOT,a.cooked,ROOT/'config/piper_contact_ownership.json',allowed,'native_target',calibration=cal)
 poses={}
 for e in data['shapes']:
  if e['path'].startswith('/World/Piper') and e.get('rigid_body_world_transform'):
   poses[e['rigid_body_path'].split('/')[-1]]=np.asarray(e['rigid_body_world_transform'])
 permitted=set()
 for j in ET.parse(ROOT/'config/piper.urdf').getroot().findall('joint'):
  permitted.add(frozenset((j.find('parent').get('link'),j.find('child').get('link'))))
 for c in ET.parse(ROOT/'config/piper.srdf').getroot().findall('disable_collisions'):
  permitted.add(frozenset((c.get('link1'),c.get('link2'))))
 ok,why=owner.check(poses,allow_pad=True)
 if ok:ok,why=owner.check_self(poses,permitted)
 result={'passed':ok,'reason':why,'native_snapshot':str(a.cooked),'raw_triangle_intersection_alone_is_failure':False,'GT_joint_information_read':False,'scope':'current exported native collision geometry and unchanged official robot mesh'}
 a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result));sys.exit(0 if ok else 1)
if __name__=='__main__':main()
