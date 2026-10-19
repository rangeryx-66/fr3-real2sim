"""Real PiPER finger closure and passive-door interaction; no ideal attachment."""
import argparse,json,os,sys,hashlib,subprocess,shutil,tempfile
from pathlib import Path
from datetime import datetime,timedelta
from zoneinfo import ZoneInfo
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def matrix(p,q):
    T=np.eye(4);T[:3,3]=p;T[:3,:3]=Rotation.from_quat(np.roll(q,-1)).as_matrix();return T


def bootstrap(a,base):
    # Only substitute robot-specific code in a guarded copy of the common loader.
    # Source file and all articulated-object definitions remain untouched.
    legacy=ROOT/'src/r1a7_articulated_sim_server.py';text=legacy.read_text();marker='commands = queue.Queue(); state = {}; lock = threading.Lock()'
    assert text.count(marker)==1;source=text.partition(marker)[0]
    substitutions=[("scripts/prepare_r1a7_description.py","scripts/prepare_piper_description.py"),
        ('nargs=7','nargs=6'),('default=(0.,1.3,1.,-1.3,0.,0.,0.)','default=(0.,1.15,-1.35,0.,.2,0.)'),
        ('config/r1a7_dex1.urdf','config/piper_sim.urdf'),('/World/R1A7','/World/Piper'),
        ('r1a7_dex1_filtered_asset_path_','piper_mobile_asset_path_'),
        ('dex1_Link1_3','gripper_link1'),('dex1_Link2_3','gripper_link2'),
        ('dex1_Link1_2','gripper_base'),('dex1_Link2_2','link6'),
        ('r1a7_tcp','tcp_link'),("names.index(f'J{i}') for i in range(1, 8)","names.index(f'joint{i}') for i in range(1, 7)"),
        ('dex1_Joint1_1','gripper_joint1'),('dex1_Joint2_1','gripper_joint2'),
        ('kp[arm] = [500., 500., 400., 400., 250., 150., 150.]','kp[arm] = 10000.'),
        ('kd[arm] = [40., 40., 32., 32., 22., 15., 15.]','kd[arm] = 400.'),
        ('kp[fingers] = 800.; kd[fingers] = 30.','kp[fingers] = 1000.; kd[fingers] = 40.'),
        ('q[fingers] = -.02','q[fingers] = [.05,-.05]'),
        ('fix_base=True, allow_self_collision=False, merge_fixed_joints=False,\n        joint_drive_type','fix_base=True, allow_self_collision=True, merge_fixed_joints=False,\n        joint_drive_type')]
    for old,new in substitutions:
        if old not in source:raise RuntimeError('scene loader changed: '+old)
        source=source.replace(old,new)
    source=source.replace("if prim.GetName() == name and str(prim.GetPath()).startswith(under)]", "if prim.GetName() == name and str(prim.GetPath()).startswith(under)\n               and (prim.HasAPI(UsdPhysics.RigidBodyAPI) or name == 'tcp_link')]")
    source=source.replace('save_to_usd=False','save_to_usd=True')
    # Fold the massless virtual opening command into the first real finger.
    # Preserve the official q2=-q1 mechanical coupling, not independent jaws.
    subprocess.run([sys.executable,str(ROOT/'scripts/prepare_piper_description.py')],check=True)
    robot=ET.parse(ROOT/'config/piper.urdf').getroot()
    for item in list(robot):
        if (item.tag=='link' and item.get('name')=='gripper_link') or (item.tag=='joint' and item.get('name')=='gripper'):robot.remove(item)
    for joint in robot.findall('joint'):
        for mimic in joint.findall('mimic'):joint.remove(mimic)
        if joint.get('name')=='gripper_joint2':ET.SubElement(joint,'mimic',joint='gripper_joint1',multiplier='-1.0',offset='0.0')
    # Preserve identical bytes while publishing the shared importer proxy atomically.
    with tempfile.NamedTemporaryFile(dir=ROOT/'config',suffix='.urdf',delete=False) as f:
        temporary=Path(f.name)
    try:
        ET.ElementTree(robot).write(temporary,encoding='unicode');os.chmod(temporary,0o644);os.replace(temporary,ROOT/'config/piper_sim.urdf')
    finally:
        temporary.unlink(missing_ok=True)
    # Importer proxy is prepared by us; original command URDF is preserved.
    source=source.replace("subprocess.run([sys.executable,str(ROOT/'scripts/prepare_piper_description.py')],check=True)","pass # already prepared")
    source=source.replace('usd_path=str(asset_cache)', 'usd_path=str(asset_cache / model_hash)')
    monitors="""scene_monitor_views=[]
for monitor_name in ['link1','link2','link3','link4','link5','link6','flange_link','gripper_base','gripper_link1','gripper_link2']:
    filters=list(diagnostic_contact_paths)
    if monitor_name not in ['gripper_link1','gripper_link2']:filters.append(contact_target_path)
    scene_monitor_views.append((monitor_name,world.scene.add(RigidPrim(
        prim_paths_expr=one_prim(monitor_name,'/World/Piper'),name='scene_contact_'+monitor_name,
        contact_filter_prim_paths_expr=filters,track_contact_forces=True,
        prepare_contact_sensors=True,max_contact_count=256))))
"""
    source=source.replace("tcp = SingleXFormPrim(one_prim('tcp_link', '/World/Piper'))",monitors+"tcp = SingleXFormPrim(one_prim('tcp_link', '/World/Piper'))")
    source=source.replace("robot = world.scene.add(SingleArticulation('/World/Piper', name='r1a7'))","robot = world.scene.add(SingleArticulation('/World/Piper', name='piper'))\n    robot.set_world_pose(BASE_POSE[:3], np.roll(Rotation.from_euler('z',BASE_POSE[3],degrees=True).as_quat(),1))")
    os.environ['R1A7_BASE_POSE']=','.join(map(str,base));os.environ['R1A7_SUPPORT_BOTTOM_Z']='-.56';os.environ['R1A7_PEDESTAL_SIZE']='.10,.10,.46'
    os.environ['R1A7_RUN_DIR']=str(a.output);os.environ['OMNI_KIT_ACCEPT_EULA']='YES';os.environ['ACCEPT_EULA']='Y'
    s=json.loads((a.source/'report.json').read_text());place=s['asset_installation'];old=sys.argv
    sys.argv=[str(legacy),'--gpu',str(a.gpu),'--asset-root',str(a.asset_root),'--asset-x',str(place['x_m']),'--asset-y',str(place['y_m']),'--asset-yaw-deg',str(place['yaw_deg']),'--fixture-height-m',str(place['fixture_height_m']),'--record-overview','--enable-isolation-diagnostics']
    if getattr(a,'ownership',None):
        from piper_mobile_demo.contact_ownership import install_owned_colliders
        scene_install="from piper_mobile_demo.contact_ownership import install_owned_colliders\nowned_contact_colliders=install_owned_colliders(stage,json.loads(Path("+repr(str(a.ownership))+").read_text()))\n"
        if source.count('world.reset(); camera.initialize()')!=1:raise RuntimeError('scene reset marker changed')
        source=source.replace('world.reset(); camera.initialize()',scene_install+'world.reset(); camera.initialize()')
    scene={'__name__':'piper_mobile_scene','__file__':str(legacy)}
    exec(compile(source,str(legacy),'exec'),scene);sys.argv=old;scene['bootstrap_hash']=hashlib.sha256(text.encode()).hexdigest();return scene


def main():
    # Retire the whole-finger ±1 mm classifier. This entry now accepts the
    # canonical-gated owned-contact test arguments and never opens the door.
    from piper_owned_formal_contact_test import main as owned_main
    owned_main()

if __name__=='__main__':main()
