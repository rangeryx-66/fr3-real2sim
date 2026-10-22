"""Robot-only FK equivalence/timing; no scene, grasp, dynamics or simulator.

Extract the two exact frozen poses() definitions for an isolated numeric test.
This avoids importing a PhysX scene just to compare URDF matrix products. It is
not an IK, collision or physical-contact certificate.
"""
import argparse,ast,hashlib,json,sys,time
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def check(config_path,seed=20261023,samples=128):
    import numpy as np
    from articulated_demo.kinematics import URDFChain,transform
    from xarm7_native_gripper import NativeGripperGeometry,GRIPPER_JOINTS,FINGER_BODIES
    c=json.loads(Path(config_path).read_text());chain=URDFChain(c['sim_urdf']);native=NativeGripperGeometry(c)
    source=ROOT/'scripts/xarm7_native_planning.py';fast=ROOT/'interaction_runtime/kinematics.py'
    tree=ast.parse(source.read_text());ns=dict(np=np,transform=transform,GRIPPER_JOINTS=GRIPPER_JOINTS,FINGER_BODIES=FINGER_BODIES)
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('LOGICAL','PAD_FRAMES') for t in node.targets):
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),ns)
    def function(path,name):
        cls=next(x for x in ast.parse(path.read_text()).body if isinstance(x,ast.ClassDef) and x.name==name)
        fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='poses')
        scope=dict(ns);exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),scope)
        return scope['poses']
    original=function(source,'NativeModel');cached=function(fast,'FastNativeModel')
    robot=ET.parse(c['sim_urdf']).getroot()
    model=SimpleNamespace(native=native,chain=chain,names=c['arm_joint_names'],
        physical_hulls={x.get('name'):None for x in robot.findall('link') if x.find('collision') is not None})
    rng=np.random.default_rng(seed);q=[]
    for name in model.names:
        limit=robot.find("joint[@name='%s']/limit"%name)
        q.append(rng.uniform(float(limit.get('lower'))+.051,float(limit.get('upper'))-.051,samples))
    q=np.asarray(q).T;fingers=rng.uniform(.02,.82,(samples,6));bases=np.c_[rng.uniform(-.4,.8,(samples,2)),np.full(samples,-.1),rng.uniform(-180,180,samples)]
    widths=rng.uniform(.002,.084,samples);maximum=0.
    for i in range(samples):
        for actual_fingers in [None,fingers[i]]:
            a=original(model,q[i],bases[i],width=float(widths[i]),finger_q=actual_fingers)
            b=cached(model,q[i],bases[i],width=float(widths[i]),finger_q=actual_fingers)
            assert set(a)==set(b)
            maximum=max(maximum,max(float(np.max(abs(a[k]-b[k]))) for k in a))
    if maximum>1e-13:raise AssertionError('FK numerical inequivalence:'+str(maximum))
    timings=[]
    for fn in [original,cached]:
        t=time.perf_counter()
        for _ in range(4):
            for i in range(samples):fn(model,q[i],bases[i],width=float(widths[i]),finger_q=fingers[i])
        timings.append(time.perf_counter()-t)
    return dict(scope='robot URDF link/pad FK only; no physics/IK/collision claim',samples=samples,
        modes=['common aperture','measured coupled fingers'],maximum_absolute_transform_difference=maximum,
        original_s=timings[0],cached_s=timings[1],speed_ratio=timings[0]/timings[1],
        source_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [source,fast]},
        robot_urdf_sha256=hashlib.sha256(Path(c['sim_urdf']).read_bytes()).hexdigest(),
        randomized_robot_states_seed=seed,hidden_object_dynamics_used=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--robot-config',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);a=p.parse_args();r=check(a.robot_config)
    a.output.write_text(json.dumps(r,indent=2));print(json.dumps(r))
