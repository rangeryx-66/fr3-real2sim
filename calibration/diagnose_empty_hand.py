"""Read-only experiment on the development scene: verify joint state consistency.

No formal trial or frozen parameter is changed. The second diagnostic condition
explicitly wakes the active hand solely to test a suspected stale velocity defect.
"""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
adapter=(ROOT/'calibration/sim_calibration.py').read_text()
end="exec(compile(text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
ns={'__file__':str(ROOT/'calibration/sim_calibration.py')}
exec(compile(adapter.replace(end,''),str(ROOT/'calibration/sim_calibration.py'),'exec'),ns)
prefix=ns['text'].split('commands=queue.Queue()')[0]
env=dict(__name__='__main__',__file__=str(ns['source']))
exec(compile(prefix,str(ns['source']),'exec'),env)
globals().update({k:v for k,v in env.items() if not k.startswith('__')})
import omni.physx
api={}
for name in ['get_physx_interface','get_physx_simulation_interface']:
    obj=getattr(omni.physx,name)()
    api[name]={x:str(getattr(obj,x).__doc__) for x in dir(obj) if any(s in x.lower() for s in ['wake','sleep'])}
report={'api':api,'conditions':[],'initial_sleep_threshold':float(robot.get_sleep_threshold())}
from pxr import PhysicsSchemaTools, Sdf
physics=omni.physx.get_physx_simulation_interface()
stage_id=omni.usd.get_context().get_stage_id()
body_id=PhysicsSchemaTools.sdfPathToInt(Sdf.Path(finger_paths[0]))
definition=json.loads((ROOT/'results/hand_calibration/gt_path_soup.json').read_text())
poses={'home':HOME,'development_grasp':definition['approach']['points'][-1]['q']}
box.set_world_pose([3.,3.,1.],[1.,0.,0.,0.])
out=ROOT/'results/unseen_diagnostics/empty_hand_sleep.json'
out.parent.mkdir(parents=True,exist_ok=True)
out.write_text(json.dumps(report,indent=2))
try:
    for pose,condition in [(p,c) for p in poses for c in ['default','wake_active_diagnostic']]:
        calib.position()
        q[arm]=poses[pose]
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros(len(names)))
        for _ in range(240):
            robot.apply_action(ArticulationAction(joint_positions=q))
            world.step(render=False)
        kp,kd=[np.array(x,copy=True) for x in robot.get_articulation_controller().get_gains()]
        kp[arm]*=2.;kd[arm]*=np.sqrt(2.)
        robot.get_articulation_controller().set_gains(kps=kp,kds=kd,save_to_usd=False)
        calib.close_force(30.)
        rows=[]
        for tick in range(6*240):
            sleeping=physics.is_sleeping(stage_id,body_id)
            if condition=='wake_active_diagnostic':physics.wake_up(stage_id,body_id)
            robot.apply_action(ArticulationAction(joint_positions=q))
            calib.step()
            world.step(render=False)
            rows.append(dict(t=(tick+1)*DT,sleeping=bool(sleeping),q=robot.get_joint_positions()[fingers].tolist(),
                qd=robot.get_joint_velocities()[fingers].tolist(),control=calib.state()))
        entry={'pose':pose,'condition':condition,'rows':rows}
        report['conditions'].append(entry)
        out.write_text(json.dumps(report,indent=2))
        print('EMPTY_HAND_RESULT',condition,rows[-1],flush=True)
    out=ROOT/'results/unseen_diagnostics/empty_hand_sleep.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2))
    print('DIAGNOSTIC_SAVED',out,flush=True)
except Exception:
    import traceback
    report['exception']=traceback.format_exc()
    out.write_text(json.dumps(report,indent=2))
    print(report['exception'],flush=True)
finally:app.close()
