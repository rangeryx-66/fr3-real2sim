"""Isolated calibration extension; original simulator and bridge untouched."""
import sys,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
source=ROOT/'src/sim_server.py';text=source.read_text()
old="world.scene.add(FixedCuboid('/World/table',name='table',position=[.5,0,-.025],scale=[.7,.7,.05],physics_material=mat))\nbox=world.scene.add(DynamicCuboid('/World/box',name='box',position=BOX,scale=SIZE,mass=.06,color=np.array([.8,.12,.08]),physics_material=mat))"
text=text.replace(old,"import calibration_scene\nbox=calibration_scene.table(world,omni.usd.get_context().get_stage(),mat)")
text=text.replace('z_position=-.06','z_position=-.76')
text=text.replace("                if op=='reset':", "                if op=='reset':\n                    results.clear()")
text=text.replace("            elif self.path=='/control':", "            elif self.path=='/calibration':payload={k:v for k,v in payload.items() if k in ['t','names','q','box','box_quat','tcp','tcp_quat','forces','calibration','busy']}\n            elif self.path=='/control':")
text=text.replace("clutter=ClutterMonitor(world,stage,mat) if a.clutter else None","clutter=None")
text=text.replace('world.reset()',"from hand_force_control import configure_stage_materials\nconfigure_stage_materials(stage,[x+'/box_3' for x in finger_paths],float(__import__('os').environ.get('CALIBRATION_MU','.7')))\nworld.reset()")
text=text.replace("q=np.zeros(len(names));q[arm]=HOME;q[fingers]=.04","from hand_force_control import HandCalibration\ncalib=HandCalibration(robot,contacts,stage,fingers,finger_paths,DT)\nq=np.zeros(len(names));q[arm]=HOME;q[fingers]=.04")
text=text.replace("box.set_world_pose(BOX,[1,0,0,0]);box.set_linear_velocity([0,0,0]);box.set_angular_velocity([0,0,0])","calib.position();calibration_scene.reset(box)")
hook="                elif op=='clutter_enabled':"
text=text.replace(hook,"""                elif op=='calibration_set_object_pose':
                    if active: raise RuntimeError('trajectory active')
                    position=np.asarray(cmd['position'],dtype=float).reshape(3)
                    quaternion=np.asarray(cmd.get('quat_wxyz',[1.,0.,0.,0.]),dtype=float).reshape(4)
                    if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
                        raise ValueError('invalid object pose')
                    box.set_world_pose(position,quaternion)
                    box.set_linear_velocity([0.,0.,0.]);box.set_angular_velocity([0.,0.,0.])
                    results[token]={'ok':True,'position':position.tolist(),'quat_wxyz':quaternion.tolist()}
                elif op=='calibration_material':
                    results[token]={'ok':True,'original':calib.original_materials,'configured':calib.configure_material(float(cmd['mu']))}
                elif op=='calibration_audit':
                    results[token]={'ok':True,'material':calib.material_audit()}
                elif op=='calibration_gate_start':
                    calib.gate_start();results[token]={'ok':True}
                elif op=='calibration_gate_finish':
                    results[token]={'ok':True,'gate':calib.gate_finish()}
                elif op=='calibration_stability':
                    from settling_gate import evaluate,GateThresholds
                    rows=[x for x in trace.records if x['phase']=='MICRO_LIFT' and x['t']>=float(cmd.get('since_t',-1e30))]
                    thresholds=GateThresholds(**cmd.get('thresholds',{}))
                    results[token]={'ok':True,'stability':evaluate(rows,thresholds=thresholds,old_gate=cmd.get('old_gate'))}
                elif op=='calibration_force':
                    calib.close_force(float(cmd['force_N']));results[token]={'ok':True}
                elif op=='calibration_position':
                    calib.position();results[token]={'ok':True}
                elif op=='calibration_phase':
                    calib.phase=cmd['phase'];results[token]={'ok':True}
                elif op=='calibration_set_joints':
                    target[arm]=cmd['q'];robot.set_joint_positions(target);robot.set_joint_velocities(np.zeros(len(names)));results[token]={'ok':True}
"""+hook)
text=text.replace('world.step(render=tick%(24 if clutter else 8)==0);tick+=1','calib.step()\n        world.step(render=False);tick+=1')
text=text.replace("clutter.phase if clutter else 'IDLE'","calib.phase")
text=text.replace('        if trace.path is not None:\n', '        calib.observe(bp,bq,tp,tq,f)\n        if trace.path is not None:\n')
text=text.replace('        sample=dict(t=tick*DT',"        if trace.path is not None:trace.records[-1]['hand_calibration']=calib.state()\n        sample=dict(t=tick*DT")
text=text.replace("clutter=clutter_state)","clutter=clutter_state,calibration=calib.state(),target_class=calibration_scene.TARGET)")
exec(compile(text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))
