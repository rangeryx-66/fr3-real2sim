"""Read-only, post-step telemetry overlay; no dynamics/controller changes."""
from pathlib import Path
R=Path(__file__).resolve().parents[1];p=R/'calibration/sim_real2sim.py'
s=p.read_text();last="exec(compile('import os\\nfrom scipy.spatial.transform import Rotation\\n'+text,str(source),'exec'),dict(__name__='__main__',__file__=str(source)))"
assert s.count(last)==1
ns={'__file__':str(p)};exec(compile(s.replace(last,''),str(p),'exec'),ns);text=ns['text'];source=ns['source']
init="print('REAL2SIM_JACOBIAN_BODY',hand_body_index,body_names,flush=True)"
text=text.replace(init,init+'''
import inspect as _dq_inspect
_dq_view=getattr(articulation_view,'_physics_view',None)
_dq_rows=[];_dq_active=False;_dq_previous=None
_dq_sources={'reported':'robot.get_joint_velocities()', 'actual_q':'robot.get_joint_positions()', 'direct_velocity':'robot._articulation_view._physics_view.get_dof_velocities()', 'direct_q':'robot._articulation_view._physics_view.get_dof_positions()', 'target_velocity':'_physics_view.get_dof_velocity_targets()', 'controller_action':'robot.get_articulation_controller().get_applied_action()', 'sample_phase':'after world.step; same callback and simulation step', 'joint_names':names, 'arm_indices':list(arm), 'ROS_velocity':'not published: ros_bridge.py publishes JointState.position only'}
_dq_sources['types']={'robot':str(type(robot)),'articulation':str(type(articulation_view)),'physics':str(type(_dq_view))}
_dq_sources['code']={}
for _name,_fn in [('reported',robot.get_joint_velocities),('view',articulation_view.get_joint_velocities)]:
    try:_dq_sources['code'][_name]=_dq_inspect.getsource(_fn)
    except Exception as exc:_dq_sources['code'][_name]=str(exc)
''')
hook="                elif op=='payload_record_start':"
commands='''                elif op=='dq_audit_start':
                    _dq_rows=[];_dq_previous=None;_dq_active=True
                    results[token]={'ok':True,'step':tick}
                elif op=='dq_audit_stop':
                    _dq_active=False;_dq_out=Path(cmd['path']);_dq_out.parent.mkdir(parents=True,exist_ok=True)
                    if not _dq_rows:raise RuntimeError('NO_DQ_AUDIT_SAMPLES')
                    np.savez_compressed(_dq_out,**{k:np.asarray([row[k] for row in _dq_rows]) for k in _dq_rows[0]})
                    _dq_out.with_suffix('.sources.json').write_text(json.dumps(_dq_sources,indent=2))
                    results[token]={'ok':True,'samples':len(_dq_rows),'path':str(_dq_out)}
'''+hook
assert text.count(hook)==1;text=text.replace(hook,commands)
hook="        sample=dict(t=tick*DT"
record='''        if _dq_active:
            _qa=np.array(robot.get_joint_positions(),copy=True);_qr=np.array(robot.get_joint_velocities(),copy=True)
            _qd=np.array(_dq_view.get_dof_positions(),copy=True).reshape(-1)
            _vd=np.array(_dq_view.get_dof_velocities(),copy=True).reshape(-1)
            _vt=np.array(_dq_view.get_dof_velocity_targets(),copy=True).reshape(-1)
            _action=robot.get_articulation_controller().get_applied_action()
            _cmdv=getattr(_action,'joint_velocities',None)
            _cmdv=np.full(len(names),np.nan) if _cmdv is None else np.array(_cmdv,copy=True)
            _fd=np.full(len(names),np.nan) if _dq_previous is None else (_qa-_dq_previous[1])/((tick-_dq_previous[0])*DT)
            _tau=np.array(robot.get_measured_joint_efforts(),copy=True)
            _dq_rows.append({'step':tick,'t':tick*DT,'world_time':float(world.current_time),'world_step':int(world.current_time_step_index),'q_actual':_qa[arm],'dq_reported':_qr[arm],'q_physx':_qd[arm],'dq_physx':_vd[arm],'dq_fd':_fd[arm],'dq_target':_vt[arm],'controller_velocity':_cmdv[arm], 'q_command':target[arm].copy(),'tau':_tau[arm],'tcp_position':np.array(tp,copy=True),'tcp_quat_wxyz':np.array(tq,copy=True),'root_velocity':np.array(_dq_view.get_root_velocities(),copy=True).reshape(-1)})
            _dq_previous=(tick,_qa.copy())
'''+hook
assert text.count(hook)==1;text=text.replace(hook,record)
exec(compile('import os\nfrom scipy.spatial.transform import Rotation\n'+text,str(source),'exec'),{'__file__':str(source),'__name__':'__main__'})
