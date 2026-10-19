"""One normally closed grasp/hold development trial; frozen scene/guards/export."""
from pathlib import Path
import sys,json,hashlib
from run_native_g1_closure_diagnostic import diagnostic_source

def contact_source(spec):
    text=diagnostic_source(dict(spec,closure_diagnostic_mode='shared_progressive'))
    anchor='initial=repair_initial_sequence(initial)'
    addition=r'''
initial=initial.replace("  closure=JawCenteredClosure(tcp(),policy);phase='CLOSE'", "  closure=JawCenteredClosure(tcp(),policy);phase='CLOSE'\n  closure_bias=qtarget[arm]-np.asarray(robot.get_joint_positions())[arm]\n  closure.response_period_s=max(dt,int(.08/dt)*dt)\n  diagnostic_write(a.output/'CONTACT_CORRECTION_START.json',json.dumps({'command_bias_rad':closure_bias.tolist(),'original_time_budget_s':100,'sensor_response_period_s':closure.response_period_s,'handle_pose_used_by_closure':False}))")
initial=initial.replace("   qvelocity=np.clip((q-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=q;step()", "   q=q+closure_bias # preserve acquired position-servo equilibrium, same technique as frozen opening\n   qvelocity=np.clip((q-qtarget[arm])/dt,-vel/3,vel/3);qtarget[arm]=q;step()")
'''
    text=text.replace(anchor,anchor+'\n'+addition)
    record="'active_update_source_sha256':provenance['active_closure_sha256']}"
    text=text.replace(record,"'active_update_source_sha256':provenance['active_closure_sha256'],'T_tcp_command':command['T'].tolist(),'T_tcp_measured':np.asarray(T).tolist(),'pad_centers_world':np.asarray(pad_centers).tolist(),'contact_correction':self.detail.copy()}")
    end="diagnostic_write(a.output/'report.json',json.dumps(result,indent=2))"
    extra="""result.update(classification='NATIVE_G1_CONTACT_FEEDBACK_CORRECTION / ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT',opening_task_success_claim=False,contact_feedback_only_policy=True,closure_correction_summary=closure.detail if 'closure' in globals() else None)
if 'closure' in globals():diagnostic_write(a.output/'CONTACT_RESPONSE_EVENTS.json',json.dumps(closure.response_records,indent=2))
"""
    text=text.replace(end,extra+end)
    return text

if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
    text=contact_source(spec);compile(text,__file__+'::contact_correction','exec')
    output=Path(spec['output']);output.mkdir(parents=True,exist_ok=True)
    (output/'ACTUAL_CONTACT_CORRECTION.py').write_text(text)
    (output/'CONTACT_EXECUTABLE_HASH.json').write_text(json.dumps({'entry_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'generated_sha256':hashlib.sha256(text.encode()).hexdigest(),'no_opening_task':True}))
    exec(compile(text,__file__+'::contact_correction','exec'),globals())
