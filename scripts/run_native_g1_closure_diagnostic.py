"""Bounded closure-only experiments using the frozen native scene and guards.
The benchmark runner and modules are never edited. No opening task is run.
"""
from pathlib import Path
import sys,json,hashlib
from xarm7_native_execution import executable

def diagnostic_source(spec):
    text=executable(spec)
    definition=r'''
diagnostic_details={}
def fixed_tcp_additional_closure():
    global qvelocity,jaw_width_command,mode,effort,phase,legal,reference
    qvelocity=np.zeros(7);mode='position';effort=0.
    locked=tcp().copy();arm_command=qtarget[arm].copy();old=FixedClosure(locked,policy)
    diagnostic_details.update(T_tcp_command_locked=locked.tolist(),arm_command_locked=arm_command.tolist(),TCP_compensation_commanded=False,rate_m_s=min(.0005,policy['slow_closure_m_s']),increment_m=.00025)
    def width_command(w):
        global jaw_width_command
        jaw_width_command=float(w);qtarget[fingers]=controller.geometry.angle(jaw_width_command)
    def physical_hold(seconds):
        for _ in range(max(1,int(seconds/dt))):step()
    def verify_contact():
        global phase,legal,reference
        phase='DIAGNOSTIC_BILATERAL_POSITION_HOLD';physical_hold(1.)
        ready,detail=grip_window();diagnostic_details['holding_verification']=detail
        diagnostic_details['bilateral_contact_observed']=True;diagnostic_details['stable_existing_grip_window']=bool(ready)
        legal=bool(ready)
        if legal:
            observe_grasp();reference=np.linalg.inv(moving())@tcp();physical_hold(.5)
        return bool(ready)
    first=None
    for _ in range(int(100/dt)):
        state=rows[-1];P=poses(np.asarray(state['q']));centers=[(P[n]@np.r_[model.pads[n].mean(0),1])[:3] for n in FINGERS]
        cmd=_fixed_update(old,tcp(),state['aperture_m'],filtered(),centers,dt)
        if cmd['mode']=='position':width_command(cmd['opening_m'])
        # Only the old aperture law is reproduced; its TCP-centering output is
        # deliberately NOT applied, to isolate continued jaw closure.
        phase='DIAGNOSTIC_OLD_'+cmd['state'];step()
        if cmd['state']=='CENTER':
            first={'t':tick*dt,'command_aperture_m':jaw_width_command,'measured_aperture_m':rows[-1]['aperture_m'],'filtered_pad_loads_n':filtered().tolist()};break
        if min(filtered())>=.05:
            diagnostic_details['already_bilateral_before_additional_closure']=True
            return verify_contact()
    if first is None:
        diagnostic_details['termination']='NO_UNILATERAL_CONTACT_WITHIN_ORIGINAL_CLOSURE_BUDGET';return False
    diagnostic_details['first_unilateral_contact']=first
    phase='DIAGNOSTIC_OLD_CENTER_APERTURE_HOLD';physical_hold(.5)
    lower=float(controller.geometry.width(controller.geometry.grid[-1]));diagnostic_details['native_minimum_aperture_m']=lower
    start=jaw_width_command;rate=diagnostic_details['rate_m_s']
    count=max(1,int(np.ceil((start-lower)/diagnostic_details['increment_m'])))
    for _ in range(count):
        if force_guard.pending:
            phase='DIAGNOSTIC_EXISTING_FORCE_SETTLE';physical_hold(.25)
            if force_guard.settled():force_guard.pending=False
        previous=jaw_width_command;next_width=max(lower,previous-diagnostic_details['increment_m'])
        steps=max(1,int(np.ceil((previous-next_width)/rate/dt)))
        phase='DIAGNOSTIC_ADDITIONAL_CLOSE_FIXED_TCP'
        for k in range(1,steps+1):
            width_command(previous+(next_width-previous)*k/steps);step()
            if min(filtered())>=.05:
                diagnostic_details['contact_acquisition_aperture_m']=rows[-1]['aperture_m']
                return verify_contact()
            if force_guard.pending:break
        if next_width<=lower and jaw_width_command<=lower:break
    phase='DIAGNOSTIC_FINAL_POSITION_HOLD';physical_hold(.5)
    diagnostic_details['termination']='NATIVE_STROKE_REACHED_WITHOUT_BILATERAL_CONTACT' if jaw_width_command<=lower else 'BOUNDED_INCREMENT_SEQUENCE_ENDED_WITHOUT_BILATERAL_CONTACT';return False
'''
    anchor="provenance.update(opening_controller_sha256="
    if text.count(anchor)!=1:raise ValueError('DIAGNOSTIC_PROVENANCE_ANCHOR')
    before = "if spec['closure_diagnostic_mode']=='fixed_tcp':\n    initial=initial[:initial.index(\"  closure=JawCenteredClosure(tcp(),policy);phase='CLOSE'\")]+" + repr("  legal=fixed_tcp_additional_closure()\n") + "\n"
    text=text.replace(anchor,definition+'\n'+before+anchor)
    begin=text.index("    grasp_relative=np.linalg.inv(moving())@tcp();reference=grasp_relative.copy();ref=tcp().copy()",text.index("exec(compile(initial,str(baseline)"))
    end=text.index('except BaseException as error:',begin)
    replacement="""    opening=False;qvelocity=np.zeros(7)
    if spec['closure_diagnostic_mode']=='shared_progressive':
        phase='SHARED_PROGRESSIVE_CLOSURE_HOLD';hold(.5)
        diagnostic_details.update(stable_existing_grip_window=bool(legal),TCP_centering_behavior='original bounded policy unchanged')
    status='CLOSURE_HOLD_VERIFIED' if legal else 'CLOSURE_DIAGNOSTIC_NOT_VERIFIED';success=bool(legal)
"""
    text=text[:begin]+replacement+text[end:]
    marker="diagnostic_write(a.output/'report.json',json.dumps(result,indent=2))"
    extra="""result.update(classification='NATIVE_G1_CLOSURE_DIAGNOSTIC / KNOWN_MODEL_DEVELOPMENT',opening_task_success_claim=False,target_deg=None,diagnostic_mode=spec['closure_diagnostic_mode'],diagnostic_details=diagnostic_details,diagnostic_runner_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
"""
    if text.count(marker)!=1:raise ValueError('DIAGNOSTIC_RESULT_ANCHOR')
    return text.replace(marker,extra+marker)

if __name__=='__main__':
    spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
    text=diagnostic_source(spec);compile(text,__file__+'::native_closure_diagnostic','exec')
    output=Path(spec['output']);output.mkdir(parents=True,exist_ok=True)
    (output/'ACTUAL_CLOSURE_DIAGNOSTIC.py').write_text(text)
    (output/'DIAGNOSTIC_EXECUTABLE_HASH.json').write_text(json.dumps({'entry_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'generated_sha256':hashlib.sha256(text.encode()).hexdigest(),'no_opening_task':True}))
    exec(compile(text,__file__+'::native_closure_diagnostic','exec'),globals())
