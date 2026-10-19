"""Isolated explicit recovery entry around the frozen native-G1 executable."""
import hashlib,json,sys
from pathlib import Path
from run_native_g1_wrist_short import wrist_source


def source(job):
    text=wrist_source(job)
    start="try:\n    signal.signal(signal.SIGINT,request_owner_stop)"
    end="grasp_event=next((e for e in telemetry_events"
    if text.count(start)!=1 or text.count(end)!=1:raise ValueError('Pinned owner boundary changed before scene initialization')
    begin=text.index(start);finish=text.index(end,begin)
    owner="""from native_g1_contact_recovery import run as run_contact_recovery
recovery_result=None
try:
    signal.signal(signal.SIGINT,request_owner_stop);signal.signal(signal.SIGTERM,request_owner_stop)
    recovery_result=run_contact_recovery(globals())
    success=recovery_result['success'];status=recovery_result['status']
except BaseException as error:
    import traceback
    status=repr(error);success=False;opening=False;qvelocity=np.zeros(7)
    diagnostic_write(a.output/'RECOVERY_OWNER_EXCEPTION.json',json.dumps({'error':status,'traceback':traceback.format_exc(),'physics_not_restarted':True},indent=2))
    print(traceback.format_exc(),flush=True)
finally:
    finish_trial()

"""
    text=text[:begin]+owner+text[finish:]
    marker="diagnostic_write(a.output/'report.json',json.dumps(result,indent=2))"
    insert="result.update(contact_recovery=recovery_result,classification='CONTACT_FEEDBACK_RECOVERY_V1 / AGENT_WRIST_RGBD / KNOWN_HINGE_STATE / APPROXIMATE_ACTUATOR_SIMULATION',physical_grasp_attempts=globals().get('physical_grasps_started',0),intentional_releases=len([e for e in telemetry_events if e.get('event')=='RECOVERY_PHYSICAL_RELEASE']),regrasps=max(0,globals().get('physical_grasps_started',0)-1))\n"
    if text.count(marker)!=1:raise ValueError('Pinned report boundary changed before scene initialization')
    return text.replace(marker,insert+marker)


if __name__=='__main__':
    job=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());text=source(job);compile(text,__file__+'::recovery_owner','exec')
    out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    (out/'ACTUAL_RECOVERY_EXECUTABLE.py').write_text(text)
    hashes={}
    for f in job['activation_source_files']:
        p=Path(__file__).resolve().parents[1]/f;hashes[f]=hashlib.sha256(p.read_bytes()).hexdigest();(out/(p.name+'.snapshot')).write_bytes(p.read_bytes())
    (out/'RECOVERY_EXECUTABLE_HASH.json').write_text(json.dumps(dict(sources=hashes,generated_sha256=hashlib.sha256(text.encode()).hexdigest(),controllers_unchanged=True,explicit_operations=True,maximum_grasps=2),indent=2))
    exec(compile(text,__file__+'::recovery_owner','exec'),globals())
