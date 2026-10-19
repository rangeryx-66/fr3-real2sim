"""Thin shared adapter. Frozen setup, closure, IK, guards and export stay literal."""
import sys,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
if '--lifecycle-selftest' in sys.argv:
    frozen=ROOT/'scripts/run_oracle_door_precontact_continuity.py'
    exec(compile(frozen.read_text(),str(frozen)+'::frozen_export_selftest','exec'),globals());sys.exit(0)
job=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text());inputs=job['skill_inputs']
if inputs['handle_region']['source']!='oracle_handle' or inputs['state_source']['type']!='oracle_dynamic':
    raise ValueError('PRELAUNCH_UNSUPPORTED_INPUT_SOURCE_NO_GT_FALLBACK_NO_ISAAC')
if inputs['permitted_recovery']!={'local_centering_attempts':1,'release':False,'regrasp':False,'move_base':False}:
    raise ValueError('PRELAUNCH_SCOPE_CONFIGURATION_MISMATCH_NO_ISAAC')
policy_path=ROOT/'scripts/shared_door_execution.py';policy_text=policy_path.read_text()
if hashlib.sha256(policy_text.encode()).hexdigest()!=job['shared_policy_sha256']:raise ValueError('PRELAUNCH_SHARED_POLICY_SOURCE_MISMATCH')
frozen=Path(job['frozen_execution_runner']);text=frozen.read_text()
if hashlib.sha256(text.encode()).hexdigest()!=job['frozen_execution_sha256']:raise ValueError('PRELAUNCH_FROZEN_SOURCE_MISMATCH')
needle="exec(compile(controller_source,spec['pink_runner']+'::unchanged_opening_control','exec'),globals())"
replacement="""frozen_opening_sha256=hashlib.sha256(controller_source.encode()).hexdigest()
drive_correction=np.eye(4)
controller_source=controller_source.replace('moving()@grasp_relative','moving()@grasp_relative@drive_correction')
"""+needle+"\nprovenance.update(frozen_runner_sha256=spec['frozen_execution_sha256'],frozen_opening_sha256=frozen_opening_sha256,shared_contact_policy_sha256=hashlib.sha256((ROOT/'scripts/shared_door_execution.py').read_bytes()).hexdigest(),skill_inputs=spec['skill_inputs'])"
if text.count(needle)!=1:raise ValueError('PRELAUNCH_ADAPTER_ANCHOR_CHANGED')
text=text.replace(needle,replacement)
old="""    while actual()<goal:
        phase='FIXED_BASE_PINK_OPEN';step()
    opening=False;qvelocity=np.zeros(6);phase='FINAL_HOLD';status='ORACLE_HANDLE_30_DEG_SUCCESS';success=True"""
new="""    policy_namespace={}
    exec(compile(policy_text,str(policy_path),'exec'),policy_namespace)
    success,status,adaptation_records=policy_namespace['open_with_contact_feedback'](globals())
    opening=False;qvelocity=np.zeros(6);phase='FINAL_HOLD' if success else 'TRANSMISSION_FAILURE_SAFE_HOLD'"""
if text.count(old)!=1:raise ValueError('PRELAUNCH_OPENING_ADAPTER_ANCHOR_CHANGED')
text=text.replace(old,new).replace('ONE_PRECONTACT_CONTINUITY_TRIAL','SHARED_FIXED_BASE_DOOR_SKILL')
exec(compile(text,str(frozen)+'::shared_contact_orchestration','exec'),globals())
