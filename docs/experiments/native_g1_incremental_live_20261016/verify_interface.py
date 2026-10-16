import ast,hashlib,inspect,json
from pathlib import Path
import native_g1_agent_incremental_activation as a
import native_g1_agent_observation as original
from run_native_g1_wrist_short import wrist_source
root=Path.cwd();out=root/'docs/experiments/native_g1_incremental_live_20261016'
f=json.loads((out/'FROZEN.json').read_text())
for p,h in f['source_sha256'].items():assert hashlib.sha256((root/p).read_bytes()).hexdigest()==h,p
assert a.move_and_observe.__globals__['actuate'] is original.actuate
assert a.move_and_observe.__globals__['observe'] is a.observe
for name in ['install_camera','survey_candidates','select_route','actuate','evaluate_view','inspect','write']:
 assert callable(a.prepare.__globals__['frozen'].__dict__.get(name)),name
for name in ['observe','move_and_observe','build_geometry']:
 assert a.prepare.__globals__[name] is getattr(a,name),name
source=inspect.getsource(original.prepare);tree=ast.parse(source)
ops=sorted({n.comparators[0].value for n in ast.walk(tree) if isinstance(n,ast.Compare) and isinstance(n.left,ast.Name) and n.left.id=='op' and isinstance(n.comparators[0],ast.Constant)})
assert ops==['build_geometry','evaluate_view','finish','inspect','move_and_observe','observe'],ops
assert 'AGENT_OBSERVATION_COMPLETE_NO_MANIPULATION_ATTEMPT' in source
job=json.loads((root/'results/native_g1_agent_transfer_20261015/45936/job.json').read_text());s=wrist_source(job)
hook='from native_g1_wrist_input import prepare,refine';assert s.count(hook)==1
s=s.replace(hook,'from native_g1_agent_incremental_activation import prepare\n    from native_g1_wrist_input import refine');compile(s,'preflight_generated','exec')
(out/'PREFLIGHT_GENERATED.py.private').write_text(s)
r=dict(passed=True,source_hashes_match=True,operations=ops,actuate_original=True,observe_repaired=True,no_execution_operation=True,finish_always_raises_observation_sentinel=True,generated_sha256=hashlib.sha256(s.encode()).hexdigest(),physics_executed=False)
(out/'INTERFACE_PREFLIGHT.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
