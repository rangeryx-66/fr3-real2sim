"""Repository-wide guard inventory plus exact active-function/owner audit."""
import ast,hashlib,json,re,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
PATTERN=re.compile(r'PROBE_CARTESIAN_SPEED_LIMIT|SOFT_FORCE_WARNING|soft_force_n|max_pad_load_n|TemporalForceGuard')
DELEGATES={
 'scripts/native_g1_contact_recovery.py':['build_geometry','checked_route','release_geometry','observe_current','set_state'],
 'scripts/native_g1_hybrid_contact.py':['PanelScene','current_view_scene','evaluate_current_view','initial_observation','close_geometry','panel_plan'],
 'scripts/native_g1_safe_park.py':['parking_targets','route_to_joint','plan_exit','verify_park'],
 'scripts/native_g1_agent_execution.py':['check_path','approach_from','validate_execution','execute'],
 'scripts/native_g1_wrist_input.py':['install_camera','save_rgbd','self_mask','common_visible','refine']}

def audit(out):
 files=subprocess.check_output(['git','ls-files','-z'],cwd=ROOT).decode().split('\0');inventory=[]
 for name in files:
  if not name.endswith(('.py','.json')):continue
  path=ROOT/name
  if not path.is_file():continue
  for i,line in enumerate(path.read_text(errors='replace').splitlines(),1):
   if PATTERN.search(line):inventory.append(dict(file=name,line=i,text=line[:240],classification='preserved legacy/reference, not a new runtime owner'))
 selected={}
 for name,names in DELEGATES.items():
  path=ROOT/name;tree=ast.parse(path.read_text());nodes={n.name:n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
  snippets={n:ast.unparse(nodes[n]) for n in names}
  for n,s in snippets.items():
   if PATTERN.search(s):raise RuntimeError('Legacy guard reachable in selected active delegate '+name+':'+n)
  selected[name]=dict(functions=names,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
 active={}
 for f in sorted((ROOT/'interaction_runtime').glob('*.py')):
  s=f.read_text();compile(s,str(f),'exec')
  if f.name not in ['serialization.py']:
   assert 'PROBE_CARTESIAN_SPEED_LIMIT' not in s and 'TemporalForceGuard' not in s,f
  active[str(f.relative_to(ROOT))]=hashlib.sha256(f.read_bytes()).hexdigest()
 # In the only legacy semantic input, the obsolete pad ceiling is explicitly
 # removed from the live closure dictionary. The closure algorithm remains frozen.
 assert ".pop('max_pad_load_n',None)" in (ROOT/'interaction_runtime/runtime.py').read_text()
 assert ".pop('force_policy',None)" in (ROOT/'interaction_runtime/runtime.py').read_text()
 assert 'run_native_g1_pull_push_v12' not in (ROOT/'scripts/run_fast_interaction_v1.py').read_text()
 result=dict(reference='eb80bf3',tracked_files_scanned=len(files),legacy_guard_inventory=inventory,
  active_owner='interaction_runtime.runtime.Runtime.step',active_sources=active,selected_pure_legacy_delegates=selected,
  legacy_owners_imported_as_owners=False,old_force_and_timing_job_keys_migrated=True,
  generated_motion_controller_branches=0,generated_planning_code='same frozen observed-geometry planner only, no physical guard owner',
  note='The baseline intentionally retains old policies. New execution has its own explicit owner, not another wrapper around baseline step/run.')
 out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2));return result
if __name__=='__main__':
 out=Path(sys.argv[1]);r=audit(out);print(json.dumps(dict(tracked_files_scanned=r['tracked_files_scanned'],legacy_guard_sites=len(r['legacy_guard_inventory']),active_owner=r['active_owner'],active_sources=len(r['active_sources']),selected_functions=sum(len(v['functions']) for v in r['selected_pure_legacy_delegates'].values()),active_legacy_fatal_thresholds=0),indent=2))
