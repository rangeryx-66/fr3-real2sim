from pathlib import Path
import hashlib,json
r=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/xarm7_shared_doors_20261012');f=json.loads((r/'CONTACT_BATCH_SOURCE_FREEZE.json').read_text());dif=[p for p,h in f['sources'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
(r/'CONTACT_BATCH_SOURCE_AUDIT.json').write_text(json.dumps({'changed_after_freeze':dif,'shared_sources_identical_during_both_attempts':not dif},indent=2));a=[]
for case in ['45134','45443']:
 j=json.loads((r/case/'job.json').read_text());m=json.loads((r/case/'physical_01/frozen_proxy.json').read_text());a.append({'case':case,'effective_manifest_handle':m['selection'],'copied_descriptive_header':j['skill_inputs']['handle_region'],'geometry_control_uses':'case-specific manifest and automatically saved plan, not descriptive header','header_matches_actual':j['skill_inputs']['handle_region']['asset_id']==case})
(r/'EFFECTIVE_HANDLE_INPUT_AUDIT.json').write_text(json.dumps(a,indent=2));print('Sources changed:',dif)
