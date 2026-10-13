from pathlib import Path
import json,pickle,gzip,hashlib,subprocess
O=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007/results/native_g1_heldout_20261013');cases=['45526','45638','45667','45623','45717','45850','45671','45783','45908','45916'];out={};integrity=[]
for a in cases:
 d=O/a;p=d/'physical_01';r=json.loads((p/'report.json').read_text());analysis=json.loads((p/'ROUND_ANALYSIS.json').read_text());audit=json.loads((p/'NONHANDLE_CONTACT_AUDIT.json').read_text());counts={}
 for folder in ['observations_chunks','physics_steps_chunks']:
  index=json.loads((p/folder/'index.json').read_text());n=0;errors=[]
  for c in index['chunks']:
   q=p/folder/c['file'];rows=pickle.load(gzip.open(q,'rb'));n+=len(rows)
   if len(rows)!=c['records'] or q.stat().st_size!=c['bytes']:errors.append(c['file'])
  counts[folder]={'decoded_records':n,'indexed_records':index['records'],'chunk_errors':errors,'matches_report':n==r['physics_steps']}
 item={'asset_id':a,'decoded_streams':counts,'video_decode_errors_bytes':(d/'video_decode.log').stat().st_size,'export_complete':(p/'EXPORT_COMPLETE.json').exists(),'actor_exit':json.loads((d/'PROCESS_EXIT.json').read_text()),'source_runner_matches':r['active_sources']['runner_sha256']=='c08fb22edcf6cffdddef32c3ddaf37a8123f6f7f59025d5e785e8305b97065e9'}
 integrity.append(item);out[a]={'report':r,'analysis':{k:v for k,v in analysis.items() if k!='contact_pairs'},'nonhandle_contact_audit':audit,'integrity':item,'gpu':json.loads((d/'GPU_OWNERSHIP.json').read_text()),'initialization_failures_preserved':len(list(d.glob('initialization_*/ATTEMPT_CLASSIFICATION_AMENDMENT.json'))),'contact_adaptation':json.loads((p/'CONTACT_ADAPTATION.json').read_text()) if (p/'CONTACT_ADAPTATION.json').exists() else None}
(O/'TEN_CASE_METRICS.json').write_text(json.dumps(out,indent=2));(O/'TEN_CASE_EVIDENCE_INTEGRITY.json').write_text(json.dumps(integrity,indent=2));print(json.dumps(integrity,indent=2),flush=True)
subprocess.run(['python3',str(Path(__file__).with_name('summarize_native_g1_heldout_ten.py'))],check=True)

