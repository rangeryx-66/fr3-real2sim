from pathlib import Path
import zipfile,hashlib,json,time
repo=Path('/Users/rangeryx/Documents/ChatGPT/robot in loop real2sim/github_release/fr3-real2sim-piper-mobile-door');doc=repo/'docs/experiments/door_transfer4_20261012';dest=repo/'engineering_artifacts_20261012/door_transfer4/results';archive=Path('/private/tmp/door_transfer4_20261012.zip');start=time.time()
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
expected=json.loads((doc/'ARCHIVE_SUMMARY.json').read_text());actual=sha(archive)
if actual!=expected['archive_sha256']:raise RuntimeError('archive hash mismatch')
with zipfile.ZipFile(archive) as z:
 for name in z.namelist():
  p=Path(name)
  if p.is_absolute() or '..' in p.parts:raise RuntimeError('unexpected archive path')
 z.extractall(dest)
manifest=json.loads((dest/'EVIDENCE_SHA256.json').read_text());bad=[n for n,h in manifest.items() if sha(dest/n)!=h];streams={}
for ident in ['45415','45176','45134','45443']:
 p=dest/ident/'physical_01';e=json.loads((p/'EXPORT_COMPLETE.json').read_text());streams[ident]={k:{'buffer':v['buffer_records'],'exported':v['index']['records'],'equal':v['buffer_records']==v['index']['records']} for k,v in e['streams'].items()}
result={'archive_sha256':actual,'archive_bytes':archive.stat().st_size,'extracted_path':str(dest),'files_checked':len(manifest),'mismatches':bad,'streams':streams,'all_streams_match':all(v['equal'] for d in streams.values() for v in d.values()),'verification_wall_s':time.time()-start}
(doc/'LOCAL_ARCHIVE_VERIFICATION.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if bad:raise RuntimeError('evidence hash mismatch')
