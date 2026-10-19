"""Read only our trusted local experiment exports; no Isaac initialization."""
import argparse,collections,gzip,hashlib,json,pickle,subprocess,time
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--parent',type=Path,required=True);a=p.parse_args();b=a.parent/'episode';out=a.parent.parents[1]
result={};physics=[]
for name in ['physics_steps_chunks','observations_chunks','closure_commands_chunks']:
 idx=json.loads((b/name/'index.json').read_text());count=0;timestamps=[];errors=[]
 for chunk in idx['chunks']:
  if chunk['first_record']!=count:errors.append('noncontiguous first_record')
  rows=pickle.load(gzip.open(b/name/chunk['file'],'rb'))
  if len(rows)!=chunk['records']:errors.append('chunk count mismatch')
  count+=len(rows);timestamps.extend(r['t'] for r in rows if 't' in r)
  if name=='physics_steps_chunks':physics.extend(rows)
 if count!=idx['records']:errors.append('index count mismatch')
 dt=np.diff(timestamps)
 result[name]=dict(records=count,chunks=len(idx['chunks']),errors=errors,strictly_monotonic=bool(np.all(dt>0)),dt_min_s=float(dt.min()) if len(dt) else None,dt_max_s=float(dt.max()) if len(dt) else None)
result['physics_phases']=dict(collections.Counter(x['phase'] for x in physics));result['minimum_joint_margin_rad']=min(x['margin_rad'] for x in physics)
result['max_abs_door_angle_deg']=max(abs(x['door_angle_deg']) for x in physics)
ff='/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static/'
result['video']=json.loads(subprocess.check_output([ff+'ffprobe','-v','error','-select_streams','v:0','-count_frames','-show_entries','stream=codec_name,width,height,r_frame_rate,nb_read_frames,duration','-of','json',str(b/'contact_baseline.mp4')],text=True))
v=subprocess.run([ff+'ffmpeg','-v','error','-i',str(b/'contact_baseline.mp4'),'-f','null','-'],capture_output=True,text=True);result['video_decode_returncode']=v.returncode;result['video_decode_errors']=v.stderr
record=json.loads((a.parent/'PROCESS_EXIT.json').read_text());result['actor']=record
r=subprocess.run(['ps','-p',str(record['pid']),'-o','pid=,stat=,args='],capture_output=True,text=True);result['actor_still_present']=bool(r.stdout.strip());result['actor_ps']=r.stdout.strip()
paths=sorted(x for x in out.rglob('*') if x.is_file() and x.name not in ['EVIDENCE_INTEGRITY.json','PRIVATE_FILE_HASHES.json'])
hashes={str(x.relative_to(out)):hashlib.sha256(x.read_bytes()).hexdigest() for x in paths};(out/'PRIVATE_FILE_HASHES.json').write_text(json.dumps(hashes,indent=2));result['private_file_count']=len(hashes);result['file_index_sha256']=hashlib.sha256((out/'PRIVATE_FILE_HASHES.json').read_bytes()).hexdigest();result['checked_unix_s']=time.time()
(out/'EVIDENCE_INTEGRITY.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
