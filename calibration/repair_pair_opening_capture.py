"""Single data-chain repair: current payload opening, never historical opening.

Keep original run immutable. Reuse raw payload captures, recollect only empty
at the measured opening, then reapply every frozen acceptance check.
"""
import os,json,subprocess,hashlib,time,sys
from pathlib import Path
import numpy as np
from strict_pair_protocol import POLICY,window_quality
R=Path(__file__).resolve().parents[1];old=R/'results/fr3_clean_pair_fd_gate';O=old/'matched_opening_repair';root=O/'banana';root.mkdir(parents=True,exist_ok=True)
spec=json.load(open(old/'banana/manifest.json'));orig=json.load(open(old/'banana/payload/strict_capture.json'))
# Only windows with the opening mismatch and no other failure contribute.
rows=[x for x in orig['rows'] if set(x['reasons'])<= {'OPENING_MISMATCH','PAIR_OPENING_MISMATCH'}]
assert rows
opening=float(np.median([x['opening_mean_m'] for x in rows]));spec['opening_m']=opening;spec['opening_source']='CURRENT_PAYLOAD_MEASURED_MEDIAN';spec['original_manifest']=str(old/'banana/manifest.json')
manifest=root/'manifest.json';manifest.write_text(json.dumps(spec,indent=2));out=root/'empty';out.mkdir(exist_ok=True)
frozen=json.load(open(old/'frozen_protocol.json'));(O/'frozen_protocol.json').write_text(json.dumps(frozen,indent=2))
env=dict(os.environ,STRICT_PAIR_MANIFEST=str(manifest),PAYLOAD_JACOBIAN_REFRESH_TICKS='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1');env.pop('PAYLOAD_ATTACHMENT_MODE',None)
cmd=['/data1/home/rangeryx/isaaclab-arena/.venv/bin/python','-u','calibration/launch_strict_pair.py','--target','banana','--seed','1020','--mode','ANYGRASP','--gpu','3','--port','20216','--domain','176','--payload-v2','--mass-com-only','--skip-scan','--calibration-force','60','--calibration-mu','1.0','--output',str(out),'--baseline-only','--center-q',str(Path(spec['source'])/'excitation_center.json'),'--gripper-opening-mm',str(opening*1000)]
with (out/'runner.log').open('w') as f:ret=subprocess.run(cmd,cwd=R,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=1800).returncode
assert ret==0
empty=json.load(open(out/'strict_capture.json'));pp=root/'payload';pp.mkdir(exist_ok=True)
for f in (old/'banana/payload').glob('*_ANYGRASP.json'):(pp/f.name).symlink_to(f)
newrows=[];accepted=[]
for entry in orig['rows']:
 base=next((x for x in empty['accepted'] if x['pose_id']==entry['pose_id']),None)
 if base is None:continue
 with np.load(entry['path'],allow_pickle=True) as f:d={k:f[k] for k in f.files}
 row=window_quality(d,entry['q_ref'],opening,payload=True);row.update(pose_id=entry['pose_id'],attempt=entry['attempt'],q_ref=entry['q_ref'],path=entry['path'],original_reasons=entry['reasons'])
 pair=float(np.max(np.abs(np.array(row['q_mean'])-base['q_mean'])));row['q_pair_error_rad']=pair
 if pair>=POLICY['q_pair_max_rad']:row['reasons'].append('Q_PAIR_MISMATCH')
 if abs(row['opening_mean_m']-base['opening_mean_m'])>=POLICY['opening_max_m']:row['reasons'].append('PAIR_OPENING_MISMATCH')
 row['accepted']=not row['reasons'];newrows.append(row)
 if row['accepted'] and not any(x['pose_id']==row['pose_id'] for x in accepted):accepted.append(row)
(pp/'strict_capture.json').write_text(json.dumps({'rows':newrows,'accepted':accepted,'finished':True,'policy':POLICY,'original_payload_data':str(old/'banana/payload'),'GT_used':False},indent=2))
changes=[p for p,h in frozen['source_sha256'].items() if hashlib.sha256((R/p).read_bytes()).hexdigest()!=h]
(O/'complete.json').write_text(json.dumps({'rows':[{'target':'banana','empty_exit':ret,'payload_reused':True}],'frozen_files_changed':changes,'finished_unix':time.time()},indent=2))
env.update(STRICT_PAIR_OUTPUT=str(O),PYTHONPATH=str(R)+':'+str(R/'calibration')+':/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages')
with (O/'analysis.log').open('w') as f:r=subprocess.run(['/data1/home/rangeryx/official_payload_env/bin/python','calibration/evaluate_strict_pair.py'],cwd=R,env=env,stdout=f,stderr=subprocess.STDOUT)
(O/'analysis_exit.json').write_text(json.dumps({'returncode':r.returncode}))
