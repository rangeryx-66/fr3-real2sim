"""Read-only held-out statistics. No alternative-threshold physical success estimates."""
import csv,hashlib,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/generalization_summary';OUT.mkdir(exist_ok=True)
def wilson(k,n):
 if not n:return None
 z=1.959963984540054;p=k/n;den=1+z*z/n;mid=(p+z*z/(2*n))/den;rad=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/den
 return [mid-rad,mid+rad]
def dist(v):
 v=sorted(v)
 if not v:return dict(n=0)
 def q(p):
  x=p*(len(v)-1);i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
 return dict(n=len(v),min=v[0],q25=q(.25),median=q(.5),q75=q(.75),max=v[-1])
def write(name,rows):
 if not rows:return
 with (OUT/name).open('w',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
allrows=[];candrows=[];stats={};sens=[];audits=[]
manifest=json.loads((ROOT/'GENERALIZATION_TARGETS.json').read_text())
for group,start,n in [('generalization_clutter50',100,50),('generalization_pose30',200,30)]:
 rows=[json.loads((ROOT/f'results/{group}/B_seed_{s:04d}.json').read_text()) for s in range(start,start+n)]
 categories={};rejected=[];selected_success=[];selected_failure=[];bad_above=[]
 for r in rows:
  wanted=manifest[str(r['seed'])];actual=r['initial_target']
  pose_error=math.dist(wanted['position'],actual['position']);assert pose_error<1e-5
  assert abs(sum(a*b for a,b in zip(wanted['quaternion_wxyz'],actual['quaternion_wxyz'])))>0.999999
  assert r['margin_m']==.001 and r['min_pad_coverage']==.045 and r['geometry_filter'] and r['lift_gain_scale']==2
  path=ROOT/f"results/{group}/inputs/seed_{r['seed']:04d}_grasps.json"
  if path.exists() and 'grasp_sha256' in r:
   assert hashlib.sha256(path.read_bytes()).hexdigest()==r['grasp_sha256']
   raw=json.loads(path.read_text())['grasps']
   if r['selected_rank'] is not None:assert r['telemetry']['grasp']==raw[r['selected_rank']]
  base=r['controller_gains'];lift=r.get('lift_controller_gains')
  assert base['kp'][-2:]==[1000.]*2 and all(abs(v-.3)<1e-6 for v in base['kd'][-2:])
  assert base['kp'][:7]==[10000.]*7 and all(abs(v-.1718873381614685)<1e-6 for v in base['kd'][:7])
  if lift:
   assert base['kp'][-2:]==lift['kp'][-2:] and base['kd'][-2:]==lift['kd'][-2:]
   assert all(abs(a-2*b)<.01 for a,b in zip(lift['kp'][:7],base['kp'][:7]))
   assert all(abs(a-math.sqrt(2)*b)<1e-6 for a,b in zip(lift['kd'][:7],base['kd'][:7]))
  if r['success']:
   h=r['hold_samples'];assert h[-1]['t']-h[0]['t']>=2 and min(x['z'] for x in h)-r['initial_target']['position'][2]>=.08
   assert min(min(x['forces']) for x in h)>.1 and max(x['z'] for x in h)-min(x['z'] for x in h)<=.01
  coverage=r['telemetry'].get('commanded_geometry',{}).get('min_pad_coverage')
  if coverage is not None:(selected_success if r['success'] else selected_failure).append(coverage)
  if r['category']=='BAD_CONTACT' and coverage is not None and coverage>=.045:bad_above.append(r['seed'])
  categories[r['category']]=categories.get(r['category'],0)+1
  for c in r['candidates']:
   cv=c['hand_geometry']['min_pad_coverage'];eligible=c['status'] in ['VALID','INSUFFICIENT_PAD_OVERLAP']
   if c['status']=='INSUFFICIENT_PAD_OVERLAP':rejected.append(cv)
   candrows.append(dict(group=group,seed=r['seed'],rank=c['rank'],score=c['score'],coverage=cv,status=c['status'],scene_executable=eligible,selected=c['rank']==r['selected_rank']))
  allrows.append(dict(group=group,seed=r['seed'],success=r['success'],category=r['category'],last_stage=r['last_stage'],detail=r.get('detail',''),selection_status='SELECTED' if r['selected_rank'] is not None else 'NO_CANDIDATE',non_target_contact=r['non_target_contact'],disturbance=r['non_target_disturbance'],scene_filtered=r.get('scene_filtered_candidates',0),pad_filtered=sum(c['status']=='INSUFFICIENT_PAD_OVERLAP' for c in r['candidates']),selected_rank=r['selected_rank'],selected_coverage=coverage,planning_seconds=r['planning_seconds'],lift_ge_8cm=r['lift_ge_8cm'],stable_hold_ge_2s=r['stable_hold_ge_2s'],max_displacement_m=r['max_displacement_m'],max_rotation_deg=r['max_rotation_deg']))
  audits.append(dict(group=group,seed=r['seed'],frozen_config=True,target_manifest_error_m=pose_error,raw_input_hash_verified=path.exists(),physical_success_verified=bool(r['success'])))
 k=sum(r['success'] for r in rows);contact=sum(r['non_target_contact'] for r in rows);disturbed=sum(r['non_target_disturbance'] for r in rows)
 stats[group]=dict(n=n,success=k,rate=k/n,ci95=wilson(k,n),contact=contact,contact_ci95=wilson(contact,n),disturbance=disturbed,disturbance_ci95=wilson(disturbed,n),categories=categories,raw_candidate_count=dist([len(r['candidates']) for r in rows]),episodes_reaching_top20=sum(len(r['candidates'])==20 for r in rows),pad_rejected=dist(rejected),selected_success_coverage=dist(selected_success),selected_failure_coverage=dist(selected_failure),bad_contact_above_gate=bad_above,criteria_pass=k/n>=.75 and contact/n<=.05 and disturbed/n<=.05)
 for threshold in [0,.02,.045,.075,.10]:
  eligible=[c for c in candrows if c['group']==group and c['scene_executable']]
  exhausted=sum(not any(c['seed']==r['seed'] and c['coverage']>=threshold for c in eligible) for r in rows)
  selected=[r for r in allrows if r['group']==group and r['selected_coverage'] is not None]
  changes=0
  for r in rows:
   ranks=[c['rank'] for c in eligible if c['seed']==r['seed'] and c['coverage']>=threshold]
   changes+=((min(ranks) if ranks else None)!=r['selected_rank'])
  sens.append(dict(group=group,threshold=threshold,selected_rank_would_change=changes,scene_executable_candidates=len(eligible),rejected_candidates=sum(c['coverage']<threshold for c in eligible),episodes_no_candidate=exhausted,observed_success_rejected=sum(r['success'] and r['selected_coverage']<threshold for r in selected),observed_failure_rejected=sum(not r['success'] and r['selected_coverage']<threshold for r in selected)))
write('episodes.csv',allrows);write('candidates.csv',candrows);write('sensitivity.csv',sens)
summary=dict(groups=stats,sensitivity=sens,audits=audits,development=dict(n=20,success=16,ci95=wilson(16,20)),limitation='Threshold analysis only counts eligibility and labels of actually executed poses; unexecuted candidate physical outcomes are unknown. No retuning or physical reruns.')
(OUT/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(stats,indent=2))
