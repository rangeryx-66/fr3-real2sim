"""Frozen-model same-start P4 replay; never reselects a model or parameter."""
import sys,copy,json,queue
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_interactive_twin_conditional import Experiment,read,write,absolute,sha
from interactive_twin_conditional.workflow import replay,logs
from interactive_twin.sysid import NoiseScales
from interactive_twin_refinement.physics import score
import numpy as np

def run(e,eid='dev_7320_00'):
 out=e.out/eid;selection=read(out/'selection_frozen.json');prepared=read(out/'prepared.json');source=prepared['source_job']
 result_path=out/'common_start_heldout/results.json'
 if result_path.exists():
  result=read(result_path)
  if result['selection_sha256']!=sha(out/'selection_frozen.json'):raise RuntimeError('HELDOUT_SELECTION_CHANGED')
  return result
 condition='original';base={**source,'only_probe':'P4','output':str(out/'common_start_heldout/reference/original')}
 write(out/'common_start_heldout/registration.json',{'selection_sha256':sha(out/'selection_frozen.json'),'same_previously_frozen_P4_waveform':True,'same_initial_snapshot_sha256':sha(source['conditional_snapshot']),'model_selection_changes':False,'purpose':'separate action response from state error accumulated during prior training actions','previous_continuation_results_retained':True})
 r=e.native(base)
 if r['status']!='PHYSICS_PROTOCOL_COMPLETE':write(out/'common_start_heldout/stop.json',{'status':r['status']});return
 tape=Path(base['output'])/'conditional_tape.json'
 for condition,phi in e.c['hidden_reference_conditions'].items():
  if condition!='original':e.native(replay(e,base,out/'common_start_heldout/reference'/condition,base['asset_root'],phi,tape)|{'role':'reference'})
 unique={}
 for condition,s in selection['conditions'].items():
  for label,m in s['methods'].items():
   if not m:continue
   unique[m['candidate_id']]=m
 for cid,m in unique.items():
  structure=next(s for s in prepared['structures'] if s['id']==m['structure_id'])
  e.native(replay(e,base,out/'common_start_heldout/predictions'/cid,structure['asset'],{k:m[k] for k in ('tau_c','b')},tape))
 noise=NoiseScales(**read(absolute('results/interactive_twin_benchmark_20261003_v2/robot_calibration/robot_calibration.json'))['noise_scales']);rows=[]
 for condition,s in selection['conditions'].items():
  ref=logs(out/'common_start_heldout/reference'/condition,('P4',));predictions={}
  for label,m in s['methods'].items():
   if not m:continue
   folder=out/'common_start_heldout/predictions'/m['candidate_id'];p=logs(folder,('P4',));r=read(folder/'report.json')
   row={'condition':condition,'method':label,'selected':m,'status':r['status'],'prediction_folder':str(folder),'identifiability':s['status'],'parameter_intervals':s['parameter_intervals']}
   if ref and p:
    row['test']=score(ref['P4'],p['P4'],noise);a=np.asarray(ref['P4']['signals']['ee_T_world_tcp'])[:,:3,3];b=np.asarray(p['P4']['signals']['ee_T_world_tcp'])[:,:3,3]
    row.update(action_relative_RMSE_mm=float(np.sqrt(np.mean(((b-b[0])-(a-a[0]))**2))*1000),action_start_offset_mm=float(np.linalg.norm(b[0]-a[0])*1000));predictions[label]=p['P4']
   rows.append(row)
  if ref and predictions:
   from interactive_twin.reporting import plot_heldout_predictions
   plot_heldout_predictions(ref['P4'],predictions,out/'common_start_heldout'/f'prediction_{condition}.png')
 result={'selection_sha256':sha(out/'selection_frozen.json'),'parameters_reselected':False,'initialization_once_per_episode':True,'rows':rows}
 write(result_path,result)
 print('COMMON_START_P4_COMPLETE',flush=True)
 return result

if __name__=='__main__':
 e=Experiment(read(absolute('configs/interactive_twin_conditional.yaml')))
 # Device allocation only: other GPUs continue the independent full-task runs.
 e.pool=queue.Queue();e.pool.put(0)
 run(e)
