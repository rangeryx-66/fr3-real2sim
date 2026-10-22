"""Read-only, same-recording V7.5 estimator comparison; no hidden truth input.

Writes compact derived diagnostics outside the immutable episode. The selected
FIT-only family is frozen from the original online model for every comparison.
"""
import argparse,copy,hashlib,json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from update_probing_v7_model import load_segments
from diagnose_probing_v3 import columns,rows_at
from xarm_ft_v7_estimator import samples,fit,prediction
from update_prismatic_v73_model import qualify
from prismatic_estimator_audit_v75 import compare_qualification,stability,applicability,perturb_raw
from run_prismatic_synthetic_v75 import SCENARIOS
from xarm_ft_v31_estimator import process
from interaction_runtime.serialization import result_packet


def audit(episode,output,allow_interrupted_fit=False):
    episode=Path(episode).resolve();output=Path(output).resolve()
    if output==episode or episode in output.parents:raise ValueError('Derived audit must not write inside the original episode')
    began=time.perf_counter();model_path=episode/'model/estimate.json'
    if not model_path.exists():
        result=dict(status='unavailable: no frozen online model file',hidden_truth_read=False)
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n');return result
    online=json.loads(model_path.read_text());template=online.get('selected')
    pieces,settings,provenance=load_segments(episode,allow_interrupted_fit);config=settings['estimation']
    if settings['prior']['joint_type']!='prismatic':raise ValueError('Prismatic-only archive audit')
    if not template:
        result=dict(status='unavailable: no eligible frozen online model; no family invented',
            segment_eligibility=provenance,processed_blocks=len(pieces),hidden_truth_read=False,
            original_online_model_sha256=hashlib.sha256((episode/'model/estimate.json').read_bytes()).hexdigest())
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result_packet(result),indent=2)+'\n')
        return result
    result=dict(status='offline diagnostic, not replacement online result',
        original_online_model_sha256=hashlib.sha256((episode/'model/estimate.json').read_bytes()).hexdigest(),
        telemetry_index_sha256=hashlib.sha256((episode/'telemetry/index.json').read_bytes()).hexdigest(),
        qualification=compare_qualification(pieces,config,template),segment_eligibility=provenance,
        hidden_truth_read=False,validation_used_for_fit_or_selection=False,scenarios={})
    train=[]
    for d in copy.deepcopy(pieces):
        if d['segments'][0].startswith('FIT'):qualify(d);train.extend(samples(d,config['domain_native'],config['intervals']))
    if len(train)>=8:
        m=fit(train,config,template['intervals'],template.get('smoothness',0))
        result.update(stability=stability(train,config,m),applicability=applicability(train,m))
    # Reconstruct own raw numeric inputs, then perturb BEFORE any compensation.
    # Eligible runs come from exactly the loader above; no source GT is accessed.
    a,index=columns(episode);raw=[]
    for d in pieces:
        ix=np.flatnonzero((a['t']>=d['t'][0]-1e-9)&(a['t']<=d['t'][-1]+1e-9))
        if len(ix)!=len(d['t'])or not np.allclose(a['t'][ix],d['t'],rtol=0,atol=1e-9):raise ValueError('Archived block/time association mismatch')
        raw.append((d,rows_at(a,ix,index['metadata'],str(d['segments'][0]))))
    for name,scenario in SCENARIOS.items():
        rr=[];held=[];retention=[]
        for d,rows in raw:
            r,q,v=perturb_raw(rows,d['q'],d['native_qd'],scenario)
            p=process(r,settings['prior'],settings['bias'],v,native_coordinate=q);p['block_id']=d['block_id']
            if p['segments'][0].startswith('FIT'):
                retention.append(qualify(p));rr.extend(samples(p,config['domain_native'],config['intervals']))
            else:samples(p,config['domain_native'],config['intervals']);held.append(p)
        packet=dict(assumptions=scenario,bins=len(rr),retention=retention)
        if len(rr)>=8:
            model=fit(rr,config,template['intervals'],template.get('smoothness',0))
            packet.update(model=model,applicability=applicability(rr,model),heldout={p['block_id']:prediction(p,model)for p in held})
        else:packet['status']='unknown: insufficient qualified FIT'
        result['scenarios'][name]=packet
    result['complete_physical_validation']=[dict(role=s['role'],physical_status=s.get('result',{}).get('status'),
        recorded_duration_s=(s.get('last_step',s['first_step'])-s['first_step'])/240,
        complete_target=bool(s.get('result',{}).get('requested_target_reached')))
        for s in index['segments']if s['role'].startswith('VALIDATION')]
    result['wall_s']=time.perf_counter()-began
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result_packet(result),indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--allow-interrupted-fit',action='store_true');a=p.parse_args()
    r=audit(a.episode,a.output,a.allow_interrupted_fit);print(json.dumps(dict(status=r['status'],wall_s=r['wall_s'])))
