"""FIT-only consistency diagnostics beside, never replacing, the frozen model.

Conditional regression uncertainty assumes one unchanged force-transmission law.
Independent FIT trajectories and time-local fits test that assumption without
hidden dynamics or VALIDATION samples. Empirical spread is not a confidence bound.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from update_probing_v7_model import load_segments
from xarm_ft_v7_estimator import samples,fit


def quality(folder):
    pieces,settings,provenance=load_segments(Path(folder),True)
    model_file=Path(folder)/'model/estimate.json'
    production=json.loads(model_file.read_text()) if model_file.exists() else {}
    qualification=bool(production.get('FIT_qualification_v73'))
    if qualification:
        from update_prismatic_v73_model import qualify
        for d in pieces:qualify(d)
    c=settings['estimation'];rows=[];groups=[]
    for d in pieces:
        if not d['segments'][0].startswith('FIT'):continue
        rr=samples(d,c['domain_native'],c['intervals']);rows.extend(rr)
        if len(rr)<8:continue
        m=fit(rr,c);local=[]
        # Diagnostic windows operate on already segment-local processed FIT;
        # they never cross or import a validation boundary.
        t0=float(d['t'][0]);t=np.asarray(d['t'])-t0
        for start in np.arange(0,max(0,float(t[-1])-2)+1e-6,2.):
            mask=(t>=start)&(t<start+2)&d['valid']
            dd=dict(d,valid=mask);r=samples(dd,c['domain_native'],c['intervals'])
            if len(r)>=8:
                f=fit(r,c);local.append(dict(relative_interval_s=[float(start),float(start+2)],parameters=f['parameters'],conditional_std=f['conditional_std'],condition=f['scaled_condition']))
        groups.append(dict(segment=d['block_id'],bins=len(rr),parameters=m['parameters'],conditional_std=m['conditional_std'],condition=m['scaled_condition'],velocity_rms_m_s=float(np.sqrt(np.mean(d['qd'][d['valid']]**2))),acceleration_rms_m_s2=float(np.sqrt(np.mean(d['qdd'][d['valid']]**2))),local_FIT_diagnostics=local))
    result=dict(schema='prismatic-FIT-consistency-v73',production_estimate_modified=False,hidden_truth_used=False,validation_used=False,production_sliding_support_qualification=qualification,groups=groups,FIT_bins=len(rows),meaning='conditional uncertainty excludes changing contact/constraint loads; constant-model cross-FIT spread can also reflect real position-dependent resistance or excitation differences, not calibrated confidence or an execution veto')
    if len(rows)>=8:
        full=fit(rows,c);leave=[]
        for name in dict.fromkeys(r['segment'] for r in rows):
            kept=[r for r in rows if r['segment']!=name]
            if len(kept)>=8:leave.append(dict(omitted=name,parameters=fit(kept,c)['parameters']))
        result.update(pooled_constant_diagnostic=full,leave_one_FIT_out=leave)
        if groups:
            B=np.array([r['parameters'][:3] for r in groups]);result['between_FIT_parameter_range']=np.ptp(B,axis=0).tolist()
            result['between_FIT_parameter_std']=np.std(B,axis=0).tolist()
        result['interpretation']='Compare between-FIT/local spread to conditional uncertainty and contact history before claiming parameter accuracy. A complete VAL trajectory alone does not establish parameter identifiability.'
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);a=p.parse_args()
    from agent_command_protocol_v12 import atomic_json
    atomic_json(a.episode/'model/FIT_CONSISTENCY_V73.json',quality(a.episode))
