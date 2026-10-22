"""Post-episode same-family diagnostic; frozen online estimate remains intact."""
import argparse,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from update_probing_v7_model import load_segments
from update_prismatic_v73_model import qualify
from xarm_ft_v7_estimator import samples,fit
from prismatic_applicability_v75 import diagnose
from interaction_runtime.serialization import result_packet


def audit(episode,output):
    p=Path(episode).resolve();out=Path(output).resolve()
    if p==out or p in out.parents:raise ValueError('Do not write into frozen episode')
    source=p/'model/estimate.json'
    if not source.exists():result=dict(status='UNKNOWN_NO_ONLINE_MODEL')
    else:
        online=json.loads(source.read_text());m=online.get('selected')
        if not m:result=dict(status='UNKNOWN_NO_ELIGIBLE_MODEL')
        else:
            labels=[s.get('quality',{}).get('label','')for s in online.get('segments',[])]
            enabled=any(label.startswith('PREDECLARED')or label=='COMPLETE_ACTION_DECLARED_FIT_INTERVALS'for label in labels)
            pp,settings,provenance=load_segments(p,enabled);rr=[];retention=[]
            for d in pp:
                if d['segments'][0].startswith('FIT'):
                    retention.append(qualify(d));rr.extend(samples(d,settings['estimation']['domain_native'],settings['estimation']['intervals']))
            if len(rr)<8:
                result=dict(status='UNKNOWN_INSUFFICIENT_QUALIFIED_FIT',bins=len(rr),qualification=retention)
                out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result_packet(result),indent=2)+'\n');return
            # Qualification is the tested factor on older recordings. Refit the
            # SAME original family/scales/ridge, rather than mislabeling a change
            # of training mask as regularization bias in the residual diagnostic.
            diagnostic_model=fit(rr,settings['estimation'],m['intervals'],m.get('smoothness',0))
            result=dict(diagnostic=diagnose(rr,diagnostic_model),qualification=retention,
                frozen_family=dict(intervals=m['intervals'],active_controls=m['active_controls']),
                frozen_online_parameters=m['parameters'],online_model_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                diagnostic_refit_parameters=diagnostic_model['parameters'],declared_FIT_intervals_enabled=enabled,
                diagnostic_source_sha256=hashlib.sha256((ROOT/'scripts/prismatic_applicability_v75.py').read_bytes()).hexdigest(),
                validation_measurements_enter_diagnostic=False,hidden_truth_read=False,original_result_modified=False)
            if online.get('FIT_qualification_v73'):
                import numpy as np
                result['strict_production_reproduction_max_absolute_coefficient_delta']=float(np.max(abs(np.asarray(m['parameters'])-diagnostic_model['parameters'])))
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result_packet(result),indent=2)+'\n')
    print(json.dumps(dict(episode=p.name,status=result.get('status',result.get('diagnostic',{}).get('status')))))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();audit(a.episode,a.output)
