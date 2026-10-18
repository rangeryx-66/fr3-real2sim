"""Separate post-hoc comparison; never imported by the diagnostic estimator."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def evaluate(diagnostic,reference):
    d=json.loads(diagnostic.read_text());r=json.loads(reference.read_text())
    truth=np.array([v['authored_truth'] for v in r['independent_parameter_evaluation']]);rows={}
    models=dict(original=d['original'],**d['variants'],**{'weak_'+k:v for k,v in d['weak_momentum'].items()})
    for name,model in models.items():
        b=np.array(model['parameters']);rows[name]=dict(parameters=b.tolist(),absolute_errors=(b-truth).tolist(),relative_error_percent=(100*(b/truth-1)).tolist())
    bias=d['original']['parameters'][0]-truth[0];acc=d['original']['acceleration_rms'];delta=d['torque_processing_changes_FWL']['single_pass_121']['delta_I_FWL_kg_m2']
    return dict(authored_reference=truth.tolist(),not_independent_PhysX_effective_inertia_measurement=True,diagnostics_sha256=hashlib.sha256(diagnostic.read_bytes()).hexdigest(),
        authored_reference_source=str(reference),truth_used_for_variant_selection=False,models=rows,
        original_inertia_bias_kg_m2=float(bias),bias_equivalent_torque_rms_nm=float(bias*acc),
        single_pass_torque_change_FWL_equivalent_rms_nm=float(abs(delta)*acc),
        explanation='FWL uses the same original acceleration residualized against velocity/sign; diagnostic processing changes, not a truth-fitted compensation coefficient')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('diagnostic',type=Path);p.add_argument('reference',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    a.output.write_text(json.dumps(evaluate(a.diagnostic,a.reference),indent=2))
