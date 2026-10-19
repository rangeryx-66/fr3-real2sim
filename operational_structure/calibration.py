"""DEV-only empirical predictive envelope. Never reads GT or TEST outcomes."""
import json,hashlib
from pathlib import Path
import numpy as np
from interactive_twin_refinement.fitting import fit_se3
from operational_structure.prediction import prediction


def calibrate(c,root):
    records=[]
    for name,filename in c['calibration_logs'].items():
        rows=json.loads((root/filename).read_text())
        T=np.asarray([r['T_tcp'] for i,r in enumerate(rows) if i%8==0 and r['phase']=='ESTIMATED_FOLLOW' and r['margin_rad']>.05 and r['relative_translation_slip_m']<.001 and min(r['forces_n'].values())>.05])
        if len(T)<48:raise RuntimeError('CALIBRATION_MOTION_UNAVAILABLE:'+name)
        # Complete contiguous last segment excluded from calibration fit.
        boundary=len(T)-1
        while boundary>24 and np.linalg.norm(T[-1,:3,3]-T[boundary,:3,3])<.002:boundary-=1
        fit=fit_se3(T[:boundary],**c['fitting']);p=prediction(T[boundary:],fit)
        records.append({'source':name,'source_file':filename,'source_sha256':hashlib.sha256((root/filename).read_bytes()).hexdigest(),'training_samples':boundary,'withheld_samples':len(T)-boundary,
                        'prediction':p,'GT_read':False})
    # Conservative envelope of existing safe DEV predictions, never a global
    # penetration or slip tolerance. Repeat scales use frozen encoder noise.
    envelope={k:max(r['prediction'][k] for r in records) for k in
              ['position_rmse_m','rotation_rmse_rad','endpoint_error_m','maximum_position_error_m','tangent_error_deg']}
    operational={k:float(v) for k,v in envelope.items() if k!='maximum_position_error_m'}
    operational.update(position_repeat_scale_m=c['fitting']['position_noise_m'],rotation_repeat_scale_rad=c['fitting']['rotation_noise_rad'])
    confidence=dict(c['confidence_policy']);confidence.update(explanation_position_m=envelope['maximum_position_error_m'],
                                                             explanation_rotation_rad=envelope['rotation_rmse_rad'])
    return {'records':records,'operational':operational,'confidence':confidence,'GT_used':False,
            'interpretation':'empirical DEV predictive envelope, separate from frozen physical safety and .30mm accuracy'}
