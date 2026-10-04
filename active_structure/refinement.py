"""Final acceptance stays strict; no GT or held-out selection inputs."""
import json
import numpy as np
from cross_object_structure.refinement import refine as existing_refine,measured_action
from active_structure.discovery import hypotheses


def refine(rows,discovery,policy,output):
    result=existing_refine(rows,discovery,policy,output)
    train,_=measured_action(rows,'ESTIMATED_FOLLOW')
    evidence=hypotheses(train,policy['active']['discovery']) if len(train)>=24 else {'status':'UNOBSERVABLE'}
    fit=result.get('refined',{});r=fit.get('revolute',{})
    excitation=np.rad2deg(r.get('angle_span_rad',0))>=policy['active']['final']['minimum_angle_deg']
    strict=bool(r.get('position_rmse_m',float('inf'))<.0003 and r.get('rotation_rmse_rad',float('inf'))<.001)
    result.update(final_geometry_gate={'position_rmse_limit_m':.0003,'rotation_rmse_limit_rad':.001,'passed':strict},
                  candidate_consistency=evidence,angular_excitation_passed=bool(excitation),
                  accepted=bool(result['accepted'] and strict and excitation and evidence['status']!='UNOBSERVABLE'))
    result['status']='REFINED_MODEL_ACCEPTED' if result['accepted'] else ('REFINEMENT_INSUFFICIENT_EXCITATION' if not excitation else 'REFINEMENT_MODEL_RESIDUAL')
    (output/'structure_selection.json').write_text(json.dumps(result,indent=2))
    return result
