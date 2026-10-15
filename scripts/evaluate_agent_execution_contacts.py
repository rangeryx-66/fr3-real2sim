"""Independent post-exit first-contact comparison at the refined visual pose."""
import argparse,json
from pathlib import Path
import numpy as np
import trimesh
from xarm7_native_planning import NativeModel
from evaluate_native_g1_multiview_predictions import first
from handle_contact_quality import handle_surfaces
from revolute10_fixed import authored_handle_colliders

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();r=a.root;run=r/'episode'
    if not (r/'PROCESS_EXIT.json').exists():raise SystemExit('Offline evaluator only after actor exit')
    job=json.loads((r/'job.json').read_text());model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
    plan=json.loads((run/'visual_plan.json').read_text());correction=json.loads((run/'visual_correction.json').read_text());export=json.loads((run/'cooked_initial.json').read_text())
    selection=job['grasp_geometry_candidates'][0];gt,native=handle_surfaces(model,export,authored_handle_colliders(export,selection),selection)
    observed=trimesh.load(run/'agent_geometry/observed_patches.ply',process=False);rear=trimesh.load(run/'agent_geometry/uncertain_rear_1.ply',process=False);T=np.asarray(correction['T_after'])
    result=dict(evaluation_only_after_exit=True,executable_grasp_never_changed_from_GT=True,pose='frozen wrist RGB-D pregrasp-refined target; no physical centering included',T=T.tolist(),observed_only=first(model,T,plan['base'],[observed]),observed_plus_nominal_rear=first(model,T,plan['base'],[observed,rear]),GT_visual=first(model,T,plan['base'],[gt]),GT_native=first(model,T,plan['base'],native,True),actual_first_contacts=json.loads((r/'analysis/telemetry/METRICS.json').read_text())['first_contacts'],note='Whole-finger geometric intersections; first positive normal contact is not bilateral acquisition. Physical right contact follows original TCP centering, so its difference is not a fixed-pose sensor error.')
    (r/'analysis/REFINED_CONTACT_COMPARISON.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ['observed_plus_nominal_rear','GT_native','actual_first_contacts']},indent=2))
