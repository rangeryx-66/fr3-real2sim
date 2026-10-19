"""Thin incremental-observation binding to the frozen explicit physical handoff.

No controller, scoring or perception algorithm changes. execute() below adds
passive input provenance only; the existing handoff owns return/approach checks.
"""
import hashlib,json,types
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import native_g1_agent_observation as observation
import native_g1_agent_execution as physical
import native_g1_agent_incremental_activation as incremental


def input_provenance(g,stage):
    """Diagnostics cannot veto the already approved physical experiment."""
    try:
        root=g['a'].output;folder=root/'agent_geometry'
        files=['observed_geometry.json','observed_patches.ply','all_measured_patches.ply',
               'uncertain_association_patches.ply','points_world.npy',
               'contact_supported_points_world.npy','semantic_pixel_provenance.npz',
               'measured_provenance.npz','uncertain_rear_0.ply','uncertain_rear_1.ply','uncertain_rear_2.ply']
        hashes={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in files if (folder/n).exists()}
        plan=root/'visual_plan.json';candidate=json.loads(plan.read_text())
        result=dict(stage=stage,geometry_folder=str(folder),geometry_sha256=hashes,
                    selected_plan_sha256=hashlib.sha256(plan.read_bytes()).hexdigest(),
                    selected_T=candidate['T'],contact_supported_mesh=str(folder/'observed_patches.ply'),
                    measured_context_mesh=str(folder/'all_measured_patches.ply'),
                    semantic_uncertainty_mesh=str(folder/'uncertain_association_patches.ply'),
                    ranking='unchanged multiview rank_surfaces/closing_scene reads this incremental folder',
                    GT_handle_surface_used=False,old_first_seed_fusion_called=False,
                    actual_T=np.asarray(g['T']).tolist() if 'T' in g else None,
                    whole_T=g.get('whole',{}).get('T'))
        observation.write(root/('INCREMENTAL_EXECUTION_INPUT_'+stage+'.json'),result)
    except Exception as error:
        g['diagnostic_issue']('incremental_execution_input_provenance',error)


def execute(g,command):
    input_provenance(g,'before')
    result=physical.execute(g,command)
    input_provenance(g,'after')
    return result


# Preserve original motion globals (including actuate) inside the repaired
# callback. The existing physical interface supplies validate_execution/execute.
proxy=SimpleNamespace(**observation.__dict__)
proxy.observe=incremental.observe
proxy.move_and_observe=incremental.move_and_observe
proxy.build_geometry=incremental.build_geometry
prepare=types.FunctionType(physical.prepare.__code__,dict(physical.__dict__,observation=proxy,execute=execute))
validate_execution=physical.validate_execution
