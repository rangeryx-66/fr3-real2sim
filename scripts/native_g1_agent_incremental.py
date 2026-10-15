"""Observation-only development adapter; frozen Agent interface/motion reused."""
import inspect
import json
import time
import types
from pathlib import Path
from types import SimpleNamespace
import native_g1_agent_observation as frozen
import native_g1_incremental_geometry as incremental


def feedback(g):
    folders=[Path(r['folder']) for r in g['agent_observations'] if r['accepted_for_geometry']]
    if not folders:
        return dict(geometry=False, reason='partial cues retained for Agent inspection, not complete grasp geometry')
    started=time.perf_counter()
    out=g['a'].output/'incremental_feedback'/('%02d'%(len(g['agent_observations'])-1))
    data=incremental.build_geometry(folders[0],folders[1:],out)
    result=dict(geometry=True,geometry_folder=str(out),image_board=str(out/'incremental_surfaces.png'),
                feedback=data['incremental_feedback'],wall_s=time.perf_counter()-started,
                candidate_contact_support=dict(available=False,reason='no candidate yet; build_geometry runs unchanged automatic ranking'))
    plan=g['a'].output/'visual_plan.json'
    if plan.exists():
        # Passive, same previously generated candidate; never a selection gate.
        from evaluate_native_g1_multiview_predictions import first
        import trimesh
        import numpy as np
        candidate=json.loads(plan.read_text())
        mesh=trimesh.load(out/'observed_patches.ply',process=False)
        result['candidate_contact_support']=dict(available=True,
            measured_surface_prediction=first(g['model'],np.asarray(candidate['T']),candidate['base'],[mesh]),
            fixed_candidate_source=str(plan),GT_contact_surfaces_used=False)
    frozen.write(out/'AGENT_INCREMENTAL_FEEDBACK.json',result)
    return result


def observe(g):
    result=frozen.observe(g)
    try: result['incremental']=feedback(g)
    except Exception as error:
        result['incremental']=dict(diagnostic_issue=repr(error),scene_preserved=True)
    return result


def build_geometry(g):
    proxy=SimpleNamespace(**frozen.multi.__dict__)
    proxy.build_geometry=incremental.build_geometry
    fn=types.FunctionType(frozen.build_geometry.__code__,dict(frozen.__dict__,multi=proxy))
    result=fn(g)
    if result.get('plan'):
        p=json.loads((g['a'].output/'visual_plan.json').read_text())
        result['candidate_contact_support']=p.get('contact_geometry_audit',{})
        frozen.write(g['a'].output/'AGENT_GEOMETRY_PLAN_RESULT.json',result)
    return result


move_and_observe=types.FunctionType(frozen.move_and_observe.__code__,dict(frozen.__dict__,observe=observe))


def prepare(g):
    # No execute operation exists. Same bounded four-view Agent interface,
    # with cooperative export if interaction cannot continue.
    source=inspect.getsource(frozen.prepare)
    source=source.replace('while not path.exists():',
        "while not path.exists():\n            if time.perf_counter()-wait_started>g['job'].get('agent_wait_timeout_s',900):raise KeyboardInterrupt('AGENT_INPUT_TIMEOUT_SAFE_EXPORT')")
    ns=dict(frozen.__dict__,observe=observe,build_geometry=build_geometry,move_and_observe=move_and_observe)
    exec(compile(source,__file__+'::frozen_agent_interface_incremental_input','exec'),ns)
    return ns['prepare'](g)
