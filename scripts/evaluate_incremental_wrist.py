"""Paired archived RGB-D study. GT is opened only in --stage evaluate.

--stage build/plan never accesses handle meshes for reconstruction/contact
scoring. Known scene collision checks remain an explicit planner dependency.
"""
import argparse
import hashlib
import json
import shutil
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import native_g1_multiview_input as frozen
import native_g1_incremental_geometry as incremental
from diagnose_incremental_wrist import accepted, replay

ROOT = Path(__file__).resolve().parents[1]
CASES = {
    '45936': ROOT/'results/native_g1_agent_transfer_20261015/45936',
    '46427': ROOT/'results/native_g1_agent_transfer_20261015/46427',
    '45600': ROOT/'results/native_g1_agent_manipulation_20261015',
}


def build(case, output):
    episode = CASES[case]/'episode'
    replay(episode, output/'diagnosis')
    folders = accepted(episode)
    started = time.perf_counter()
    old = frozen.build_geometry(folders[0], folders[1:], output/'A')
    elapsed_a = time.perf_counter()-started; started = time.perf_counter()
    new = incremental.build_geometry(folders[0], folders[1:], output/'B')
    elapsed_b = time.perf_counter()-started
    archived = np.load(episode/'agent_geometry/points_world.npy')
    reproduced = np.load(output/'A/points_world.npy')
    result = dict(case=case, archived_points=len(archived),
                  reproduced_frozen_points=len(reproduced),
                  frozen_replay_max_nearest_difference_m=float(cKDTree(archived).query(reproduced)[0].max()),
                  A_extent_mm=old['dimensions_m'][0]*1000,
                  B_extent_mm=new['dimensions_m'][0]*1000,
                  A_points=old['observed_points'], B_points=new['observed_points'],
                  A_wall_s=elapsed_a, B_wall_s=elapsed_b,
                  contributions=new['incremental_feedback']['views'],
                  input_sha256=json.loads((output/'diagnosis/DIAGNOSIS.json').read_text())['input_sha256'],
                  GT_geometry_inputs=False)
    (output/'PAIRED_BUILD.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


def plan(case, output):
    from xarm7_native_planning import NativeModel
    parent = CASES[case]; job = json.loads((parent/'job.json').read_text())
    source = json.loads((Path(job['source'])/'report.json').read_text())
    model = NativeModel(None,job['asset_root'],source)
    archived = json.loads((parent/'episode/visual_plan.json').read_text())
    # Archive A ranking already exists. B uses exactly the same candidate
    # families, weights, station and per-case archived planning budget.
    path = output/'new_plan'; path.mkdir(exist_ok=True)
    shutil.copyfile(parent/'episode/cooked_initial.json',path/'cooked_initial.json')
    g = dict(job=job,model=model,a=SimpleNamespace(output=path),
             allowed=archived['allowed_authored_colliders'])
    started=time.perf_counter(); candidate,_=frozen.plan_observed(g,output/'B')
    fields=['section_fraction','insertion_m','roll_deg','pitch_deg','T','base',
            'planned_reached_state','minimum_approach_arc_margin_rad','contact_geometry_audit']
    summary=dict(case=case,found=candidate is not None,wall_s=time.perf_counter()-started,
                 A={k:archived.get(k) for k in fields},
                 B={k:candidate.get(k) for k in fields} if candidate else None,
                 weights_and_candidate_families_changed=False, GT_handle_scoring=False,
                 station_source='unchanged registered known-model station')
    (output/'RANKING_COMPARISON.json').write_text(json.dumps(summary,indent=2))
    print('PLAN_DONE',case,summary['found'],summary['wall_s'],flush=True)


def evaluate(case,output):
    from xarm7_native_planning import NativeModel
    from revolute10_fixed import authored_handle_colliders
    from handle_contact_quality import handle_surfaces
    from evaluate_native_g1_multiview_predictions import first
    from visualize_native_g1_gt_observed import sample,distance,stats,project,pca
    parent=CASES[case];episode=parent/'episode'
    job=json.loads((parent/'job.json').read_text())
    source=json.loads((Path(job['source'])/'report.json').read_text())
    model=NativeModel(None,job['asset_root'],source)
    export=json.loads((episode/'cooked_initial.json').read_text())
    selection=job['grasp_geometry_candidates'][0]
    gt,native=handle_surfaces(model,export,authored_handle_colliders(export,selection),selection)
    G=sample(gt,30000);axis=pca(G)
    visible=np.zeros(len(G),bool)
    for folder in accepted(episode):
        meta=json.loads((folder/'camera.json').read_text());depth=np.load(folder/'depth_m.npy')
        xy,valid,dz=project(G,np.asarray(meta['K']),np.asarray(meta['T_world_camera_optical']),depth)
        selfmask=np.load(folder/'robot_self_exclusion.npy')
        valid[valid]&=~selfmask[xy[valid,1],xy[valid,0]]
        visible |= valid & (abs(dz)<=.002)
    archived=json.loads((episode/'visual_plan.json').read_text())
    correction=episode/'visual_correction.json'
    T=np.asarray(json.loads(correction.read_text())['T_after'] if correction.exists() else archived['T'])
    rows=[]
    for mode in ['A','B']:
        folder=output/mode;P=np.load(folder/'points_world.npy')
        measured=trimesh.load(folder/'observed_patches.ply',process=False)
        d=distance(gt,P);nearest=cKDTree(P).query(G)[0]
        predictions=dict(measured=first(model,T,archived['base'],[measured]))
        for i,scale in enumerate(frozen.SPEC['rear_scales']):
            rear=trimesh.load(folder/('uncertain_rear_%d.ply'%i),process=False)
            predictions[str(scale)]=first(model,T,archived['base'],[measured,rear])
        rows.append(dict(mode=mode,observed_error=stats(d),
                         observed_extent_along_GT_mm=float(np.ptp(P@axis)*1000),
                         GT_surface_samples_visible=int(visible.sum()),
                         visible_surface_coverage_within_2mm=float(np.mean(nearest[visible]<=.002)),
                         all_GT_surface_coverage_within_2mm=float(np.mean(nearest<=.002)),
                         observed_points_over_3mm_fraction=float(np.mean(d>.003)),
                         observed_points_over_5mm_fraction=float(np.mean(d>.005)),
                         observed_points_over_10mm_fraction=float(np.mean(d>.010)),
                         predictions_at_same_frozen_TCP=predictions))
    result=dict(case=case,rows=rows,GT_use='separate post-hoc evaluator only',
                coverage='30k area-weighted deterministic GT samples; 2mm diagnostic, never an execution gate',
                contamination='distance outliers are a proxy, not automatically background labels; ornamental mounts differ from registered central handle',
                GT_contact=dict(visual=first(model,T,archived['base'],[gt]),
                                native=first(model,T,archived['base'],native,True)))
    (output/'INDEPENDENT_GT_EVALUATION.json').write_text(json.dumps(result,indent=2))
    print('EVALUATION_DONE',case,flush=True)


def context(case,output,evaluation_meshes=None):
    """Post-hoc nearest visual-part labels; ornaments are not background noise."""
    from xarm7_native_planning import NativeModel
    from rank_prepared_handle_assets import load
    from visualize_native_g1_gt_observed import distance,stats
    parent=CASES[case];job=json.loads((parent/'job.json').read_text())
    model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
    selection=job['grasp_geometry_candidates'][0]
    _,parts=load(model.asset_urdf)
    parts=[p for p in parts if p[0]==selection['handle_link'] and p[1]=='visual']
    meshes=[]
    for part in parts:
        m=part[3].copy();m.apply_transform(model.asset_T);meshes.append(m)
    assembly_names=evaluation_meshes or [selection['mesh']]
    assembly=trimesh.util.concatenate([m for p,m in zip(parts,meshes) if p[2] in assembly_names])
    rows=[]
    for mode in ['A','B']:
        P=np.load(output/mode/'points_world.npy')
        distances=np.array([distance(m,P) for m in meshes])
        which=distances.argmin(0);nearest=distances.min(0)
        labels={p[2]:dict(points=int(np.sum(which==i)),
                         surface_error=stats(nearest[which==i])) for i,p in enumerate(parts) if np.any(which==i)}
        assembly_distance=distance(assembly,P)
        rows.append(dict(mode=mode,nearest_visual_part=labels,
                         nearest_moving_link_surface_error=stats(nearest),
                         declared_handle_assembly_error=stats(assembly_distance),
                         assembly_outliers_over_3mm=int(np.sum(assembly_distance>.003)),
                         assembly_max_error_mm=float(assembly_distance.max()*1000),
                         points_over_5mm_from_any_moving_visual_fraction=float(np.mean(nearest>.005))))
    per_view=[]
    for folder in sorted((output/'B').glob('*/preserved_points_world.npy')):
        P=np.load(folder);d=distance(assembly,P)
        per_view.append(dict(view=folder.parent.name,points=len(P),error=stats(d),
                             outliers_over_3mm=int(np.sum(d>.003))))
    (output/'GT_PART_CONTEXT.json').write_text(json.dumps(dict(case=case,rows=rows,
        per_view=per_view,evaluation_only_assembly_meshes=assembly_names,
        GT_evaluation_only=True,note='Nearest-part labels distinguish central shaft, support/cap and panel. These labels never enter reconstruction.'),indent=2))
    print('CONTEXT_DONE',case,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',choices=CASES,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stage',choices=['build','plan','evaluate','context'],required=True)
    p.add_argument('--evaluation-meshes',nargs='+',help='Separate GT evaluator only: central shaft/support/cap mesh identities')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    if a.stage=='context':context(a.case,a.output,a.evaluation_meshes)
    else:globals()[a.stage](a.case,a.output)
