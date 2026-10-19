"""Three-way offline evaluation; never imported by the live Agent.

A=a47fe09, B=db7ccaa, C=observation-based semantic uncertainty partition.
GT is restricted to evaluate/context/materiality. Planning uses observed inputs.
"""
import argparse,hashlib,inspect,json,shutil,time,types
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh
import evaluate_incremental_wrist as old


def links(case,out):
    source=old.ROOT/'results/native_g1_incremental_wrist_20261016/offline'/case
    out.mkdir(parents=True,exist_ok=True)
    for name in ['A','B']:
        p=out/name
        if not p.exists():p.symlink_to((source/name).resolve(),target_is_directory=True)


def evaluate(case,out,assembly):
    for name in ['evaluate','context']:
        source=inspect.getsource(getattr(old,name)).replace("['A','B']","['A','B','C']")
        if name=='context':source=source.replace("output/'B'","output/'C'")
        ns=dict(old.__dict__);exec(source,ns)
        if name=='context':ns[name](case,out,assembly)
        else:ns[name](case,out)
    from xarm7_native_planning import NativeModel
    from rank_prepared_handle_assets import load
    from visualize_native_g1_gt_observed import distance,stats
    parent=old.CASES[case];job=json.loads((parent/'job.json').read_text())
    model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
    _,parts=load(model.asset_urdf);names=assembly or [job['grasp_geometry_candidates'][0]['mesh']]
    meshes=[]
    for p in parts:
        if p[1]=='visual' and p[2] in names:
            m=p[3].copy();m.apply_transform(model.asset_T);meshes.append(m)
    M=trimesh.util.concatenate(meshes);P=np.load(out/'C/contact_supported_points_world.npy');d=distance(M,P)
    result=dict(contact_supported_error=stats(d),contact_supported_over3mm_points=int(np.sum(d>.003)),
                all_measured_points_preserved=np.array_equal(np.load(out/'B/points_world.npy'),np.load(out/'C/points_world.npy')),
                C_semantic_association=json.loads((out/'C/observed_geometry.json').read_text())['semantic_association'])
    (out/'SEMANTIC_EVALUATION.json').write_text(json.dumps(result,indent=2));print(case,result,flush=True)


def plan(case,out):
    source=old.ROOT/'results/native_g1_incremental_wrist_20261016/offline'/case
    files=['observed_patches.ply','uncertain_rear_0.ply','uncertain_rear_1.ply','uncertain_rear_2.ply']
    equal={n:hashlib.sha256((out/'C'/n).read_bytes()).hexdigest()==hashlib.sha256((source/'B'/n).read_bytes()).hexdigest() for n in files}
    before=json.loads((source/'RANKING_COMPARISON.json').read_text())
    if all(equal.values()):
        result=dict(found=True,planning_reused_byte_identical_inputs=True,geometry_equal=equal,
                    A=before['A'],B=before['B'],C=before['B'],wall_s=0.,original_B_wall_s=before['wall_s'])
    else:
        from xarm7_native_planning import NativeModel
        from native_g1_multiview_input import plan_observed
        parent=old.CASES[case];job=json.loads((parent/'job.json').read_text())
        model=NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
        path=out/'C_plan';path.mkdir(exist_ok=True)
        shutil.copyfile(parent/'episode/cooked_initial.json',path/'cooked_initial.json')
        g=dict(job=job,model=model,a=SimpleNamespace(output=path),allowed=before['B']['allowed_authored_colliders'] if 'allowed_authored_colliders' in before['B'] else json.loads((parent/'episode/visual_plan.json').read_text())['allowed_authored_colliders'])
        start=time.perf_counter();candidate,_=plan_observed(g,out/'C')
        result=dict(found=candidate is not None,planning_reused_byte_identical_inputs=False,geometry_equal=equal,
                    A=before['A'],B=before['B'],C=candidate,wall_s=time.perf_counter()-start)
    (out/'RANKING_COMPARISON.json').write_text(json.dumps(result,indent=2));print('PLAN_DONE',case,result['found'],result['wall_s'],flush=True)


def candidate_evaluate(case,out):
    """Post-hoc contact support at each already selected pose; no corrections."""
    import xarm7_native_planning as native
    from handle_contact_quality import handle_surfaces
    from revolute10_fixed import authored_handle_colliders
    from evaluate_native_g1_multiview_predictions import first
    parent=old.CASES[case];job=json.loads((parent/'job.json').read_text())
    model=native.NativeModel(None,job['asset_root'],json.loads((Path(job['source'])/'report.json').read_text()))
    export=json.loads((parent/'episode/cooked_initial.json').read_text());selection=job['grasp_geometry_candidates'][0]
    gt,cooked=handle_surfaces(model,export,authored_handle_colliders(export,selection),selection)
    from native_g1_multiview_input import fast_ray
    ns=dict(native.__dict__,ray_surface=fast_ray)
    source=inspect.getsource(native.common_quality).replace(' from handle_contact_quality import ray_surface\n','')
    exec(source,ns)
    comparison=json.loads((out/'RANKING_COMPARISON.json').read_text());rows=[]
    link=model.manifest['moving_link'];joint=model.manifest['joint_name']
    J=model.asset_T@model.asset.root_to_link(link,{joint:0.});axis=J[:3,:3]@model.asset.joints[link].axis
    for mode in ['A','B','C']:
        c=comparison[mode];T=np.asarray(c['T']);tangent=np.cross(axis,T[:3,3]-J[:3,3])
        observed=trimesh.load(out/mode/'observed_patches.ply',process=False)
        support=ns['common_quality'](model,(gt,cooked),T,c['base'],tangent)
        rows.append(dict(mode=mode,T=T.tolist(),
            predicted_observed=first(model,T,c['base'],[observed]),
            GT_native_evaluation_only=first(model,T,c['base'],cooked,True),
            GT_common_aperture_support_evaluation_only=support['score'],
            GT_best_common_aperture_m=support['native']['best_common_pose_aperture']['common_aperture_m']))
    (out/'SELECTED_CANDIDATE_EVALUATION.json').write_text(json.dumps(dict(rows=rows,
        GT_use='diagnostic only after selection; poses never corrected; no calibrated force/success claim'),indent=2))
    print('CANDIDATE_EVAL_DONE',case,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',choices=old.CASES,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--stage',choices=['evaluate','plan','candidate_evaluate'],required=True)
    p.add_argument('--assembly',nargs='+');a=p.parse_args();links(a.case,a.output)
    if a.stage=='evaluate':evaluate(a.case,a.output,a.assembly)
    elif a.stage=='plan':plan(a.case,a.output)
    else:candidate_evaluate(a.case,a.output)
