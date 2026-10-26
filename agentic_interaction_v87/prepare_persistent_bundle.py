"""Offline IO/opaque-plant packaging for the two frozen user-selected joints.

No actor launch or grasp search. Hidden coefficients are generated in the
private scene role and never returned, printed, or copied into Agent jobs.
"""
import argparse,copy,hashlib,json,os,secrets
from pathlib import Path
import numpy as np
from agentic_interaction_v8.resources import export_bundle
from agentic_interaction_v81.blind_integration import PRIORS


def prepare(root,assembly,registration,descriptor):
    root=Path(root);root.mkdir(exist_ok=True,parents=True)
    reg=json.loads(Path(registration).read_text());assets=json.loads((Path(assembly)/'PREPARATION.json').read_text())['cases']
    original=json.loads(Path(descriptor).read_text());output_descriptor={k:v for k,v in original.items()if k!='scenes'}
    output_descriptor['scenes']={};public=[]
    for case in reg['fixed_cases']:
        aid=case['asset_id'];kind=case['joint_type'];a=next(x for x in assets if x['asset_id']==aid)
        manifest=json.loads((Path(a['asset_root'])/'manifest.json').read_text())
        source=root/'stations'/aid;source.mkdir(exist_ok=False,parents=True)
        source_report=dict(asset_installation=reg['asset_preparation']['installation'],robot_base_pose=reg['asset_preparation']['initial_station'])
        (source/'report.json').write_text(json.dumps(source_report,indent=2)+'\n')
        template=copy.deepcopy(original['scenes']['asset_45936'if kind=='revolute'else'asset_45746']['job_template'])
        for k in ('episode_id','deadline_shanghai','debug_budget_profile','passive_geometry_cameras','passive_rgbd','passive_dataset_output','passive_evaluation_output'):template.pop(k,None)
        template.update(asset_id=aid,research_reference='fixed-two-unseen-persistent-ABC',skill=dict(joint_type=kind),
            articulation_v7=dict(joint_type=kind,limits_native=[a['limits_native'][k]for k in ('lower','upper')],units=template['articulation_v7']['units']),
            benchmark_goal_native=case['goal_native'],benchmark_goal_rad=case['goal_native'],authored_target_rad=a['limits_native']['upper'],
            grasp_geometry_candidates=[a['selection']],visual_prompt='cabinet door handle'if kind=='revolute'else'drawer handle',
            runtime_acquisition_budget=6,maximum_agent_operations=100,agent_wait_timeout_s=600,
            episode_wall_budget_s=14400.,episode_sim_budget_s=14400.,safe_exit_wall_reserve_s=300.,safe_exit_sim_reserve_s=120.)
        template['estimation_v7']['domain_native']=[min(-.02*case['goal_native'],a['limits_native']['lower']),a['limits_native']['upper']]
        output_descriptor['scenes']['asset_'+aid]=dict(asset_root=a['asset_root'],station_report=str(source/'report.json'),job_template=template)
        private=root/'scene_private';private.mkdir(exist_ok=True,mode=0o700)
        rng=np.random.default_rng(secrets.randbits(64));law=PRIORS[kind]
        c=float(rng.uniform(*law['coulomb_nm'if kind=='revolute'else'coulomb_n']))
        b=float(rng.uniform(*law['viscous_nm_s_rad'if kind=='revolute'else'viscous_n_s_m']))
        plant=[dict(case=0,asset=aid,joint_type=kind,mass_scale=float(rng.uniform(*PRIORS['mass_scale'])),
            density_gradient=rng.uniform(*PRIORS['density_gradient'],3).tolist(),tau_c=c,b=b,spring=False,
            law=dict(family='constant',base=c,amplitude=0.,center=0.,width=1.))]
        path=private/(aid+'.json');path.write_text(json.dumps(plant,indent=2));path.chmod(0o400)
        public.append(dict(asset_id=aid,target_joint=a['target_joint'],joint_type=kind,physics_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            source_sha256=a['source_sha256'],prepared_sha256=a['prepared_sha256'],nonfixed_joints_retained=a['nonfixed_joints_retained'],
            limits_native=a['limits_native'],goal_native=case['goal_native'],selected_joint_commands=False))
    (root/'MIGRATION_DESCRIPTOR.json').write_text(json.dumps(output_descriptor,indent=2)+'\n')
    bundle=export_bundle(Path(__file__).resolve().parents[1],output_descriptor,root/'resources')
    (root/'PUBLIC_INPUT_COMMITMENTS.json').write_text(json.dumps(dict(cases=public,registered_shared_plant_family=PRIORS,
        dynamics_reveal='after both actors exit and model/online decisions seal; no online coefficient reads',
        bundle_sha256=hashlib.sha256((root/'resources/BUNDLE.json').read_bytes()).hexdigest(),
        no_old_grasp_view_or_motion_import=True),indent=2)+'\n')
    print('PACKAGED_FIXED_CASES',','.join(c['asset_id']for c in public),'hidden_coefficients_not_returned',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--assembly',type=Path,required=True)
    p.add_argument('--registration',type=Path,required=True);p.add_argument('--descriptor',type=Path,required=True)
    a=p.parse_args();prepare(a.root,a.assembly,a.registration,a.descriptor)
