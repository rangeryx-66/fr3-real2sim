"""One-shot blind 8-object / 16-episode registration. Does not launch physics.

Run only after native-contact adapter regressions. The Agent may read the
public commitment/protocol, never scene_private/CONFIGURATIONS.json.
"""
import argparse,hashlib,json,secrets,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def register(root,implementation_commit,execution_scope='all'):
    if execution_scope not in ('all','revolute_first'):raise ValueError('Unknown execution scope')
    root=Path(root).resolve();dest=root/'registration'
    if dest.exists():raise ValueError('Registration exists: do not replace seeds/cases after outcomes')
    pre=json.loads((root/'preflight/PROVISIONAL_CASES.json').read_text());stations={r['asset_id']:r for r in json.loads((root/'preflight/STATION_IK.json').read_text())}
    objects=[];episodes=[];private=[]
    for row in pre['cases']:
        aid=row['asset_id'];station=stations[aid]
        if not station['coarse_IK'] or not any(r.get('home_collision_valid') for r in station['audit']):raise ValueError('Prephysical station/IK unresolved: '+aid)
        job=json.loads((root/'inputs'/aid/'job_template.json').read_text());m=json.loads((Path(job['asset_root'])/'manifest.json').read_text());kind=row['joint_type']
        objects.append(dict(asset_id=aid,joint_type=kind,joint_name=row['joint_name'],coordinate_unit=row['unit'],limits_native=row['limits_native'],registered_contact_mesh=station['mesh'],registered_contact_body=station['handle_link'],base_station=station['station'],source_urdf_sha256=row['geometry_sha256'],collision_representation=m.get('collision_representation','frozen prepared per-source-piece convex proxies; no new collider change'),geometry_used_for_station_preflight=True,online_grasp_geometry='fresh wrist RGB-D/SAM3 only',saved_grasp_or_view_input=False))
        # One of each family per object, secretly shuffled; independent seeded
        # mass/density/dynamics draws. No family label is exposed per episode.
        families=['constant','localized_passive_friction']
        if secrets.randbits(1):families.reverse()
        for j,family in enumerate(families):
            seed=secrets.randbits(64);rng=np.random.default_rng(seed);linear=kind=='prismatic'
            C=float(rng.uniform(.2,.6) if linear else rng.uniform(.018,.03));b=float(rng.uniform(1.,6.) if linear else rng.uniform(.1,.35));span=job['benchmark_goal_native']
            case=dict(case=len(private),asset=aid,joint_type=kind,seed=seed,mass_scale=float(rng.uniform(.75,1.25)),density_gradient=rng.uniform(-.3,.3,3).tolist(),tau_c=C,b=b,spring=False,
                law=dict(family=family,base=C,amplitude=0. if family=='constant' else float(rng.uniform(.3,.65)*C),center=float(rng.uniform(.2,.8)*span),width=float(rng.uniform(.10,.25)*span)))
            private.append(case);episodes.append(dict(episode_id=f'{aid}_{j}',asset_id=aid,configuration_index=case['case'],joint_type=kind,status='NOT_EXECUTED',
                execution_authorized=execution_scope=='all' or kind=='revolute',
                deferral_reason='PRISMATIC_ENGINEERING_UNRESOLVED' if execution_scope=='revolute_first' and linear else None))
    if len(objects)!=8 or sum(o['joint_type']=='revolute' for o in objects)!=4 or len(episodes)!=16:raise ValueError('Expected four doors, four drawers, two configurations each')
    dest.mkdir();hidden=root/'scene_private';hidden.mkdir(exist_ok=True);truth=hidden/'CONFIGURATIONS.json'
    if truth.exists():raise ValueError('Preserve existing hidden configuration file')
    truth.write_text(json.dumps(private,indent=2));truth.chmod(0o400)
    from run_angle_resistance_v5 import SOURCES
    files=set(SOURCES)|{str(p.relative_to(ROOT)) for p in (ROOT/'interaction_runtime').glob('*.py') if not p.name.startswith('._')}
    files|={str(p.relative_to(ROOT)) for p in (ROOT/'scripts').glob('*v7*.py')}
    files|={'scripts/xarm_ft_id_scene.py','scripts/xarm_ft_known_joint_prior.py','scripts/xarm_ft_v31_estimator.py','scripts/native_g1_execution_candidates.py','scripts/native_g1_contact_recovery.py','scripts/native_g1_robust_contact_ranking.py','scripts/native_g1_safe_park.py','scripts/plan_shared_door_skill.py','scripts/diagnose_probing_v3.py','scripts/agent_command_protocol_v12.py','configs/interaction_runtime_v1.json'}
    files.add('docs/experiments/cross_object_v7_20261020/PROTOCOL.md')
    frozen={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in sorted(files)}
    protocol=dict(reference='25e2189',implementation_commit=implementation_commit,objects=objects,episodes=episodes,source_sha256=frozen,hidden_configuration_sha256=hashlib.sha256(truth.read_bytes()).hexdigest(),
        parameter_family=dict(mass_scale=[.75,1.25],positive_density_gradient=[-.3,.3],revolute=dict(C_Nm=[.018,.03],b_Nm_s_rad=[.1,.35]),prismatic=dict(C_N=[.2,.6],b_N_s_m=[1.,6.]),localized_amplitude_fraction=[.3,.65],localized_center_fraction_target=[.2,.8],localized_width_fraction_target=[.1,.25],family_assignment='one constant and one smooth passive profile per object; shuffled assignment hidden'),
        exclusions=pre['prephysical_exclusions'],recovery=dict(maximum_handle_acquisitions=4,maximum_Agent_operations=40,selected_view_adjustments=4,policy='failure -> checked unloaded release/park -> inspect fresh wrist/contact feedback -> regenerate/evaluate ranked feasible alternatives; no unchanged retry, no first-invalid-candidate abandonment; end only for safety, exhausted feasible alternatives or finite resources including exit reserve'),
        resource_budget=dict(episode_wall_budget_s=5400.,episode_sim_budget_s=2400.,safe_exit_wall_reserve_s=300.,safe_exit_sim_reserve_s=90.),
        evaluation='predeclared FIT, independent complete VALIDATION; no incomplete VALIDATION or hidden truth in fitting/model selection; optional post-VALIDATION FIT reported separately from sealed primary model',
        execution_scope=execution_scope,
        execution_scope_reason='User authorized the preselected four-door subset if the current drawer engineering tests remain unresolved; no replacement objects or extra drawer tuning' if execution_scope=='revolute_first' else 'All sixteen registered episodes',
        blinding='scene-generator/evaluator only; same-account procedural separation, not adversarial OS isolation; reveal completed subset only after all its online decisions finish; deferred configuration truth remains hidden',
        scope='CODEX_IN_LOOP / WRIST_RGBD_SAM3 / KNOWN_JOINT_STATE / APPROXIMATE_ACTUATORS / IDEAL_WRIST_FT; registered native collision/kinematic station dependency; bar-handle ordinary objects, not unseen population validation',
        evidence='essential 240Hz numerical streams and checkpoint RGB-D; no continuous video/passive cameras; target <100MB per episode; raw private')
    (dest/'BENCHMARK.json').write_text(json.dumps(protocol,indent=2));print(json.dumps(dict(registered_objects=8,registered_episodes=16,configuration_commitment=protocol['hidden_configuration_sha256'],hidden_values_printed=False)))
    return protocol


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--implementation-commit',required=True);p.add_argument('--execution-scope',choices=['all','revolute_first'],default='all');a=p.parse_args();register(a.root,a.implementation_commit,a.execution_scope)
