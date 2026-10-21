"""Data-only next-benchmark preregistration. Never imports/launches Isaac.

Fresh seeds and authored values go only to scene_private. This does not bypass
the V7.3 engineering runner's explicit prohibition on registered evaluation.
Activation of a registered launcher is a separate authorized step.
"""
import argparse,hashlib,json,secrets,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]


def register(engineering,reference,output,commit):
    engineering=Path(engineering);output=Path(output)
    ready=json.loads((engineering/'READINESS_REVIEW.json').read_text())
    if not ready.get('ready_for_independent_benchmark'):raise ValueError('Engineering readiness not established')
    if output.exists():raise ValueError('Preserve preregistration: never replace seeds/configurations')
    freeze=engineering/'SOURCE_FREEZE.json';frozen=json.loads(freeze.read_text())['sources']
    if not frozen or any(not (ROOT/p).exists() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h for p,h in frozen.items()):
        raise ValueError('Frozen engineering source mismatch; no configuration generated')
    prior=json.loads(Path(reference).read_text());objects=[o for o in prior['objects'] if o['joint_type']=='prismatic']
    if len(objects)!=4 or {o['asset_id'] for o in objects}!={'19179','45290','27044','45746'}:raise ValueError('Frozen four-drawer split required')
    reviewed=ready['cases']
    if not all(reviewed[o['asset_id']]['acquired'] and reviewed[o['asset_id']]['reverse'] and reviewed[o['asset_id']]['complete_validation'] and reviewed[o['asset_id']]['safe_park'] for o in objects):raise ValueError('Missing physical readiness evidence')
    private=[];episodes=[]
    for o in objects:
        job=json.loads((engineering/'jobs'/(o['asset_id']+'_fresh_00.json')).read_text());span=float(job['benchmark_goal_native'])
        families=['constant','localized_passive_friction']
        if secrets.randbits(1):families.reverse()
        for j,family in enumerate(families):
            seed=secrets.randbits(64);rng=np.random.default_rng(seed);C=float(rng.uniform(.2,.6))
            case=dict(case=len(private),asset=o['asset_id'],joint_type='prismatic',seed=seed,mass_scale=float(rng.uniform(.75,1.25)),density_gradient=rng.uniform(-.3,.3,3).tolist(),tau_c=C,b=float(rng.uniform(1,6)),spring=False,law=dict(family=family,base=C,amplitude=0. if family=='constant' else float(rng.uniform(.3,.65)*C),center=float(rng.uniform(.2,.8)*span),width=float(rng.uniform(.10,.25)*span)))
            private.append(case);episodes.append(dict(episode_id='v74_'+o['asset_id']+'_'+str(j),asset_id=o['asset_id'],configuration_index=case['case'],joint_type='prismatic',status='REGISTERED_NOT_EXECUTED',execution_authorized=False))
    output.mkdir(parents=True);hidden=output/'scene_private';hidden.mkdir();truth=hidden/'CONFIGURATIONS.json';truth.write_text(json.dumps(private,indent=2));truth.chmod(0o400)
    paths=list((ROOT/'interaction_runtime').glob('*.py'))+[ROOT/'configs/interaction_runtime_v1.json',ROOT/'configs/prismatic_preload_v71.json',ROOT/'configs/sensor_lighting_v1.json']
    paths+=list((ROOT/'scripts').glob('*prismatic*v73*.py'))+[ROOT/'scripts/register_prismatic_v74.py',ROOT/'scripts/xarm_ft_v7_estimator.py',ROOT/'scripts/xarm_ft_v31_estimator.py',ROOT/'scripts/update_probing_v7_model.py']
    # Finder AppleDouble sidecars are resource metadata, not Python modules.
    sources=dict(frozen)
    sources.update({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if not p.name.startswith('._')})
    result=dict(schema='prismatic-v74-preregistration',implementation_commit=commit,objects=objects,episodes=episodes,source_sha256=sources,hidden_configuration_sha256=hashlib.sha256(truth.read_bytes()).hexdigest(),readiness_review_sha256=hashlib.sha256((engineering/'READINESS_REVIEW.json').read_bytes()).hexdigest(),split=dict(objects='four development objects; NOT unseen-object transfer',configurations='two fresh independent blinded configurations per object; one constant and one smooth localized family, shuffled assignment hidden',validation='complete predeclared distinct-profile trajectory; excluded from FIT and model selection'),family_prior=dict(mass_scale=[.75,1.25],positive_density_gradient=[-.3,.3],C_N=[.2,.6],b_N_s_m=[1,6],localized_amplitude_fraction=[.3,.65],localized_center_fraction_goal=[.2,.8],localized_width_fraction_goal=[.10,.25]),recovery=dict(initial_requested_preload_n=1.,legal_requests_n=[.5,.75,1.,1.5],maximum_acquisitions=4,maximum_operations=40,maximum_view_adjustments_per_generation=4,hard_stop_authoritative=True,Agent_decisions_only_after_verified_unloaded_boundary=True),resources=dict(wall_s=3600,simulation_s=1500,exit_reserve_wall_s=300,exit_reserve_sim_s=120,unloaded_Agent_deadline_s=360,evidence_target_bytes=100000000),metrics=dict(range='actual pull/reverse, legacy80mm stage retained separately from useful local identification; do not force unsafe80–150mm',interaction='first acquisition, transmitting pull/reverse, recovery attempts/outcomes, FIT, complete VALIDATION, release and parking; all registered denominators',identification='mass/damping/resistance error, conditional and cross-FIT uncertainty, excitation/rank/conditioning, complete held-out effort and free position/velocity prediction; unsupported parameters count as unidentified',resistance_support='observed FIT support only; no extrapolated curve accuracy',primary='qualified bidirectional FIT plus complete independent VALIDATION; no fit-only/short-motion-only PASS'),execution=dict(launched=False,registered_launcher_activated=False,core_entry='scripts/run_prismatic_v73.py',core_entry_current_scope='engineering-only; cannot silently execute these registered slots',activation='requires separately authorized registered launcher activation against these frozen source/physics commitments, before any outcome; no automatic research run'),blinding='same-account procedural separation; seeds/parameters withheld from Agent and estimator until all online actions/models are sealed',preservation='all original sixteenV7.2 results and71+22revolute hashes unchanged; no original slot reused',scientific_boundary='CODEX_IN_LOOP / WRIST_RGBD_SAM3 / KNOWN_RAIL_STATE / REGISTERED_STATION / APPROXIMATE_ACTUATORS / IDEAL_WRIST_FT')
    (output/'BENCHMARK.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(registered_episodes=8,hidden_values_printed=False,launched=False,commitment=result['hidden_configuration_sha256'])));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--engineering-root',type=Path,required=True);p.add_argument('--reference-registration',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--implementation-commit',required=True);a=p.parse_args();register(a.engineering_root,a.reference_registration,a.output,a.implementation_commit)
