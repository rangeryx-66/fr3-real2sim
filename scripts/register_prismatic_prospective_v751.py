"""One data-only prospective configuration after ordinary-handle repair.

Uses the existing V7.4 positive-density family. The private seed and values
are never printed or sent to Agent/estimator packets. Not a benchmark rerun.
"""
import argparse,hashlib,json,secrets
from pathlib import Path
import numpy as np


def register(root):
    root=Path(root);base=root/'results/prismatic_v751_20261022'
    private=base/'prospective/scene_private/CONFIGURATIONS.json'
    if private.exists():raise ValueError('NEVER_REGENERATE_A_FROZEN_CONFIGURATION')
    a=base/'episodes/v751_45290_transfer_01_interface_repair'
    seal=json.loads((a/'ONLINE_SEAL_V751.json').read_text())
    if not seal['actor_exited']or not seal['online_actions_finished']:raise ValueError('ENGINEERING_MODEL_NOT_SEALED')
    trace=json.loads((a/'AGENT_ACTION_TRACE.json').read_text())
    final=[r['result']for r in trace if r['command']['operation']=='attempt_grasp_and_probe'][-1]
    if not(final['safe_exit']['safe']and len(final['segments'])==4 and all(s['ok']for s in final['segments'])):
        raise ValueError('ORDINARY_HANDLE_CHAIN_NOT_READY')
    b=json.loads((base/'episodes/v751_difficult_exit_19855_00/PHYSICAL_UNLOADED_EXIT.json').read_text())
    if not b['safe_exit']['safe']:raise ValueError('PHYSICAL_EXIT_NOT_READY')
    seed=secrets.randbits(64);rng=np.random.default_rng(seed);C=float(rng.uniform(.2,.6))
    case=dict(case=0,asset='45290',joint_type='prismatic',seed=seed,
              mass_scale=float(rng.uniform(.75,1.25)),density_gradient=rng.uniform(-.3,.3,3).tolist(),
              tau_c=C,b=float(rng.uniform(1,6)),spring=False,
              law=dict(family='constant',base=C,amplitude=0.,center=.075,width=.03))
    private.parent.mkdir(parents=True,exist_ok=False);private.write_text(json.dumps([case],indent=2)+'\n');private.chmod(0o400)
    job=json.loads((base/'jobs/v751_45290_transfer_01_interface_repair.json').read_text())
    job.update(episode_id='v751_prospective_45290_00',identification_case=0,
               private_physics_configuration=str(private),
               research_reference='V7.5.1 single prospective new-dynamics check; NOT a formal benchmark',
               scientific_blinding='private scene generator + independent post-seal evaluator only; no hidden seed/parameters in Agent/worker inputs')
    job['output']=str(base/'episodes'/job['episode_id'])
    jp=base/'jobs'/(job['episode_id']+'.json');jp.write_text(json.dumps(job,indent=2)+'\n')
    docs=root/'docs/experiments/prismatic_v751_20261022'
    frozen=docs/'REGISTRATION_C1.json'
    reg=json.loads((frozen if frozen.exists()else docs/'REGISTRATION_C.json').read_text())
    reg.update(version='V7.5.1-D-single-prospective-constant-family',episode_id=job['episode_id'],
        classification='one same-development-object, new-hidden-dynamics prospective episode; not unseen-object or population success rate',
        job_sha256=hashlib.sha256(jp.read_bytes()).hexdigest(),
        private_configuration_file=str(private.relative_to(root)),
        private_configuration_sha256=hashlib.sha256(private.read_bytes()).hexdigest(),
        readiness=dict(ordinary_handle_seal_sha256=hashlib.sha256((a/'ONLINE_SEAL_V751.json').read_bytes()).hexdigest(),
                       physical_unloaded_exit=True,round_cap_milestone_separate=True),
        family_prior=dict(positive_density_mass_scale=[.75,1.25],density_gradient=[-.3,.3],C_N=[.2,.6],b_N_s_m=[1,6],resistance='constant family declared before generation; no state-dependent extrapolation claim'),
        execution=dict(episodes=1,valid_episode_retry=False,initial_request_N=1.,legal_requests_N=[.5,.75,1.,1.5],
                       fresh_wrist=True,automatic_grasp=True,Agent_feedback=True,maximum_acquisitions=3,
                       complete_validation_immediately_after_minimum_FIT=True,hidden_truth_after_model_and_actions_sealed=True))
    for name in ['interaction_runtime/prismatic_v751.py','tests/test_prismatic_v751.py','scripts/register_prismatic_prospective_v751.py']:
        reg['source_sha256'][name]=hashlib.sha256((root/name).read_bytes()).hexdigest()
    (docs/'REGISTRATION_D.json').write_text(json.dumps(reg,indent=2)+'\n')
    print(json.dumps(dict(registered_episodes=1,launched=False,hidden_values_printed=False,
                         commitment=reg['private_configuration_sha256'],job_sha256=reg['job_sha256'])))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);a=p.parse_args();register(a.root)
