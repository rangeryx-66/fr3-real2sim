"""Scene-generator role only: fresh opaque plant commitments, no actor launch.

Uses the existing V7 positive-density/passive-resistance family. This module is
not imported by the Agent session, controller or numeric worker. Blinding is
role/protocol separation, not an OS security claim.
"""
import argparse,hashlib,json,os,secrets
from pathlib import Path

PRIORS=dict(mass_scale=[.75,1.25],density_gradient=[-.3,.3],
    revolute=dict(coulomb_nm=[.018,.030],viscous_nm_s_rad=[.1,.35]),
    prismatic=dict(coulomb_n=[.2,.6],viscous_n_s_m=[1.,6.]),
    resistance_family='constant native passive friction; no spring or joint drive')


def prepare(root):
    import numpy as np
    root=Path(root);root.mkdir(parents=True,exist_ok=False);os.chmod(root,0o700)
    public=dict(schema='v81-full-range-blind-integration',priors=PRIORS,
        truth_reveal='only after actor exit, all online decisions and model sealing',episodes=[])
    for kind,asset,scene in [('revolute','45936','cabinet_bar'),('prismatic','45290','drawer_bar')]:
        seed=secrets.randbits(64);rng=np.random.default_rng(seed);linear=kind=='prismatic'
        c=float(rng.uniform(*PRIORS[kind]['coulomb_n'if linear else'coulomb_nm']))
        b=float(rng.uniform(*PRIORS[kind]['viscous_n_s_m'if linear else'viscous_nm_s_rad']))
        case=dict(case=0,asset=asset,joint_type=kind,seed=seed,
            mass_scale=float(rng.uniform(*PRIORS['mass_scale'])),
            density_gradient=rng.uniform(*PRIORS['density_gradient'],3).tolist(),
            tau_c=c,b=b,spring=False,
            law=dict(family='constant',base=c,amplitude=0.,center=0.,width=1.))
        plant=root/(kind+'_HIDDEN_CONFIGURATIONS.json')
        plant.write_text(json.dumps([case],indent=2));plant.chmod(0o400)
        episode='v81_full_range_blind_'+kind+'_01'
        spec=dict(schema='agentic-interaction/v81',episode_id=episode,phase='engineering',
            scene=scene,physics_file=str(plant.resolve()),
            physics_sha256=hashlib.sha256(plant.read_bytes()).hexdigest(),identification_case=0,
            wall_budget_s=5400,sim_budget_s=2700,agent_wait_s=600,acquisition_budget=6,
            debug_budget_profile='full_range_integration_debug_v1')
        path=root/(kind+'_SPEC.json');path.write_text(json.dumps(spec,indent=2))
        public['episodes'].append(dict(episode_id=episode,joint_type=kind,asset=asset,
            scene=scene,physics_sha256=spec['physics_sha256'],
            spec_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (root/'PUBLIC_COMMITMENTS.json').write_text(json.dumps(public,indent=2))
    return public


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True,type=Path)
    a=p.parse_args();print(json.dumps(prepare(a.root),indent=2))
