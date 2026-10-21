"""Read-only commitment verification and explicit V7.4 job activation."""
import hashlib,json
from pathlib import Path

CLASSIFICATION='REGISTERED_BLIND_PRISMATIC_V74'
ACTIVATION_SOURCES=('interaction_runtime/registered_prismatic_v74.py',
    'scripts/prismatic_registration_v74.py','scripts/prepare_prismatic_v74.py',
    'scripts/run_registered_prismatic_v74.py','scripts/launch_registered_prismatic_v74.py')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())

def verify(root,registration,private):
    root=Path(root);m=read(registration)
    if m['schema']!='prismatic-v74-preregistration':raise ValueError('Wrong registration')
    bad=[p for p,h in m['source_sha256'].items() if not (root/p).exists() or sha(root/p)!=h]
    if bad:raise ValueError('Frozen source mismatch:'+str(bad))
    if sha(private)!=m['hidden_configuration_sha256']:raise ValueError('Hidden commitment mismatch')
    ids=[e['episode_id'] for e in m['episodes']]
    if len(ids)!=8 or len(set(ids))!=8 or any(e['configuration_index']!=i for i,e in enumerate(m['episodes'])):
        raise ValueError('Original eight-slot order required')
    return m

def verify_job(root,job):
    if job.get('experiment')!='V7_BENCHMARK' or job.get('run_classification')!=CLASSIFICATION:
        raise ValueError('Explicit registered V7.4 job required; never relabel engineering')
    if any(job.get(k) for k in ('preload_fixture','prismatic_integrated_ik_feedback','prismatic_contact_retention','engineering_microtracking','engineering_pregrasp_reference')):
        raise ValueError('Unvalidated prototype or cached fixture forbidden')
    m=verify(root,job['benchmark_registration'],job['private_physics_configuration'])
    rows=[e for e in m['episodes'] if e['episode_id']==job.get('registered_episode_id')]
    if len(rows)!=1:raise ValueError('Unknown registered slot')
    e=rows[0]
    if (job.get('identification_case')!=e['configuration_index'] or job.get('asset_id')!=e['asset_id'] or
        job['articulation_v7']['joint_type']!='prismatic' or job.get('native_g1_preload_request_n')!=1.):
        raise ValueError('Registered identity/configuration/initial preload mismatch')
    activation=read(job['registered_activation'])
    if not activation.get('execution_authorized') or activation['hidden_configuration_sha256']!=m['hidden_configuration_sha256']:
        raise ValueError('Explicit activation missing')
    if any(sha(Path(root)/p)!=h for p,h in activation['source_sha256'].items()):raise ValueError('Activation source mismatch')
    if sha(job['benchmark_registration'])!=activation['registration_sha256']:raise ValueError('Registration changed after activation')
    return m,e
