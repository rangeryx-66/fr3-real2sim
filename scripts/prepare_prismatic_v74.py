"""Prepare original eight registered slots; never read or regenerate hidden values."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from prismatic_registration_v74 import verify,sha,CLASSIFICATION,ACTIVATION_SOURCES
from interaction_runtime.prismatic_v73 import audit_frozen
from agent_command_protocol_v12 import atomic_json

def prepare(registration,private,engineering,output,gpu,visual_gpu):
    m=verify(ROOT,registration,private);frozen=audit_frozen(ROOT);output=Path(output)
    if output.exists():raise ValueError('Do not overwrite an activated registration')
    output.mkdir(parents=True);(output/'jobs').mkdir()
    activation=dict(schema='v74-authorized-activation',execution_authorized=True,
        authorization='User V7.4 task; no engineering scope relabelling',
        registration_sha256=sha(registration),hidden_configuration_sha256=m['hidden_configuration_sha256'],
        frozen_registered_source_count=len(m['source_sha256']),frozen_revolute=frozen,
        episode_order=[e['episode_id'] for e in m['episodes']],
        source_sha256={p:sha(ROOT/p) for p in ACTIVATION_SOURCES},
        controller_or_physics_changed=False,lighting='unchanged frozen explicit sensor_lighting_v1',
        registered_constructor='V7.3 wiring parity; original engineering guard remains active',
        online_blinding='private values never copied to job or Agent packets; reveal only after all eight online seals')
    atomic_json(output/'ACTIVATION.json',activation)
    for e in m['episodes']:
        j=json.loads((Path(engineering)/'jobs'/(e['asset_id']+'_fresh_00.json')).read_text())
        j.pop('engineering_revision',None)
        r=m['resources'];j.update(experiment='V7_BENCHMARK',run_classification=CLASSIFICATION,
            registered_episode_id=e['episode_id'],benchmark_registration=str(Path(registration).resolve()),
            registered_activation=str((output/'ACTIVATION.json').resolve()),
            research_reference='54e35f2 / registered V7.4',identification_case=e['configuration_index'],
            private_physics_configuration=str(Path(private).resolve()),output=str((output/'episodes'/e['episode_id']).resolve()),
            gpu=gpu,visual_gpu=visual_gpu,native_g1_preload_request_n=1.,
            episode_wall_budget_s=r['wall_s'],episode_sim_budget_s=r['simulation_s'],
            safe_exit_wall_reserve_s=r['exit_reserve_wall_s'],safe_exit_sim_reserve_s=r['exit_reserve_sim_s'],
            agent_wait_timeout_s=r['unloaded_Agent_deadline_s'],storage_target_bytes=r['evidence_target_bytes'],
            runtime_acquisition_budget=m['recovery']['maximum_acquisitions'],maximum_agent_operations=m['recovery']['maximum_operations'])
        atomic_json(output/'jobs'/(e['episode_id']+'.json'),j)
    print(json.dumps(dict(registered_jobs=8,hidden_values_read=False,launched=False,commitment=m['hidden_configuration_sha256'])))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--registration',type=Path,required=True);p.add_argument('--private',type=Path,required=True);p.add_argument('--engineering',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--gpu',type=int,required=True);p.add_argument('--visual-gpu',type=int,required=True);a=p.parse_args();prepare(a.registration,a.private,a.engineering,a.output,a.gpu,a.visual_gpu)
