"""Compact engineering results and immutable V7.2/revolute audit."""
import argparse,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from interaction_runtime.prismatic_v73 import audit_frozen
from agent_command_protocol_v12 import atomic_json

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text()) if path.exists() else {}

def audit(root):
    baseline_path=ROOT/'docs/experiments/prismatic_benchmark_v72_20261020/FINAL_PROVENANCE.json'
    if not baseline_path.exists():baseline_path=root/'BASELINE_V72_PROVENANCE.json'
    baseline=read(baseline_path)
    if 'runs' not in baseline:raise ValueError('Original V7.2 provenance manifest required; do not infer baseline hashes from current files')
    bad=[];checked=0
    for name,files in baseline['runs'].items():
        for f,expected in files.items():
            p=ROOT/'results/cross_object_v7_final_v72_20261021/episodes'/name/f;checked+=1
            if not p.exists() or digest(p)!=expected:bad.append(str(p))
    expected_models=read(root/'BASELINE_V72_MODEL_HASHES.json');model_bad=[];model_checked=0
    for name,h in expected_models.items():
        if not h:continue
        p=ROOT/'results/cross_object_v7_final_v72_20261021/episodes'/name/'model/estimate.json';model_checked+=1
        if not p.exists() or digest(p)!=h:model_bad.append(name)
    result=dict(frozen_revolute=audit_frozen(ROOT),V72=dict(original_manifest_sha256=digest(baseline_path),episodes=len(baseline['runs']),hashes_checked=checked,mismatch=bad,online_models_checked=model_checked,online_model_mismatch=model_bad),engineering_attempts=[])
    for p in sorted((root/'episodes').glob('*')):
        if not p.is_dir():continue
        end=read(p/'PROCESS_COMPLETION.json')
        if not end:continue # Never consume authored references during online decisions.
        trace=read(p/'AGENT_ACTION_TRACE.json');trace=trace if isinstance(trace,list) else []
        attempts=[]
        for e in trace:
            if e['command']['operation']!='attempt_grasp_and_probe':continue
            r=e['result'];attempts.append(dict(sequence=e['command']['sequence'],requested_preload_n=e['command'].get('native_g1_preload_request_n'),physical_acquisition_started=r.get('physical_grasp_started',False),acquisition=r.get('acquisition'),family=r.get('family'),error=r.get('error'),segments=r.get('segments',[]),safe_exit=r.get('safe_exit')))
        index=read(p/'telemetry/index.json');model=read(p/'model/estimate.json');job=read(p/'job.json');truth=read(p/'HIDDEN_AUTHORED_PHYSICS.json')
        sources=read(p/'SOURCE_PROVENANCE.json');files=[x for x in p.rglob('*') if x.is_file()]
        result['engineering_attempts'].append(dict(id=p.name,wall_s=end['wall_s'],actor_exited=end['actor_exited'],status=read(p/'EPISODE_RESULT.json').get('status'),attempts=attempts,observations=len(list((p/'rgbd').glob('generation*'))),physics_steps=index.get('physics_steps_seen'),numeric_records=index.get('records'),model=model.get('selected'),constant_model=model.get('constant_model'),FIT_bins=model.get('training_bins'),validation_bins=model.get('validation_bins'),source_sha256=sources.get('sources'),storage_bytes=sum(x.stat().st_size for x in files),authored_reference_post_episode=truth,private_evidence_index={str(x.relative_to(p)):digest(x) for x in [p/'telemetry/index.json',p/'EPISODE_RESULT.json',p/'SOURCE_PROVENANCE.json',p/'model/estimate.json'] if x.exists()}))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();atomic_json(a.root/'ENGINEERING_AUDIT.json',audit(a.root))
