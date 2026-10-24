"""Immutable segment packets and atomic model publications, no changed math."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from agent_command_protocol_v12 import atomic_json


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def snapshot(root,destination,completed,episode_id):
    """Index immutable raw chunks by relative reference, never copy arrays."""
    root=Path(root);destination=Path(destination);destination.mkdir(parents=True,exist_ok=False)
    source=json.loads((root/'telemetry/index.json').read_text());data=deepcopy(source)
    ids={s['id'] for s in completed}
    # Never include a subsequently started or completed VALIDATION in a FIT
    # packet. Its own later evaluation packet remains a separate publication.
    data['segments']=deepcopy(completed)
    end=max((s['last_step'] for s in completed),default=-1)
    chunks=[c for c in source['chunks'] if c['first_step']<=end]
    data['chunks']=[]
    for chunk in chunks:
        path=root/'telemetry'/chunk['file']
        if sha(path)!=chunk['sha256']:raise ValueError('IMMUTABLE_NUMERIC_CHUNK_HASH_MISMATCH')
        c=dict(chunk,file=os.path.relpath(path,destination/'telemetry'))
        data['chunks'].append(c)
    atomic_json(destination/'telemetry/index.json',data)
    settings=(root/'identification_input.json').read_bytes()
    (destination/'identification_input.json').write_bytes(settings)
    atomic_json(destination/'job.json',dict(episode_id=episode_id))
    manifest=dict(schema='v81-immutable-numeric-packet',episode_id=episode_id,
        completed_segment_ids=sorted(ids),index_sha256=sha(destination/'telemetry/index.json'),
        settings_sha256=sha(destination/'identification_input.json'),
        raw_array_copies=0,raw_references='same-episode immutable chunks',
        hidden_dynamics_available=False,packet_complete=True)
    atomic_json(destination/'PACKET.json',manifest)
    return manifest


def submit(ops,ready):
    if not ops.r.job.get('v81_integrated_repair'):raise ValueError('EXPLICIT_EXTENSION_REQUIRED')
    completed=deepcopy([s for s in ops.r.record.segments if s.get('completed')])
    if not completed:return
    identity=tuple((s['id'],s['last_step']) for s in completed)
    if identity==getattr(ops,'_v81_last_packet_identity',None):return
    ops._v81_last_packet_identity=identity
    number=len(ops.updates)+1;packet=ops.r.output/'numeric_packets'/('%03d'%number)
    latest=max((s['id'] for s in completed if s['role'].startswith('FIT')),default=0)
    state=dict(state='QUEUED',packet=str(packet),latest_FIT_id=latest,
        completed_ids=[s['id'] for s in completed],created_unix_s=time.time())
    atomic_json(ops.r.output/'model/WORKER_LIFECYCLE.json',state)
    def run():
        try:
            if not ready.wait(30):raise TimeoutError('NUMERIC_PRODUCER_BARRIER_TIMEOUT')
            manifest=snapshot(ops.r.output,packet,completed,ops.r.job['episode_id'])
            atomic_json(packet/'WORKER_STATE.json',dict(state,state='RUNNING',manifest=manifest))
            with (packet/'worker.log').open('w') as log:
                proc=subprocess.run([sys.executable,str(ops.r.root/'agentic_interaction_v81/update_model.py'),
                    str(packet),'--output',str(packet/'model')]+
                    (['--allow-interrupted-fit'] if ops.r.job.get('allow_interrupted_fit') else []),
                    stdout=log,stderr=subprocess.STDOUT,timeout=60)
            if proc.returncode:raise RuntimeError('PRODUCTION_WORKER_EXIT_'+str(proc.returncode))
            folder=packet/'model';readiness=json.loads((folder/'FIT_READINESS_V81.json').read_text())
            if readiness['model_sha256']!=sha(folder/'estimate.json'):
                raise ValueError('MODEL_READINESS_COMMIT_MISMATCH')
            publication=dict(state='READY' if readiness['qualified_bidirectional'] else 'DATA_INSUFFICIENT',
                packet=str(packet),model=str(folder/'estimate.json'),
                readiness=str(folder/'FIT_READINESS_V81.json'),model_sha256=readiness['model_sha256'],
                latest_FIT_id=readiness['latest_completed_FIT_id'],published_unix_s=time.time())
            # Each packet is written once. Only the pointer/public UI copies
            # change, monotonically; no reader seals a mismatched pair.
            old=ops.r.output/'model/MODEL_PUBLICATION.json'
            previous=json.loads(old.read_text()) if old.exists() else {}
            if previous.get('latest_FIT_id',-1)<=publication['latest_FIT_id']:
                for name in ('estimate.json','FIT_READINESS_V81.json','feedback.png'):
                    p=folder/name
                    if p.exists():
                        target=ops.r.output/'model'/name;temp=target.with_suffix(target.suffix+'.pending')
                        temp.write_bytes(p.read_bytes());temp.replace(target)
                atomic_json(old,publication)
                atomic_json(ops.r.output/'model/WORKER_LIFECYCLE.json',publication)
            atomic_json(packet/'WORKER_STATE.json',publication)
            return publication
        except Exception as error:
            failed=dict(state,state='WORKER_FAILED',reason=repr(error))
            atomic_json(ops.r.output/'model/WORKER_LIFECYCLE.json',failed)
            if packet.exists():atomic_json(packet/'WORKER_STATE.json',failed)
            return failed
    ops.updates.append(ops.worker.submit(run))
