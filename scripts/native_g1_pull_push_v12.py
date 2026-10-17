"""V1.2 communication/packet integration. Frozen physical operations reused.

Only the original planned post-push second acquisition is available; this is
not an extra recovery budget. No contact, motion or perception constants here.
"""
import inspect
import subprocess
import sys
import time
from pathlib import Path
from unittest.mock import patch
import native_g1_hybrid_contact as hybrid
import native_g1_large_range_contact as continuous
from agent_command_protocol_v12 import wire_owner
from identification_packet_v12 import publish_raw, verified_marker

_save = hybrid.save_segment
_handle = hybrid.handle_attempt
_run = hybrid.run


def save_segment(g, first, label, result):
    _save(g, first, label, result)
    folder = Path(g['hybrid_segments'][-1]['folder'])
    publish_raw(folder)
    with (folder / 'torque_worker.log').open('w') as log:
        worker = subprocess.Popen([sys.executable, str(g['ROOT'] / 'scripts/identification_packet_v12.py'), str(folder)],
            stdout=log, stderr=subprocess.STDOUT)
    g['torque_workers'].append(worker)


def pull(g):
    # Frozen segment stepping/handover; only passive save/worker ownership
    # changes. Launch one worker per packet, including panel-push packets.
    records = []
    for action in g['torque_preapproved_actions']:
        first = len(g['rows']); index = len(g['hybrid_segments'])
        result = dict(ok=False, status='interrupted before segment completion', action=action)
        try:
            result = continuous.segment(g, action, index); records.append(result)
        finally:
            save_segment(g, first, action['role'], result)
        if not result['ok']: return False, result['status'], records
    return True, 'PREAPPROVED_BIDIRECTIONAL_BLOCK_COMPLETE', records


def handle_attempt(g, command):
    actions = command.get('segments')
    if not actions or len(actions) > 4: return dict(executed=False, reason='explicit finite segment list required')
    if any(not (0 < float(a['target_deg']) <= 30 and a['role'].startswith(('FIT', 'VALIDATION'))) for a in actions):
        return dict(executed=False, reason='outside registered finite action envelope')
    g['torque_preapproved_actions'] = actions
    return _handle(g, command)


def update_models_unloaded(g):
    if g['recovery_state'] != 'UNLOADED_OBSERVATION': return dict(updated=False, reason='not verified unloaded')
    began = time.monotonic()
    while any(w.poll() is None for w in g['torque_workers']) and time.monotonic() - began < 90:
        g['app'].update(); time.sleep(.1)
    packets = []
    for record in g['hybrid_segments']:
        folder = Path(record['folder'])
        try:
            status = verified_marker(folder, 'MODEL_READY.json')
            packets.append(dict(folder=str(folder), model=status,
                diagnostic='processing pending or failed' if status is None else None))
        except Exception as error:
            packets.append(dict(folder=str(folder), diagnostic=repr(error)))
    return dict(packets=packets, waited_only_unloaded=True, wall_s=time.monotonic() - began)


def generated_owner_source():
    source = inspect.getsource(_run)
    anchor = 'source=inspect.getsource(recovery.run)'
    if source.count(anchor) != 1: raise ValueError('Pinned explicit owner seam changed')
    source = source.replace(anchor, 'source=wire_owner(inspect.getsource(recovery.run))')
    return source


def run(g):
    policy = g['a'].output / 'CONTINUOUS_CONTACT_POLICY.py'; policy.write_text(continuous.contact_policy_source())
    g['spec'] = dict(g['spec'], shared_policy=str(policy)); g['torque_workers'] = []
    source = generated_owner_source()
    (g['a'].output / 'ACTUAL_V12_INTERFACE.py').write_text(source)
    namespace = dict(hybrid.__dict__, wire_owner=wire_owner)
    exec(compile(source, __file__ + '::explicit_transport_seam', 'exec'), namespace)
    with patch.object(hybrid, 'handle_attempt', handle_attempt), patch.object(hybrid, 'pull', pull), \
         patch.object(hybrid, 'push_step', continuous.push_step), patch.object(hybrid, 'save_segment', save_segment), \
         patch.object(hybrid, 'update_models_unloaded', update_models_unloaded):
        # Module globals were copied before patching; wire these operations
        # explicitly instead of depending on an inherited callback dictionary.
        namespace.update(handle_attempt=handle_attempt, pull=pull, push_step=continuous.push_step,
            save_segment=save_segment, update_models_unloaded=update_models_unloaded)
        try:
            return namespace['run'](g)
        finally:
            # Owner closes only its passive workers; raw packets survive errors.
            for worker in g['torque_workers']:
                if worker.poll() is None:
                    worker.terminate()
                    try: worker.wait(timeout=10)
                    except subprocess.TimeoutExpired: worker.kill(); worker.wait()
