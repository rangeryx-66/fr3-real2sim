"""Bounded concurrent transport/packet tests, zero Isaac imports or actors."""
import concurrent.futures
import inspect
import json
from pathlib import Path
import tempfile
import threading
import time
from agent_command_protocol_v12 import publish, receive, complete, wire_owner, ProtocolError
from identification_packet_v12 import process, publish_raw, atomic_json, verified_marker


def verify():
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as name:
        directory = Path(name); commands = directory / 'commands'; responses = directory / 'responses'
        commands.mkdir(); responses.mkdir()
        consumed = []
        def consumer():
            for sequence in range(200):
                path = commands / ('%03d.json' % sequence)
                deadline = time.monotonic() + 10
                while not path.exists():
                    assert time.monotonic() < deadline
                    time.sleep(.0001)
                value, ack = receive(path, sequence, responses)
                assert value['sequence'] == sequence and ack['status'] == 'ACCEPTED'
                complete(responses, sequence, dict(result=dict(ok=True)))
                consumed.append(sequence)
        with concurrent.futures.ThreadPoolExecutor(max_workers=9) as pool:
            pending = pool.submit(consumer)
            tasks = [pool.submit(publish, commands, i, dict(operation='observe', inspected='x' * 10000)) for i in range(200)]
            for task in tasks: task.result()
            pending.result()
        assert len(consumed) == 200
        # Duplicate concurrent producers never replace the winning command.
        def duplicate(i):
            try: publish(commands, 200, dict(operation='finish', reason=str(i))); return True
            except FileExistsError: return False
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            wins = list(pool.map(duplicate, range(8)))
        assert sum(wins) == 1
        bad = [b'', b'{"operation":', b'[]', b'{"protocol_version":1,"sequence":204,"operation":"observe","operation":"finish"}',
            b'{"protocol_version":1,"sequence":205,"operation":"evaluate_view","target":{"eye_world_m":[NaN,0,1]}}',
            b'{"protocol_version":1,"sequence":999,"operation":"observe"}',
            b'{"protocol_version":1,"sequence":207,"operation":"move_and_observe"}',
            b'{"protocol_version":1,"sequence":208,"operation":"plan_panel_push","rectangle":[0,0,1,1]}']
        for sequence, payload in enumerate(bad, 201):
            path = commands / ('%03d.json' % sequence); path.write_bytes(payload)
            value, ack = receive(path, sequence, responses)
            assert value is None and ack['recoverable'] and not ack['robot_action_executed']
        publish(commands, 209, dict(operation='finish', reason='scene remains usable after invalid messages'))
        value, ack = receive(commands / '209.json', 209, responses)
        assert value['operation'] == 'finish'

        # Run the actual transformed owner against harmless dependency stubs.
        text = (root / 'scripts/native_g1_contact_recovery.py').read_text()
        start = text.index('def run(g):'); source = wire_owner(text[start:])
        compile(source, 'verified_actual_owner_transport', 'exec')
        class Passive:
            def install(self, g): pass
        class Observation:
            def write(self, path, data): atomic_json(path, data)
        class App:
            def is_running(self): return True
            def update(self): pass
        def write(g, name, value): atomic_json(g['a'].output / name, value)
        import os, traceback
        namespace = dict(install_camera=lambda g: None, passive=Passive(), initial_observation=lambda g: {},
            write=write, observation=Observation(), os=os, time=time, Path=Path, traceback=traceback)
        exec(source, namespace)
        owner = directory / 'owner'; inbox = owner / 'agent_commands'; inbox.mkdir(parents=True)
        (inbox / '000.json').write_text('{')
        publish(inbox, 1, dict(operation='finish'))
        from types import SimpleNamespace
        g = dict(a=SimpleNamespace(output=owner), job={}, step=lambda: None, app=App(),
            recovery_state='UNLOADED_OBSERVATION', tick=0, dt=1/240, _profile_entry=time.perf_counter())
        result = namespace['run'](g)
        assert result['physical_grasp_attempts'] == 0
        assert json.loads((owner/'agent_responses/000.json').read_text())['status'] == 'PROTOCOL_ERROR'
        assert json.loads((owner/'agent_responses/001.json').read_text())['status'] == 'COMPLETED'

        # Consumers start before their producers and before preceding estimates.
        packet0 = directory / 'packet0'; packet1 = directory / 'packet1'
        packet0.mkdir(); packet1.mkdir(); order = []
        def worker(folder):
            if folder == packet0: time.sleep(.12)
            if folder == packet1: assert verified_marker(packet0, 'MODEL_READY.json') is not None
            order.append(folder.name)
            (folder / 'TORQUE_COMPONENTS.json').write_text(json.dumps(dict(status='no informative data')))
        def producer(folder, previous, delay):
            time.sleep(delay)
            (folder / 'measurements.pkl').write_bytes(b'partial')
            time.sleep(.03)
            (folder / 'measurements.pkl').write_bytes(b'complete preserved raw packet')
            atomic_json(folder / 'INPUT.json', dict(previous_packets=previous))
            publish_raw(folder)
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            futures = [pool.submit(process, packet1, worker, 5), pool.submit(process, packet0, worker, 5),
                pool.submit(producer, packet1, [str(packet0)], 0), pool.submit(producer, packet0, [], .05)]
            for task in futures: task.result()
        assert order == ['packet0', 'packet1']
        assert verified_marker(packet1, 'MODEL_READY.json')['dependencies'][0]['status'] == 'NO_LOCAL_DATA'
        # Missing producer is reported, never silently fit without its prefix.
        packet2 = directory / 'packet2'; packet2.mkdir()
        (packet2/'measurements.pkl').write_bytes(b'raw')
        atomic_json(packet2/'INPUT.json', dict(previous_packets=[str(directory/'missing')]))
        publish_raw(packet2)
        try: process(packet2, worker, .01)
        except TimeoutError: pass
        else: raise AssertionError('Missing packet was silently skipped')
        assert not (packet2/'MODEL_READY.json').exists() and (packet2/'MODEL_ERROR.json').exists()
        return dict(physical_actors=0, concurrent_commands=200, malformed_recoverable=len(bad),
            same_sequence_winners=sum(wins), actual_owner_bad_then_finish='PASS',
            packet_producer_consumer_order=order, missing_prefix='EXPLICIT_DIAGNOSTIC_NO_PARTIAL_FIT',
            physical_safety_or_control_changed=False)


if __name__ == '__main__': print(json.dumps(verify(), indent=2))
