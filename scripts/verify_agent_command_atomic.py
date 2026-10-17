"""Deterministic old empty-file race and concurrent atomic publisher replay; no Isaac."""
import json,tempfile,threading,time
from pathlib import Path
from publish_agent_command_atomic import publish


def verify():
    with tempfile.TemporaryDirectory() as name:
        root=Path(name);old=root/'old.json'
        # Equivalent to the observed SFTP O_CREAT/O_TRUNC interval. The final
        # payload can be valid even though a concurrent reader already failed.
        old.write_bytes(b'');old_error=None
        try:json.loads(old.read_text())
        except json.JSONDecodeError as error:old_error=str(error)
        old.write_text(json.dumps(dict(operation='observe')))
        errors=[]
        for i in range(250):
            final=root/('%03d.json'%i);payload=dict(operation='observe',sequence=i,trace='x'*100000)
            def read():
                deadline=time.monotonic()+5
                while not final.exists():
                    if time.monotonic()>deadline:errors.append('reader timeout');return
                    time.sleep(.0001)
                try:
                    if json.loads(final.read_text())!=payload:errors.append('payload mismatch')
                except Exception as error:errors.append(repr(error))
            reader=threading.Thread(target=read);reader.start();publish(root,i,payload);reader.join()
        assert old_error and not errors
        result=dict(physics_actors=0,old_empty_file_error=old_error,original_final_payload_valid=True,concurrent_publications=250,atomic_read_errors=errors,
            control_or_physics_changed=False,Isaac_runtime_validation=False)
        print(json.dumps(result,indent=2));return result
if __name__=='__main__':verify()
