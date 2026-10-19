"""One-shot lifecycle observer; requests shutdown only after verification ends."""
from pathlib import Path
import time,os,signal,json,sys
O=Path(sys.argv[1]);pid=int(sys.argv[2]);deadline=time.monotonic()+900
while time.monotonic()<deadline:
 if (O/'SHUTDOWN_FLUSHED.json').exists():break
 if (O/'READY_FOR_OWNER_STOP').exists():
  command=Path('/proc/%d/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode()
  if 'scripts/probe_nero_implicit.py --run 3' not in command:raise RuntimeError('not the owned NERO verification actor')
  (O/'OWNER_STOP_REQUEST.json').write_text(json.dumps({'pid':pid,'command':command,'signal':'SIGTERM','unix_time':time.time(),'reason':'all prescribed robot-only verification movements completed'},indent=2))
  os.kill(pid,signal.SIGTERM)
  break
 if not Path('/proc/%d'%pid).exists():break
 time.sleep(1.)
