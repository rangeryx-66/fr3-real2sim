"""Parent-independent elapsed process lifetime/cleanup record for our registered short job."""
import json,os,sys,time
from pathlib import Path
folder=Path(sys.argv[1]);proc=json.loads((folder/'PROCESS.json').read_text());pid=proc['pid'];path=Path('/proc')/str(pid)
args=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode()
if 'scripts/run_fast_interaction_v1.py' not in args:raise RuntimeError('Not the registered short actor')
stat=(path/'stat').read_text().split();start=float(stat[21])/os.sysconf('SC_CLK_TCK');began=time.monotonic()
while path.exists() and time.monotonic()-began<600:
 try:
  if (path/'stat').read_text().split()[2]=='Z':break
 except OSError:break
 time.sleep(.2)
(folder/'PROCESS_COMPLETION.json').write_text(json.dumps(dict(pid=pid,owned_command=args,actor_exited=not path.exists(),wall_including_initialization_and_cleanup_s=time.monotonic()-start),indent=2))
