"""One shared execution entry point, one normal closed-start attempt per case."""
from pathlib import Path
import sys,json,hashlib,os,shutil
# Post-batch I/O-only repair. Never changes control or creates a pass gate.
_bundle_ffmpeg=Path("/data1/home/rangeryx/tools/ffmpeg/ffmpeg-7.0.2-amd64-static")
if shutil.which("ffmpeg") is None and (_bundle_ffmpeg/"ffmpeg").is_file():
    os.environ["PATH"]=str(_bundle_ffmpeg)+os.pathsep+os.environ.get("PATH","")
from xarm7_door_adapter import executable
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
source=executable(spec);out=Path(spec['output']);out.mkdir(parents=True,exist_ok=True)
(out/'ACTUAL_SHARED_EXECUTABLE.py').write_text(source)
files=[Path(__file__),Path(__file__).with_name('xarm7_runtime.py'),Path(__file__).with_name('xarm7_door_adapter.py'),Path(spec['frozen_execution_runner']),Path(spec['baseline_runner']),Path(spec['pink_runner']),Path(spec['active_closure_source']),Path(spec['shared_policy'])]
(out/'XARM_EXECUTION_PROVENANCE.json').write_text(json.dumps({'classification':'XARM7 / ORACLE_HANDLE / KNOWN_MODEL / APPROXIMATE_ACTUATOR_SIMULATION','sources':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},'composed_executable_sha256':hashlib.sha256(source.encode()).hexdigest(),'virtual_mount':True,'intentional_release_regrasp':False},indent=2))
compile(source,str(out/'ACTUAL_SHARED_EXECUTABLE.py'),'exec')
exec(compile(source,str(out/'ACTUAL_SHARED_EXECUTABLE.py'),'exec'),globals())
