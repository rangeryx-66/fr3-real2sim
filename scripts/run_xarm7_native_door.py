"""One registered closed-start native-hand trial, frozen common arm skill."""
from pathlib import Path
import sys,json,hashlib
from xarm7_native_execution import executable
spec=json.loads(Path(sys.argv[sys.argv.index('--job')+1]).read_text())
text=executable(spec);compile(text,__file__+'::shared_native_interface','exec')
output=Path(spec['output']);output.mkdir(parents=True,exist_ok=True)
(output/'ACTUAL_SHARED_EXECUTABLE.py').write_text(text)
(output/'EXECUTABLE_HASH.json').write_text(json.dumps({'entry_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'generated_sha256':hashlib.sha256(text.encode()).hexdigest()}))
exec(compile(text,__file__+'::shared_native_interface','exec'),globals())
