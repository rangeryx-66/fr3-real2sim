"""Allow a separately sealed scene manifest; retain original Arena construction."""
import os
from pathlib import Path
_source=Path(__file__).with_name('arena_scene.py')
_text=_source.read_text()
_old="PROTOCOL=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())"
assert _text.count(_old)==1
_text=_text.replace(_old,"PROTOCOL=json.loads(Path(os.environ.get('FR3_ARENA_PROTOCOL',str(ROOT/'ARENA_COMPLEX_PROTOCOL.json'))).read_text())")
exec(compile(_text,str(_source),'exec'),globals())
