"""Scene manifest selection only; physics/controller code remains unchanged."""
from pathlib import Path
_source=Path(__file__).with_name('sim_arena.py')
_text=_source.read_text()
_old='import arena_scene\\nbox=arena_scene.table'
assert _text.count(_old)==1
_text=_text.replace(_old,'import arena_scene_repair as arena_scene\\nbox=arena_scene.table')
_text=_text.replace("f'{arena_scene.TARGET}_scene.usdc'", "f'{arena_scene.TARGET}_repair_scene.usdc'")
exec(compile(_text,str(_source),'exec'),globals())
