from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'active_gpu': 1, 'physics_gpu': 1, 'multi_gpu': False})
from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path
print('ASSETS_ROOT', get_assets_root_path(skip_check=True), flush=True)
w=World(stage_units_in_meters=1.0)
w.scene.add_default_ground_plane()
w.reset()
for i in range(30): w.step(render=True)
print('SIM_SMOKE_OK',flush=True)
app.close()
