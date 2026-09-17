import os,time,subprocess,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];base=ROOT/'results/hand_calibration';PY='/data1/home/rangeryx/isaaclab-arena/.venv/bin/python'
while not (base/'bowl_ALL_COMPLETE.json').exists():time.sleep(5)
env=os.environ.copy();env.update(FR3_ARENA_TARGET='soup',CALIBRATION_MU='0.',DIAGNOSTIC_DIR='zero_friction',CALIBRATION_PORT='18769',OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y',OPENBLAS_NUM_THREADS='1')
with (base/'zero_friction_sim.log').open('w') as log:
 sim=subprocess.Popen([PY,'-u','calibration/sim_diagnostic.py','--gpu','3','--port','18769'],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
 try:
  end=time.monotonic()+600
  while True:
   if sim.poll() is not None:raise RuntimeError('diagnostic simulator exited')
   try:urllib.request.urlopen('http://127.0.0.1:18769/calibration',timeout=2).read();break
   except Exception:
    if time.monotonic()>end:raise TimeoutError('diagnostic startup')
    time.sleep(2)
  os.environ['CALIBRATION_PORT']='18769'
  from run_trials import episode
  out=base/'zero_friction';out.mkdir(exist_ok=True)
  for rep in range(3):episode('soup',f'soup_force_F10_mu0.0_r{rep}',10,0.,'force',out)
  (out/'COMPLETE').write_text('3 measurement-only repeats')
 finally:
  sim.terminate()
  try:sim.wait(timeout=30)
  except subprocess.TimeoutExpired:sim.kill();sim.wait()
