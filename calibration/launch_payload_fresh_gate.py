from pathlib import Path
p=Path(__file__).with_name('launch_real2sim.py');s=p.read_text()
for old,new in [('calibration/run_real2sim_pipeline.py','calibration/run_payload_recovery_pipeline.py'),('calibration/sim_real2sim.py','calibration/sim_payload_fresh_gate.py')]:
 assert s.count(old)==1
 s=s.replace(old,new)
exec(compile(s,str(p),'exec'),{'__name__':'__main__','__file__':str(p)})
