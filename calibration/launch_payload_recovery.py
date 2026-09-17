from pathlib import Path
p=Path(__file__).with_name('launch_real2sim.py');s=p.read_text().replace("'calibration/run_real2sim_pipeline.py'","'calibration/run_payload_recovery_pipeline.py'");exec(compile(s,str(p),'exec'),{'__file__':str(p),'__name__':'__main__'})
