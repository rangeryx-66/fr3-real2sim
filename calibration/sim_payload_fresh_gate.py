"""Isolated PayloadID start gate overlay; frozen grasp and runtime guards untouched."""
from pathlib import Path
p=Path(__file__).with_name('sim_real2sim.py');s=p.read_text()
needle="qguard.get('passed') and qguard.get('currently_clear') and min(f)>.1"
assert s.count(needle)==2
s=s.replace(needle,"__import__('payload_start_gate').evaluate_start(support)['passed'] and qguard.get('currently_clear') and min(f)>.1")
needle="'support':qguard,'forces_N':f.tolist()"
assert s.count(needle)==1
s=s.replace(needle,"'support':qguard,'payload_start_gate':__import__('payload_start_gate').evaluate_start(support),'forces_N':f.tolist()")
exec(compile(s,str(p),'exec'),{'__name__':'__main__','__file__':str(p)})
