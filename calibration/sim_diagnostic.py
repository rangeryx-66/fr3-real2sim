from pathlib import Path
p=Path(__file__).with_name('sim_calibration.py')
s=p.read_text().replace('from hand_force_control import HandCalibration','from diagnostic_hand import DiagnosticHand as HandCalibration')
exec(compile(s,str(p),'exec'),dict(__name__='__main__',__file__=str(p)))
