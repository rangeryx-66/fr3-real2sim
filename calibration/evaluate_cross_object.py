"""Reuse the frozen q-compensation implementation with an external case list."""
import os
from pathlib import Path
p=Path(__file__).with_name('compare_q_baselines.py');s=p.read_text();s=s.replace("O=R/'results/fr3_q_compensation'","O=Path(os.environ['CROSS_OUTPUT'])")
start=s.index(' cases={};base=');end=s.index(' summary={}',start)
s=s[:start]+''' cases={}
 for entry in json.load(open(O/'evaluation_cases.json')):
  cases[entry['name']]=(Path(entry['payload_dir']),[(x['pose_id'],Path(x['empty_path']),Path(x['payload_path']),None) for x in entry['pairs']])
'''+s[end:]
# Exploratory local-empty method is never a production selection in this test.
s=s.replace(",('empty_affine_exploratory',(np.array([x['tau_loaded'] for x in accepted])-pred).ravel())",'')
exec(compile(s,str(p),'exec'),{'__file__':str(p),'__name__':'__main__','os':os})
