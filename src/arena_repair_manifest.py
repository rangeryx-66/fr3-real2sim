"""Predeclare 32 held-out layouts, independent of diagnostic outcomes."""
import json
from arena_manifest import ROOT,NAMES,layout
old=json.loads((ROOT/'ARENA_COMPLEX_PROTOCOL.json').read_text())
fresh=[layout(name,2000+4*i+j) for i,name in enumerate(NAMES) for j in range(4)]
assert not set(e['seed'] for e in fresh)&set(e['seed'] for e in old['episodes'])
protocol={**old,'experiment':'Arena execution repair original 40 plus held-out 32',
          'formal_count':72,'development_seeds':[e['seed'] for e in old['episodes']],
          'heldout_seeds':[e['seed'] for e in fresh], 'episodes':old['episodes']+fresh}
path=ROOT/'ARENA_REPAIR_PROTOCOL.json'
if path.exists():assert json.loads(path.read_text())==protocol
else:path.write_text(json.dumps(protocol,indent=2))
