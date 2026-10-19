import sys,inspect,json,time,os
from pathlib import Path
ROOT=Path('/data1/home/rangeryx/fr3_real2sim_known_whole_20261007');sys.path.insert(0,str(ROOT/'scripts'))
import run_oracle_door_trials as owner
with (owner.out/'ADDITIONAL_RETEST_RESERVED.json').open('x') as f:json.dump({'asset_id':'45415','additional_retest_index':1,'only_additional_retest':True,'same_plan':True,'shared_fix':'retain closure position command during unilateral centering','wall':time.time(),'pid':os.getpid()},f,indent=2)
source=inspect.getsource(owner.execute).replace("'development_01'","'development_02'").replace("'run_oracle_door.py'","'run_oracle_door_closure_continuity.py'").replace("'trial':1","'trial':2")
namespace=dict(owner.__dict__);exec(compile(source,__file__+'::single_targeted_retest','exec'),namespace);namespace['execute']('45415')
(owner.out/'ADDITIONAL_RETEST_COMPLETE.json').write_text(json.dumps({'wall':time.time(),'no_more_physical_trials':True},indent=2))
