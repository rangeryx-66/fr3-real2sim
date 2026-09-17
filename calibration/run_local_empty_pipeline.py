"""Diagnostic empty only: q_ref discrepancy is recorded, not rejected."""
import runpy
from pathlib import Path
import strict_pair_protocol
original=strict_pair_protocol.window_quality
def diagnostic(*args,**kwargs):
 r=original(*args,**kwargs);r['diagnostic_ignored_reasons']=[x for x in r['reasons'] if x=='Q_REF_MISMATCH'];r['reasons']=[x for x in r['reasons'] if x!='Q_REF_MISMATCH'];r['accepted']=not r['reasons'];return r
strict_pair_protocol.window_quality=diagnostic
runpy.run_path(str(Path(__file__).with_name('run_strict_pair_pipeline.py')),run_name='__main__')
