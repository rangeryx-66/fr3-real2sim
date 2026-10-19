"""Development label only; frozen fixed-base contact controller remains literal."""
from pathlib import Path
shared=Path(__file__).with_name('run_benchmark_a30.py')
# The nested runner reads the original frozen controller and acquired relative
# transform. Nothing in sensing, command generation or physical guards changes.
source=shared.read_text().replace("code=shared.read_text()", "code=shared.read_text().replace(\"'classification':'KNOWN_MODEL_DIAGNOSTIC'\",\"'classification':'ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT'\").replace('BENCHMARK_A_30_DEG_SUCCESS','ORACLE_HANDLE_30_DEG_SUCCESS')")
exec(compile(source,str(shared)+'::oracle_handle_development_label','exec'),globals())
