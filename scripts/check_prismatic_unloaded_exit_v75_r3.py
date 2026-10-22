"""Archived native-collision regression only; no Isaac actor or executable pose export."""
import argparse,json,time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np


def run(episode,job,output):
    from diagnose_probing_v3 import columns
    from check_prismatic_exit_v75 import run as original_check
    import interaction_runtime.prismatic_park_v75 as winding
    from interaction_runtime.prismatic_unloaded_park_v75_r3 import plan_unloaded_exit
    from native_g1_safe_park import plan_exit as original_exit
    p=Path(episode);began=time.perf_counter();a,_=columns(p)
    # The original executor only calls plan_exit AFTER jaw release, unloaded
    # pad checks and force_guard.settled(). A returned final plan proves this
    # recorded predicate passed; it is not synthesized from missing force data.
    trace=json.loads((p/'AGENT_ACTION_TRACE.json').read_text())
    terminal=trace[-1]['result'].get('safe_exit',{})
    if trace[-1]['command']['operation']!='finish' or not terminal.get('plan',{}).get('command_bridge',{}).get('valid'):
        raise ValueError('RECORDED_VERIFIED_UNLOADED_EXIT_CONTEXT_REQUIRED')
    if np.max(a['pad_load'][-1])>.05:raise ValueError('FINAL_PAD_LOAD_NOT_UNLOADED')
    checks={}
    def compare(g):
        g.update(filtered=lambda:np.asarray(a['pad_load'][-1]),force_guard=SimpleNamespace(settled=lambda:True))
        # Preserve legal-winding target extension during original + fallback.
        from interaction_runtime.prismatic_park_v75 import parking_targets
        import native_g1_safe_park as native
        original_targets=native.parking_targets
        with patch.object(native,'parking_targets',lambda state:parking_targets(state,original_targets)):
            result=plan_unloaded_exit(g,original_exit)
            scene=g['collision']
            with patch.object(scene,'check',lambda *args,**kw:(False,'DELIBERATE_BLOCKED_ROUTE_REGRESSION')):
                blocked=plan_unloaded_exit(g,original_exit)
        checks.update(original_predicates_only=True,blocked_route_rejected=not blocked['valid'],
            recorded_unloaded_provenance='original final release/settled predicate reached plan_exit',
            runtime_execution=False,hidden_parameters_read=False)
        return result
    with patch.object(winding,'plan_exit',compare):original_check(p,job,output)
    result=json.loads(Path(output).read_text())
    result.update(R3_diagnostic=checks,wall_s=time.perf_counter()-began)
    Path(output).write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode',type=Path);p.add_argument('--job',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=run(a.episode,a.job,a.output)
    print(json.dumps(dict(R3_valid=r.get('equivalent_winding',{}).get('valid'),checks=r['R3_diagnostic'])))
