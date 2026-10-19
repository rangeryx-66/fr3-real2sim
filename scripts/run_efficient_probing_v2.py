"""One live Agent research episode with compact measurements, not an archived fixture."""
import argparse,hashlib,json,os,shutil,sys,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from agent_command_protocol_v12 import atomic_json
from interaction_runtime.serialization import result_packet

def main(runtime_factory=None,serve_fn=None,extra_sources=()):
    p=argparse.ArgumentParser();p.add_argument('--job',required=True,type=Path);p.add_argument('--check',action='store_true');a=p.parse_args();job=json.loads(a.job.read_text());out=Path(job['output']);out.mkdir(parents=True,exist_ok=True)
    sources=list((ROOT/'interaction_runtime').glob('*.py'))+[ROOT/'configs/interaction_runtime_v1.json',ROOT/'configs/efficient_probing_v2.json',ROOT/'scripts/update_probing_v2_model.py',Path(__file__)]
    sources += [ROOT/f for f in extra_sources]
    hashes={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
    for f in sources:
        target=out/'source'/f.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,target)
    atomic_json(out/'SOURCE_PROVENANCE.json',dict(reference='1c98f9b',research_reference=job.get('research_reference','1c98f9b'),runtime_factory=(runtime_factory.__name__ if runtime_factory else 'ProbingRuntime'),sources=hashes,physics_hz=240,primary_wrench='native fixed-interface wrist six-axis F/T',fresh_observation=True,saved_grasp_input=False))
    from interaction_runtime.probing_v2 import ProbingRuntime,serve
    from interaction_runtime.compact_v2 import compact_checkpoint_files,storage
    r=None;result=dict(status='INITIALIZING')
    try:
        r=(runtime_factory or ProbingRuntime)(job)
        if a.check:
            import numpy as np
            home=r.qtarget[r.arm].copy();moved=home.copy();moved[0]+=.03;moved[2]-=.02;r.g['phase']='SETTLE';r.hold(.25)
            r.move_path([moved],'PREGRASP');r.move_path([home],'SAFE_PARK_MOTION');r.hold(.25)
            result=dict(status='NATIVE_INTERFACE_CHECK_COMPLETE',object_contact=False,physics_steps=r.tick,FT_valid=all(row['wrist_ft']['valid'] for row in r.rows),max_TCP_mm_s=max(row['tcp_speed_m_s'] for row in r.rows)*1000)
        else:result=(serve_fn or serve)(r)
    except BaseException as e:result=dict(status='FAILED',error=repr(e),traceback=traceback.format_exc())
    finally:
        atomic_json(out/'EPISODE_RESULT.json',result_packet(result))
        if r is not None:
            # Isaac fast shutdown may exit Python: finalize structured data first.
            r.record.flush();r.record.queue.join()
            compact_checkpoint_files(out)
            r.close()
    return 0
if __name__=='__main__':sys.exit(main())
