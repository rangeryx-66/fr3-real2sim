"""Explicit prismatic V7.5 session; protocol/safety control flow frozen to V7.3.

Only the operations factory differs. No dynamic function exec or dependency
rebinding. Tests compare the session AST against its preserved original.
"""
import json,time
from agent_command_protocol_v12 import atomic_json,receive,complete
from .serialization import result_packet
from .policy import SafetyStop
from .segments import ExecutionBudgetExhausted,validate_action
from .prismatic_feedback_v75 import PrismaticEvidenceOperationsV75


def serve_v75(r):
    ops=PrismaticEvidenceOperationsV75(r);g=r.g;commands=r.output/'agent_commands';responses=r.output/'agent_responses';commands.mkdir(exist_ok=True);responses.mkdir(exist_ok=True);trace=[];last=None
    try:
        from .registered_observation_v71 import initialize
        initialize(r,ops)
        for seq in range(int(r.job.get('maximum_agent_operations',40))):
            if g['recovery_state']!='UNLOADED_OBSERVATION' or r.supervisor.latched is not None:
                return dict(status='HARD_STOP_OR_NO_SAFE_DECISION_BOUNDARY',trace=trace,stop=r.supervisor.latched)
            resources=r.episode_budget.remaining(r.tick*r.dt);wait=min(float(r.job['agent_wait_timeout_s']),resources['wall_s'])
            if wait<=0:return dict(status='BUDGET_EXHAUSTED',safe_exit=ops.release_park(),trace=trace)
            atomic_json(r.output/'DECISION_PACKET.json',ops.decision_packet(seq,last));atomic_json(r.output/'WAITING_FOR_AGENT.json',dict(sequence=seq,unloaded=True,timeout_s=wait,decision_packet=str(r.output/'DECISION_PACKET.json')))
            began=time.perf_counter();path=commands/('%03d.json'%seq)
            while not path.exists():
                if r.stopped or (r.output/'STOP_AND_EXPORT').exists() or time.perf_counter()-began>=wait:
                    return dict(status='ORCHESTRATION_TIMEOUT' if not r.stopped else 'OWNER_STOP',safe_exit=ops.release_park(),trace=trace)
                time.sleep(.05)
            command,ack=receive(path,seq,responses)
            if command is None:trace.append(ack);continue
            op=command['operation'];started=time.perf_counter()
            try:
                for a in command.get('segments',[]):validate_action(a);ops.articulation().validate_target(a)
                if op=='observe':result=ops.observe()
                elif op=='inspect':result=ops.decision_packet(seq)
                elif op=='evaluate_view':result=ops.evaluate_view(command['target']);ops.cache[seq]=result
                elif op=='move_and_observe':result=ops.move_and_observe(ops.cache.get(command['evaluated_command'],{}))
                elif op=='build_geometry':result=ops.request_plan(command)
                elif op=='validate_execution':result=ops.validate_execution()
                elif op=='attempt_grasp_and_probe':result=ops.attempt(command)
                elif op=='inspect_contact_result':result=ops.feedback()
                elif op in ('reobserve_and_replan','fresh_mode_observation'):result=ops.reobserve()
                elif op=='safe_release_retreat':result=ops.release_park()
                elif op=='finish':result=dict(safe_exit=ops.release_park())
                else:result=dict(executed=False,recoverable=True,reason='MODE_NOT_IMPLEMENTED_FOR_HORIZONTAL_DRAWER')
            except SafetyStop as e:
                result=dict(status='SAFETY_STOP',reason=e.reason,evidence=e.evidence)
            except ExecutionBudgetExhausted as e:
                result=dict(status='BUDGET_EXHAUSTED',reason=str(e),safe_exit=ops.release_park())
            except Exception as e:
                result=dict(operation_stopped=True,reason=repr(e),recoverable=g['recovery_state']=='UNLOADED_OBSERVATION' and r.supervisor.latched is None)
                if not result['recoverable']:result['safe_exit']=dict(safe=False,reason='NO_VERIFIED_UNLOADED_BOUNDARY')
            entry=dict(command=command,result=result,wall_s=time.perf_counter()-started,unloaded_wait_s=started-began);complete(responses,seq,result_packet(entry));trace.append(entry);last=result
            atomic_json(r.output/'AGENT_ACTION_TRACE.json',result_packet(trace))
            from .compact_v2 import prune_optional
            atomic_json(r.output/'STORAGE_SUMMARY.json',prune_optional(r.output))
            if op=='finish':return dict(status='SESSION_COMPLETE',trace=trace,safe_exit=result.get('safe_exit'))
            if r.supervisor.latched is not None or result.get('status')=='BUDGET_EXHAUSTED':return dict(status=result.get('status','SAFETY_STOP'),trace=trace)
        return dict(status='BOUNDED_OPERATION_BUDGET',safe_exit=ops.release_park(),trace=trace)
    finally:
        ops.close_workers()
        # The inherited compact finalizer keeps the raw RGB/depth/selected mask
        # once. Retain new semantic/hypothesis provenance before it removes
        # reconstructible mask renderings and intermediate files.
        audit=[]
        for folder in sorted((r.output/'rgbd').glob('*')):
            row=dict(checkpoint=folder.name)
            for name in ['SEMANTIC_RECOVERY_V73.json','depth_handle_proposals.json','AGENT_HANDLE_HYPOTHESIS_APPROVAL.json']:
                if (folder/name).exists():row[name]=json.loads((folder/name).read_text())
            if len(row)>1:audit.append(row)
        atomic_json(r.output/'PERCEPTION_PROVENANCE_V73.json',result_packet(audit))
