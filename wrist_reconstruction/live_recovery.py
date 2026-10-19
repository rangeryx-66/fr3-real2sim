"""Keep the existing Isaac runtime parked while checked recovery data are supplied."""
import json,time,traceback
import numpy as np
from pathlib import Path
from wrist_reconstruction.constraint_policy import State,TaskTermination


def validate_clearance(r,recovery,request):
    opening=float(request['opening_m'])
    actual=float(r.finger_q()[0]-r.finger_q()[1])
    if not actual<=opening<=min(.1,actual+.001*r.capture.config['retreat']['maximum_release_increments']):
        raise RuntimeError('RECOVERY_REQUEST_OPENING_OUTSIDE_EXISTING_BOUNDS')
    fingers=np.array([opening/2,-opening/2]);path=request.get('arm_path',[r.arm_q().tolist()])
    Q=np.asarray(path,float)
    if Q.ndim!=2 or Q.shape[1]!=6 or not np.all(np.isfinite(Q)) or np.max(abs(Q[0]-r.arm_q()))>.001:
        raise RuntimeError('RECOVERY_REQUEST_ARM_START_MISMATCH')
    scene=recovery.mobile.scene(r.base)
    for a,b in zip(Q,np.vstack([Q[1:],Q[-1:]])):
        for q in np.linspace(a,b,max(2,int(np.max(abs(b-a))/.025)+2)):
            ok,why,_=recovery.mobile.check(scene,q,r.base,fingers)
            if not ok:raise RuntimeError('RECOVERY_REQUEST_ARM_'+why)
    plan={'opening':opening,'escape':Q.tolist(),'home':None,'row':{'kind':'queued-checked-clearance','index':len(recovery.retreat.rows),'request_nonce':request['nonce'],'base':list(r.base)}}
    if request.get('base_target') is not None:
        if len(Q)>1:raise RuntimeError('RECOVERY_REQUEST_USE_SEPARATE_ARM_AND_BASE_ACTIONS')
        base=np.asarray(request['base_target'],float)
        if base.shape!=(4,) or not np.all(np.isfinite(base)) or abs(base[2]-r.base[2])>1e-8:raise RuntimeError('RECOVERY_REQUEST_BASE_INVALID')
        route=recovery.mobile.locked_route(list(r.base),base.tolist(),r.arm_q(),fingers)
        if not route['valid']:raise RuntimeError('RECOVERY_REQUEST_BASE_NO_ROUTE')
        plan['base_clearance']={'base':base.tolist(),'route':route}
    return plan


def wait_for_recovery(r,recovery,events,value,reason):
    world=r.capture.scene['world'];root=r.capture.output;request_path=root/'recovery_request.json';pending=root/'recovery_pending.json'
    requests=0;seen=set();r.drive.active=False;r.halt_at_measured_state();world.pause()
    events.transition(State.SAFE_HOLD,'live_recovery.park')
    def save(status,error=None):
        row={'status':status,'reason':reason,'t':r.time(),'state_estimate':value,'q':r.arm_q().tolist(),'finger_q':r.finger_q().tolist(),'base':list(r.base),'handle':recovery.visual_current(recovery.current_D),'D_observed':recovery.current_D.tolist(),'successful_grasp_template':recovery.template,'requests':requests,'request_file':str(request_path),'deadline_wall_s':r.deadline,'object_state_restore':False,'code_hot_modified':False,'world_paused_for_planning':status=='WAITING_FOR_CHECKED_RECOVERY'}
        if error:row['last_request_error']=error
        tmp=pending.with_suffix('.tmp');tmp.write_text(json.dumps(row,indent=2));tmp.replace(pending)
    save('WAITING_FOR_CHECKED_RECOVERY')
    while time.time()<r.deadline:
        world.render()
        if request_path.exists():
            try:
                request=json.loads(request_path.read_text());nonce=str(request['nonce'])
                if nonce in seen:time.sleep(1);continue
                seen.add(nonce);requests+=1
                if requests>6:save('BOUNDED_QUEUED_RECOVERY_CANDIDATES_EXHAUSTED');continue
                action=request['action']
                if action=='checked_clearance':
                    plan=validate_clearance(r,recovery,request);recovery.retreat.rows.append(plan['row']);recovery.retreat.save()
                    world.play();row={'reason':'QUEUED_CHECKED_CLEARANCE','request_nonce':nonce,'regrasp_attempts':[]};recovery.history.append(row)
                    recovery.escape(recovery.current_D,row,safety_release=True,plans=[plan]);recovery.clearance_ready=True
                elif action=='retry':world.play()
                else:raise RuntimeError('RECOVERY_REQUEST_ACTION_INVALID')
                # Each queued request allocates one distinct bounded retry;
                # actual failures/templates/base counters are never erased.
                recovery.policy['operation_cycles']=max(recovery.policy['operation_cycles'],recovery.cycles+1)
                result=recovery.run(value,reason='RECOVERY_QUEUED_REPLAN',force_mobile=request.get('force_mobile',True),safety_release=True)
                save('RESUMED');events.transition(State.CONTINUE,'live_recovery.resume');return result
            except (RuntimeError,ValueError,KeyError,TypeError) as error:
                if isinstance(error,TaskTermination) and str(error)=='WALL_BUDGET_EXHAUSTED':raise
                events.dispatch('RECOVERY_REQUEST_FAILED','live_recovery.request',detail=str(error),traceback=traceback.format_exc())
                r.halt_at_measured_state();world.pause();save('WAITING_FOR_CHECKED_RECOVERY',str(error))
        time.sleep(1)
    save('WALL_BUDGET_EXHAUSTED');events.finish('WALL_BUDGET_EXHAUSTED',scope='live recovery waiting retains physical state');raise TaskTermination('WALL_BUDGET_EXHAUSTED')
