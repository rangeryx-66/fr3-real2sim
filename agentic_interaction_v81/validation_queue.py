"""Persistent independent-validation requests, bounded safe worker handover."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from agent_command_protocol_v12 import atomic_json
from .model_packets import sha


def requests(ops):
    path=ops.r.output/'VALIDATION_REQUESTS.json'
    return json.loads(path.read_text()) if path.exists() else dict(requests=[])


def write(ops,data):atomic_json(ops.r.output/'VALIDATION_REQUESTS.json',data)


def identity(action):return hashlib.sha256(json.dumps(action,sort_keys=True).encode()).hexdigest()


def enqueue(ops,command):
    data=requests(ops)
    for action in command.get('segments',[]):
        if not action['role'].startswith('VALIDATION'):continue
        key=identity(action)
        existing=next((r for r in data['requests'] if r['id']==key),None)
        if existing:
            if ops.r.job.get('v81_active_recovery') and existing['status']=='PHYSICAL_FAILURE':
                existing.setdefault('failed_attempts',[]).append(deepcopy({
                    k:v for k,v in existing.items() if k!='failed_attempts'}))
                existing.update(status='PENDING_REPLAN',approval_reason=command.get('reason'),
                    inspected_image_sha256=command.get('inspected_image_sha256'))
                existing.pop('result',None)
            continue
        data['requests'].append(dict(id=key,action=deepcopy(action),status='PENDING_FIT',
            approved_before_execution=True,approval_reason=command.get('reason'),
            inspected_image_sha256=command.get('inspected_image_sha256'),
            episode_id=ops.r.job['episode_id'],created_unix_s=time.time()))
    write(ops,data)


def _model_state(ops):
    expected=max((s['id'] for s in ops.r.record.segments if s['role'].startswith('FIT')),default=0)
    if expected==0:return dict(state='DATA_INSUFFICIENT',reason='NO_COMPLETED_FIT_DATA',
        needed='qualified bidirectional FIT before independent VALIDATION')
    folder=ops.r.output/'model';pub=folder/'MODEL_PUBLICATION.json';life=folder/'WORKER_LIFECYCLE.json'
    lifecycle=json.loads(life.read_text()) if life.exists() else {}
    if not pub.exists():
        return dict(state='WORKER_FAILED' if lifecycle.get('state')=='WORKER_FAILED' else 'MODEL_COMPUTING',
            expected_FIT_id=expected,worker=lifecycle)
    published=json.loads(pub.read_text())
    if published.get('latest_FIT_id')!=expected:
        return dict(state='WORKER_FAILED' if lifecycle.get('state')=='WORKER_FAILED'
            and lifecycle.get('latest_FIT_id')==expected else 'MODEL_COMPUTING',
            expected_FIT_id=expected,published_FIT_id=published.get('latest_FIT_id'),worker=lifecycle)
    model_path=Path(published['model']);readiness_path=Path(published['readiness'])
    ready=json.loads(readiness_path.read_text());model=json.loads(model_path.read_text())
    if (sha(model_path)!=published['model_sha256'] or ready['model_sha256']!=published['model_sha256']
            or ready['episode_id']!=ops.r.job['episode_id']):
        return dict(state='WORKER_FAILED',reason='IMMUTABLE_MODEL_PUBLICATION_MISMATCH')
    usable=ready.get('qualified_bidirectional') and model.get('status')=='updated'
    return dict(state='READY' if usable else 'DATA_INSUFFICIENT',publication=published,readiness=ready,
        needed=dict(positive_bins=max(0,8-ready.get('positive_bins',0)),
            negative_bins=max(0,8-ready.get('negative_bins',0)),
            distinct_eligible_trajectories=max(0,2-ready.get('distinct_eligible_trajectories',0)),
            excitation=ready.get('information'),parameter_support=(model.get('selected')or{}).get('conditionally_supported')))


def model_state(ops):
    try:return _model_state(ops)
    except (OSError,json.JSONDecodeError,KeyError,TypeError,ValueError) as error:
        if not ops.r.job.get('v81_active_recovery'):raise
        # Only immutable numerical publication files are read above. A worker
        # publication failure is not a controller/sensor fault and must not
        # discard an already completed physical attempt after safe unloading.
        return dict(state='WORKER_FAILED',reason='NUMERICAL_PUBLICATION_UNAVAILABLE',
            error=repr(error),qualified_model_approved=False,manipulation_may_continue=True)


def mark(ops,action,status,**metadata):
    data=requests(ops);key=identity(action)
    for row in data['requests']:
        if row['id']==key:row.update(status=status,**metadata)
    write(ops,data)


def admit(ops,action):
    """No Agent wait. Measured-cost, finite, guarded numerical handover.

    Full native physics/guards continue; this is not a paused-PhysX success.
    If loading becomes unstable, release through the original finite owner.
    """
    status=model_state(ops);held=0.;r=ops.r
    publication=r.output/'model/MODEL_PUBLICATION.json'
    prior=json.loads(publication.read_text()) if publication.exists() else {}
    # This uses worker latency, never an obsolete model's coefficients. The
    # original acquisition hold is unchanged. A stopped endpoint may step
    # real physics while a finite numeric job finishes; instability unloads.
    latency=float(prior.get('worker_wall_s',0.))
    wall_cap=min(10.,max(1.,1.5*latency))
    resources=r.episode_budget.remaining(r.tick*r.dt)
    wall_cap=min(wall_cap,resources['wall_s'])
    sim_cap=min(3. if latency else 1.,resources['sim_s'])
    deadline=time.monotonic()+wall_cap
    handover=dict(measured_previous_worker_wall_s=latency,wall_cap_s=wall_cap,
        simulation_cap_s=sim_cap,acquisition_hold_unchanged=True,
        real_physics_and_original_protections_active=True,Agent_wait=False)
    segment=r.record.segments[-1] if r.record.segments else {}
    previous=segment.get('result',{})
    stable=(bool(previous.get('safely_stopped') and previous.get('physical_transmission_valid')
        and previous.get('status') in ('TARGET_REACHED','SAFE_PARTIAL_PROGRESS'))
        if segment.get('first_step',-1)>=getattr(ops,'_v81_block_first_record',0)
        else bool(getattr(ops,'_v81_current_acquisition_verified',False)))
    begin_aperture=float(r.rows[-1]['aperture_m']) if r.rows else None
    begin_slip=float(r.rows[-1]['relative_translation_slip_m']) if r.rows else None
    while (status['state']=='MODEL_COMPUTING' and stable and held<sim_cap-r.dt/2
            and time.monotonic()<deadline):
        if r.supervisor.latched is not None:break
        step=min(.05,sim_cap-held)
        r.hold(step);held=min(sim_cap,held+step)
        row=r.rows[-1]
        stable=(abs(float(row['aperture_m'])-begin_aperture)<.0005
            and abs(float(row['relative_translation_slip_m'])-begin_slip)<r.closure_policy['max_slip_m'])
        status=model_state(ops)
    if status['state']!='READY' or not stable:
        mark(ops,action,'PENDING_MODEL' if status['state']=='MODEL_COMPUTING' else
            'NEEDS_EXCITATION' if status['state']=='DATA_INSUFFICIENT' else
            'WORKER_FAILED' if status['state']=='WORKER_FAILED' else 'PENDING_REPLAN',
            last_model_state=status,finite_guarded_hold_sim_s=held,contact_stable=stable,
            numerical_handover_budget=handover)
        return dict(ready=False,model_state=status,finite_guarded_hold_sim_s=held,
            request_preserved=True,numerical_handover_budget=handover,
            reason='CONTACT_UNSTABLE' if not stable else status['state'])
    # Recheck after any actual hold: no stale measured-state route approval.
    route=ops.check_successor(action)
    if not route.get('valid'):
        mark(ops,action,'PENDING_REPLAN',native_rejection=route)
        return dict(ready=False,reason='VALIDATION_CURRENT_ROUTE_REJECTED',native_check=route,
            model_state=status,request_preserved=True)
    pub=status['publication'];folder=r.output/'validation_request_seals';folder.mkdir(exist_ok=True)
    seal=dict(id=identity(action),action=action,publication=pub,
        readiness=status['readiness'],model_sha256=pub['model_sha256'],
        initialized_from_measured_coordinate=r.actual(),future_validation_used=False,
        finite_guarded_hold_sim_s=held,numerical_handover_budget=handover)
    atomic_json(folder/(identity(action)+'.json'),seal)
    mark(ops,action,'SEALED_READY',seal=str(folder/(identity(action)+'.json')))
    return dict(ready=True,model_state=status,finite_guarded_hold_sim_s=held,seal=seal)


def boundary(ops):
    data=requests(ops)
    pending=[r for r in data['requests'] if r['status'] not in ('COMPLETE','PHYSICAL_FAILURE')]
    if not pending:return dict(pending=[],state='NONE')
    if (ops.g['recovery_state']!='UNLOADED_OBSERVATION' or ops.r.supervisor.latched is not None
            or ops.r.grasp_reference is not None or not ops.g.get('at_safe_park')
            or max(ops.r.filtered())>ops.r.policy.data['force']['unloaded_pad_n']
            or not ops.r.supervisor.force.settled()):
        return dict(state='NO_SAFE_DECISION_BOUNDARY',pending=pending)
    status=model_state(ops)
    # Workers are bounded to 60 s. A verified parked robot may wait for active
    # computation, never indefinitely for insufficient data or a failed worker.
    remaining=ops.r.episode_budget.remaining(ops.r.tick*ops.r.dt)
    deadline=time.monotonic()+min(60.,max(0.,remaining['wall_s']))
    while (not ops.r.job.get('v84_abc') and status['state']=='MODEL_COMPUTING' and time.monotonic()<deadline):
        if not any(not f.done() for f in ops.updates):break
        time.sleep(.05);status=model_state(ops)
    if status['state']=='READY':
        for row in pending:mark(ops,row['action'],'READY_FOR_CURRENT_STATE_REPLAN',publication=status['publication'])
    current=requests(ops)
    pending=[r for r in current['requests'] if r['status'] not in ('COMPLETE','PHYSICAL_FAILURE')]
    return dict(state=status['state'],pending=pending,model=status,
        priority=('Keep the ORIGINAL pending VALIDATION; complete it when a qualified model and current-state route are ready. '
            'While numerical work is pending or failed, useful manipulation/FIT and safe recovery remain available.'
            if ops.r.job.get('v81_active_recovery') else
            'Resume the ORIGINAL pending VALIDATION after current-state native planning and fresh Agent grasp approval; optional range expansion must wait.'))


def retry_model(ops,command):
    """One new immutable job per failed FIT snapshot, only while safely unloaded.

    A repeat failure remains visible and prompts a different strategy; it never
    ends manipulation, approves stale coefficients, or refits validation data.
    """
    from .task_recovery import enabled,unloaded
    from .model_packets import submit
    import threading
    if not enabled(ops) or not unloaded(ops):
        return dict(retried=False,reason='ACTIVE_RECOVERY_UNLOADED_BOUNDARY_REQUIRED')
    if any(not future.done() for future in ops.updates):
        return dict(retried=False,state='MODEL_COMPUTING',request_preserved=True)
    current=model_state(ops)
    if current['state'] not in ('WORKER_FAILED','MODEL_COMPUTING'):
        return dict(retried=False,state=current['state'],request_preserved=True)
    fit=[s for s in ops.r.record.segments if s.get('completed') and s['role'].startswith('FIT')]
    key=tuple((s['id'],s['last_step']) for s in fit)
    retried=getattr(ops,'_v81_retried_fit_snapshots',set())
    if not key or key in retried:
        return dict(retried=False,state='NUMERICAL_DIAGNOSIS_REQUIRED',request_preserved=True,
            manipulation_may_continue=True,reason='SAME_FAILED_FIT_SNAPSHOT_ALREADY_RETRIED')
    # Use the same producer barrier as the original recorder, never assume that
    # in-memory completed records imply committed numeric chunks.
    ops.r.record.flush();ready=threading.Event();ops.r.record.queue.put(ready)
    retried.add(key);ops._v81_retried_fit_snapshots=retried
    ops._v81_last_packet_identity=None
    submit(ops,ready)
    return dict(retried=True,state='MODEL_COMPUTING',request_preserved=True,
        same_production_estimator=True,validation_used_for_fit=False,
        immutable_new_packet=True,reason=command['reason'])
