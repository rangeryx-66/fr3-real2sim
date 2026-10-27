"""Measured-state release, local separation, then independent parking.

No predicted parking route is an operation admission certificate. Every actual
edge retains native collisions, timing, limits and hard supervision. A local
clearance is reported separately and never relabelled as verified parking.
"""
import hashlib,json,math
import numpy as np
from contextlib import contextmanager
from unittest.mock import patch
from interaction_runtime.policy import SafetyStop

EXIT_PATH_NOT_FOUND='EXIT_PATH_NOT_FOUND'
LOCAL_RETREAT_NOT_FOUND='LOCAL_RETREAT_NOT_FOUND'
SAFE_RELEASE_NOT_CONFIRMED='SAFE_RELEASE_NOT_CONFIRMED'
SAFE_HOLD_NOT_ESTABLISHED='SAFE_HOLD_NOT_ESTABLISHED'
HARD_SAFETY_STOP='HARD_SAFETY_STOP'
PARTIAL={EXIT_PATH_NOT_FOUND,LOCAL_RETREAT_NOT_FOUND,SAFE_RELEASE_NOT_CONFIRMED,SAFE_HOLD_NOT_ESTABLISHED}


def measured(r):
    return dict(coordinate_native=float(r.actual()),moving_pose=np.asarray(r.moving()).tolist(),
        robot_q=np.asarray(r.robot.get_joint_positions()).tolist(),
        retained_command=np.asarray(r.qtarget).tolist(),base=r.base.tolist(),
        t_sim=float(r.tick*r.dt),latch=r.supervisor.latched)


def safe_hold(ops):
    from agentic_interaction_v81.recovery_supervisor import trusted
    r=ops.r
    if not trusted(ops) or np.max(abs(r.g['qvelocity']))>1e-10:return False
    q=np.asarray(r.robot.get_joint_positions())
    if not r.station_motion.ready(r.rows[-1]['t'],q[r.arm]):return False
    # Acquisition assigns grasp_reference only after bilateral verification.
    # A failed acquisition can still own the original permitted pad/handle
    # contact during RELEASE. Use the real-time owner's identical phase scope;
    # all other native collisions and the per-step contact guard remain active.
    okay,_=r.collision.check(r.model.poses(q[r.arm],r.base,finger_q=q[r.fingers]),
        r.moving(),r.intentional_contact())
    return bool(okay)


def settle(ops):
    # Existing supervised stationary dwell, including actual object motion;
    # no requirement to be inside the unrelated native-rest joint cell.
    r=ops.r
    for _ in range(math.ceil(2./.3)):
        r.hold(.3)
        if safe_hold(ops):return True
    return False


def fingerprint(r,target):
    data=dict(state=measured(r),target=np.asarray(target).tolist())
    # Time is not a new scene; changing a request's wording is not recovery.
    data['state'].pop('t_sim')
    return hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()


def local_routes(ops,audit):
    """Original bounded 4/8 cm directions, without an onward-parking demand."""
    from native_g1_contact_recovery import checked_route
    from agentic_interaction_v81.exit_clearance import ParkingClearance
    r=ops.r;g=dict(ops.g,collision=ParkingClearance(r.collision))
    T=np.asarray(r.tcp()).copy();q=np.asarray(r.robot.get_joint_positions())[r.arm]
    directions=[('tool_backoff',-T[:3,2]),('tool_backoff_up',-T[:3,2]+[0,0,1])]
    geometry=r.output/'agent_geometry/observed_geometry.json';plan=r.output/'visual_plan.json'
    if geometry.exists() and plan.exists():
        normal=np.asarray(json.loads(geometry.read_text())['normal_world'])
        start=float(json.loads(plan.read_text())['path'][0]['state'])
        normal=ops.articulation().delta(float(r.actual())-start)[:3,:3]@normal
        directions += [('current_panel_outward',normal),('outward_up',normal+[0,0,1])]
    bridge=checked_route(g,[q,r.qtarget[r.arm]])
    audit.append(dict(kind='measured_command_bridge',check=bridge))
    if not bridge['valid']:return
    for distance in [.04,.08]:
        for name,direction in directions:
            direction=np.asarray(direction,float);length=np.linalg.norm(direction)
            if not np.isfinite(length) or length<1e-9:continue
            direction/=length;path=[q.tolist()];seed=q.copy();why=None
            for d in np.linspace(0,distance,int(round(distance/.002))+1)[1:]:
                goal=T.copy();goal[:3,3]+=d*direction
                seed=r.solver.solve(goal,r.base,seed,starts=1)
                if seed is None:why='NO_RETREAT_IK';break
                path.append(seed.tolist())
            check=checked_route(g,[r.qtarget[r.arm].tolist()]+path[1:]) if why is None else dict(valid=False,reason=why)
            if check['valid']:check=checked_route(g,path)
            row=dict(kind=name,distance_m=distance,check=check);audit.append(row)
            if check['valid']:yield path,row


def parking_targets(ops):
    import native_g1_safe_park as native
    if ops.articulation().joint_type=='prismatic':
        from interaction_runtime.prismatic_park_v75 import parking_targets as equivalent
        yield from equivalent(ops.g,native.parking_targets)
    else:
        # Native rest's other six joints stay identical. +/-20deg were early
        # task headings, not a physical safety envelope. Search the SAME rest
        # family over legal shoulder headings, retaining native endpoint,
        # complete swept/timed collision checks and measured rest-cell proof.
        original=list(native.parking_targets(ops.g))
        yield from original
        r=ops.r;home=np.asarray(original[0][1]).copy()
        measured_q=np.asarray(r.robot.get_joint_positions())[r.arm]
        lo,hi=np.asarray(r.model.limits)[0];margin=r.policy.data['joint_margin_rad']
        step=math.radians(20.)  # existing native shoulder-heading spacing
        headings=[float(measured_q[0])]+[k*step for k in range(
            math.ceil((lo+margin)/step),math.floor((hi-margin)/step)+1)]
        seen=[np.asarray(q) for _,q in original]
        for yaw in sorted(headings,key=lambda value:abs(value-measured_q[0])):
            target=home.copy();target[0]=yaw
            if r.model.margin(target)<=margin or any(np.allclose(target,q,atol=1e-10,rtol=0) for q in seen):continue
            seen.append(target)
            yield 'native_rest_heading_%+.6f'%yaw,target


def release_park(ops):
    """Execute stages only from fresh confirmed states, never clear a latch."""
    from native_g1_contact_recovery import release_geometry
    import native_g1_safe_park as native
    from native_g1_safe_park import verify_park
    from agentic_interaction_v81.exit_routes import timed_returns,confirmed_unloaded
    r=ops.r;g=ops.g
    out=dict(safe=False,status=SAFE_HOLD_NOT_ESTABLISHED,
        operation_result_unchanged=True,release=dict(confirmed=False),
        retreat=dict(reached=False),parking=dict(reached=False),stages=[])
    def stage(name):
        state=measured(r);out['stages'].append(dict(stage=name,state=state))
        r.event('STAGED_EXIT_'+name,state=state)
    def end(status):
        out['status']=status;out['reason']=status
        out['hold_confirmed']=safe_hold(ops) if r.supervisor.latched is None else False
        if (status==SAFE_HOLD_NOT_ESTABLISHED and not out['hold_confirmed']
                and r.supervisor.latched is None):
            from agentic_interaction_v81.recovery_supervisor import trusted
            from agentic_interaction_v84.operations import contact_trusted
            commands_stopped=(np.max(abs(r.g['qvelocity']))<=1e-10 and
                np.max(abs(getattr(r,'base_velocity',np.zeros(2))))<=1e-10)
            if commands_stopped and trusted(ops):
                # A finite dwell's failure is not proof of lost control. Keep
                # real-physics C supervision; no B continuation is authorized.
                g['v87_hold_pending']=True
                ops.abc_state='C';g['recovery_state']='LOADED_CRITIQUE'
                out.update(contact_retained=bool(contact_trusted(ops)),unloaded=False,hold_pending=True,
                    protective_hold_only=True,maximum_C_wall_s=20.,maximum_C_sim_s=5.)
        if status==SAFE_RELEASE_NOT_CONFIRMED and out['hold_confirmed'] and r.grasp_reference is not None:
            from agentic_interaction_v84.operations import contact_trusted
            if contact_trusted(ops):
                ops.abc_state='C';g['recovery_state']='LOADED_CRITIQUE'
                out.update(contact_retained=True,unloaded=False,
                    maximum_C_wall_s=20.,maximum_C_sim_s=5.)
        return out
    if r.supervisor.latched is not None:return end(HARD_SAFETY_STOP)
    stage('ACTUAL_STATE')
    # Request the ORIGINAL exit phase before stop, as the existing release
    # owner does. V8.7 still brakes under the last accepted motion limits,
    # then restores RELEASE. Its subsequent stationary dwell must retain
    # access to the existing exit reserve when ordinary work is exhausted.
    g['phase']='RELEASE';r.stop_command()
    if not settle(ops):return end(SAFE_HOLD_NOT_ESTABLISHED)
    g.pop('v87_hold_pending',None)
    # Existing verified parking may be reused only after measured revalidation.
    if g.get('at_safe_park') and g.get('last_exit_plan') and confirmed_unloaded(r):
        park=verify_park(g,g['last_exit_plan'])
        if park['reached']:
            out.update(safe=True,status='SAFE_PARK_REACHED',park=park,already_verified_parked=True)
            out['release']['confirmed']=True;out['parking']=park;return out
    if not confirmed_unloaded(r):
        audit=release_geometry(g,r.rows[-1]['aperture_m']);out['release']['jaw_sweep']=audit
        if not audit['valid']:return end(SAFE_RELEASE_NOT_CONFIRMED)
        stage('RELEASE_ALLOWED_FROM_STABLE_STATE')
        g['phase']='RELEASE';r.grasp_reference=None;g['reference']=None;r.reference_drive=None
        width=float(r.rows[-1]['aperture_m'])
        count=max(2,math.ceil(abs(.084-width)/(r.policy.data['jaw_release_m_s']*r.dt)))
        for w in np.linspace(width,.084,count+1):r.jaw_width=float(w);r.step()
        r.hold(.3)
        if not confirmed_unloaded(r):r.hold(1.)
        if not confirmed_unloaded(r):return end(SAFE_RELEASE_NOT_CONFIRMED)
    if not settle(ops):return end(SAFE_RELEASE_NOT_CONFIRMED)
    out['release'].update(confirmed=True,state=measured(r));stage('RELEASE_CONFIRMED')
    out['retreat']['candidates']=[]
    memory=getattr(ops,'_v87_failed_exit_edges',set())
    ops._v87_failed_exit_edges=memory
    for path,row in local_routes(ops,out['retreat']['candidates']):
        key=fingerprint(r,path)
        if key in memory:row['skipped_unchanged_failed_route']=True;continue
        try:r.move_path(path[1:],'CLEARANCE_RETREAT')
        except SafetyStop:raise
        except RuntimeError as error:
            if not str(error).startswith('MOTION_PATH_UNAVAILABLE:'):raise
            memory.add(key);row['timed_route_rejection']=str(error);continue
        if not settle(ops):return end(SAFE_HOLD_NOT_ESTABLISHED)
        if not confirmed_unloaded(r):return end(SAFE_RELEASE_NOT_CONFIRMED)
        out['retreat'].update(reached=True,state=measured(r));stage('LOCAL_CLEARANCE_REACHED');break
    if not out['retreat']['reached']:
        # Failure of a particular 4/8cm Cartesian direction does not disprove
        # an independently checked joint-space parking route. That route's
        # ORIGINAL timed_returns checks include its current start, every edge,
        # native hand contact envelopes and actual stopped endpoint. Do not
        # fabricate a clearance certificate or authorize any new contact.
        out['retreat']['independent_parking_search']=True
    # Fresh wrist sensing is a separate unloaded boundary, followed by a new
    # current moving-body state for EVERY parking target/edge.
    try:out['post_clearance_observation']=ops.observe()
    except SafetyStop:raise
    except (OSError,RuntimeError,ValueError) as error:
        # Wrist presentation/segmentation cannot veto an independently native
        # checked safety exit. Re-read trusted physical hold/unload below;
        # hard SafetyStop above still propagates without another movement.
        out['post_clearance_observation_error']=repr(error)
    if not safe_hold(ops) or not confirmed_unloaded(r):return end(SAFE_HOLD_NOT_ESTABLISHED)
    stage('FRESH_POST_CLEARANCE_STATE');out['parking']['candidates']=[]
    with timed_returns(r) as timed:
        for region,target in parking_targets(ops):
            key=fingerprint(r,target);row=dict(region=region);out['parking']['candidates'].append(row)
            if key in memory:row['skipped_unchanged_failed_route']=True;continue
            current=np.asarray(r.robot.get_joint_positions())[r.arm]
            path,why=native.route_to_joint(g,current,target)
            row.update(route_found=path is not None,reason=why)
            if (path is not None and r.job.get('v87_staged_exit_fixture')
                    and not getattr(ops,'_v87_park_rejection_injected',False)
                    and getattr(r,'v81_task_progress',{}).get('maximum_measured_native',-np.inf)
                        >=r.job['v81_task_goal_native']):
                # Conservative planner rejection only; no fabricated collision,
                # no force injection and no modification of the actual scene.
                ops._v87_park_rejection_injected=True
                row.update(engineering_injected=True,original_native_route_found=True,
                    reason='INJECTED_PREFERRED_PARK_ROUTE_UNAVAILABLE_AFTER_GOAL')
                memory.add(key);continue
            if path is None:memory.add(key);continue
            try:r.move_path(path[1:],'SAFE_PARK_MOTION')
            except SafetyStop:raise
            except RuntimeError as error:
                if not str(error).startswith('MOTION_PATH_UNAVAILABLE:'):raise
                memory.add(key);row['timed_route_rejection']=str(error);continue
            if not settle(ops):return end(SAFE_HOLD_NOT_ESTABLISHED)
            plan=dict(parking_q=np.asarray(target).tolist(),parking_region=region)
            park=verify_park(g,plan);row['measured_verification']=park
            if not park['reached']:
                memory.add(key)
                if not settle(ops):return end(SAFE_HOLD_NOT_ESTABLISHED)
                continue
            g.update(at_safe_park=True,last_exit_plan=plan,recovery_state='UNLOADED_OBSERVATION')
            out.update(safe=True,status='SAFE_PARK_REACHED',park=park)
            out['parking'].update(reached=True,verification=park)
            stage('MEASURED_PARK_REACHED');out['timed_parking_audit']=timed;return out
        out['timed_parking_audit']=timed
    return end(EXIT_PATH_NOT_FOUND if out['retreat']['reached'] else LOCAL_RETREAT_NOT_FOUND)


def recovery_boundary(ops,command,result,exit_result):
    """A missing park route preserves completed motion and a verified hold."""
    from agentic_interaction_v81.recovery_supervisor import released,released_hold
    from .recovery import invalidate
    if exit_result.get('hold_pending'):
        from agentic_interaction_v81.recovery_supervisor import trusted
        if trusted(ops):
            ops.abc_state='C';ops.g['recovery_state']='LOADED_CRITIQUE'
            return dict(result,status='RECOVERABLE_FAILURE',outcome='RECOVERABLE_FAILURE',
                reason='MEASURED_STABILITY_PENDING',episode_termination_required=False,
                recovery=dict(control_trusted=True,hold_verified=False,park_verified=False,
                    protective_hold_only=True,motion_resume_allowed=False,
                    World_recreated=False,object_state_reset=False,original_goal_preserved=True))
    if released(ops) and released_hold(ops):
        ops._v86_exit_pending=True
        invalidate(ops);ops.g['recovery_state']='UNLOADED_OBSERVATION'
        return dict(result,status='RECOVERABLE_FAILURE',outcome='RECOVERABLE_FAILURE',
            exit_status=exit_result['status'],episode_termination_required=False,
            operation_result_preserved=True,
            recovery=dict(hold_verified=True,park_verified=False,exit_pending=True,
                release_confirmed=bool(exit_result.get('release',{}).get('confirmed')),
                local_retreat_reached=bool(exit_result.get('retreat',{}).get('reached')),
                World_recreated=False,object_state_reset=False,
                original_goal_preserved=True,current_coordinate_native=float(ops.r.actual()),
                next='fresh checked local retreat/alternative parking; do not repeat the rejected edge'))
    return dict(result,status='CONTROL_STATE_UNTRUSTED',outcome='UNRECOVERABLE',
        exit_status=exit_result['status'],reason='SAFE_HOLD_OR_RELEASE_NOT_CONFIRMED',
        recovery=dict(hold_verified=False,park_verified=False,automatic_motion_allowed=False))


@contextmanager
def motion_scope(ops):
    if ops.articulation().joint_type!='prismatic':
        yield;return
    import interaction_runtime.prismatic_candidate_v75 as candidates
    import interaction_runtime.prismatic_chain_v75 as chain
    import interaction_runtime.prismatic_cost_v75 as cost
    from .prismatic_motion import evaluate_prismatic_motion
    from .staged_cost import admit_staged_motion
    with patch.object(candidates,'evaluate_chain',evaluate_prismatic_motion),\
         patch.object(chain,'evaluate_chain',evaluate_prismatic_motion),\
         patch.object(cost,'admit',admit_staged_motion):yield


def operations_class(parent):
    class StagedExitOperations(parent):
        def request_plan(self,command):
            with motion_scope(self):return super().request_plan(command)
        def attempt(self,command):
            with motion_scope(self):return super().attempt(command)
        def redeclare_actions(self,command):
            with motion_scope(self):return super().redeclare_actions(command)
    return StagedExitOperations
