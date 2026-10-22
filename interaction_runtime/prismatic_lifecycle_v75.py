"""Prismatic-only evidence identity, measured-cost admission and joint winding.

No physical limits, collision predicates or frozen revolute call sites change.
Admission is an unloaded scheduling result, never a contact or safety verdict.
"""
import hashlib
import itertools
import math
from pathlib import Path
import numpy as np


def evidence_identity(episode, attempt, generation, first_step, last_step):
    if not episode or any(type(v) is not int or v < 0 for v in (attempt, generation, first_step, last_step)):
        raise ValueError('Explicit nonnegative integer evidence identity required')
    if attempt < 1 or last_step <= first_step:
        raise ValueError('Actual attempt and nonempty half-open step interval required')
    return dict(episode_id=str(episode), attempt_id=attempt,
                observation_generation=generation, first_step=first_step,
                last_step_exclusive=last_step)


def attempt_indices(arrays, identity):
    """Never fall back to a preceding segment, even if current contact is absent."""
    step=np.asarray(arrays['step'])
    keep=(step >= identity['first_step']) & (step < identity['last_step_exclusive'])
    if 'attempt_id_v75' in arrays:
        keep &= arrays['attempt_id_v75'] == identity['attempt_id']
    if 'observation_generation_v75' in arrays:
        keep &= arrays['observation_generation_v75'] == identity['observation_generation']
    return np.flatnonzero(keep)


def provenance(folder, index, indices, arrays, identity):
    """Immutable contributing chunk hashes; no duplicate raw measurements."""
    folder=Path(folder);step=arrays['step'][indices]
    sources=[]
    for c in index['chunks']:
        if len(step) and c['first_step'] <= step[-1] and c['last_step'] >= step[0]:
            path=folder/'telemetry'/c['file'];actual=hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != c['sha256']:
                raise ValueError('Contact evidence chunk integrity failure')
            sources.append(dict(file='telemetry/'+c['file'],sha256=actual))
    return dict(identity, source_files=sources, samples=len(indices),
                simulation_interval_s=([float(arrays['t'][indices[0]]),float(arrays['t'][indices[-1]])] if len(indices) else None),
                source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                previous_attempt_fallback=False, raw_measurements_private=True)


def equivalent_joint_targets(target, measured, limits, periodic_indices, margin,
                             speed_limits=None, maximum=4):
    """Existing rest pose in legal 2pi joint representations, closest first.

    Only explicitly revolute/continuous arm joints may wrap. The commanded
    configuration is never reset: ordinary collision-checked actuator travel
    must still connect measured state to the returned endpoint.
    """
    target=np.asarray(target,float);measured=np.asarray(measured,float);limits=np.asarray(limits,float)
    if target.shape != measured.shape or limits.shape != (len(target),2):
        raise ValueError('Joint dimensions disagree')
    if not np.isfinite(np.r_[target,measured,limits.ravel()]).all() or maximum < 1:
        raise ValueError('Finite bounded joint targets required')
    periodic=set(periodic_indices)
    if not periodic.issubset(range(len(target))):raise ValueError('Unknown periodic joint')
    choices=[]
    for j,q in enumerate(target):
        low,high=limits[j]+np.array([margin,-margin])
        ks=range(math.ceil((low-q)/(2*math.pi)),math.floor((high-q)/(2*math.pi))+1) if j in periodic else [0]
        values=[(q+2*math.pi*k,k) for k in ks if low < q+2*math.pi*k < high]
        if not values:return []
        # At most two admissible nearest representations per bounded joint.
        values.sort(key=lambda v:abs(v[0]-measured[j]));choices.append(values[:2])
    speed=np.ones(len(target)) if speed_limits is None else np.asarray(speed_limits,float)
    if speed.shape != target.shape or np.any(speed <= 0):raise ValueError('Positive joint speeds required')
    result=[]
    for row in itertools.product(*choices):
        q=np.array([v[0] for v in row]);w=[v[1] for v in row]
        result.append(dict(q=q.tolist(),winding=w,
            joint_travel_lower_bound_s=float(np.max(abs(q-measured)/speed)),
            pose_equivalent_by_revolute_periodicity=True))
    return sorted(result,key=lambda r:r['joint_travel_lower_bound_s'])[:maximum]


def finite_admission(remaining, costs, *, includes_exit=False):
    """Reserve a complete minimum FIT+VALIDATION chain BEFORE reacquisition.

    Costs are explicit measured or timed-plan bounds, with provenance. Unknown
    costs return UNKNOWN, not fake zero or an enlarged episode deadline.
    """
    required=('approach','closure_hold','FIT','VALIDATION','exit')
    missing=[k for k in required if k not in costs or not costs[k].get('available')]
    if missing:return dict(admit=False,status='COST_UNKNOWN',missing=missing,physical_failure=False)
    for k in required:
        c=costs[k]
        if not c.get('source') or not all(np.isfinite(c[n]) and c[n] >= 0 for n in ('wall_s','sim_s')):
            raise ValueError('Nonnegative finite costs and measured/timed source required: '+k)
    need={n:sum(costs[k][n] for k in required if includes_exit or k != 'exit') for n in ('wall_s','sim_s')}
    # EpisodeBudget.remaining already subtracts the existing exit reserve.
    # Charge only any measured exit cost ABOVE that reserve, never twice.
    if not includes_exit:
        for n in need:
            reserve=remaining.get('reserved_'+n)
            if reserve is None:return dict(admit=False,status='EXIT_RESERVE_UNKNOWN',physical_failure=False)
            need[n]+=max(0.,costs['exit'][n]-reserve)
    okay=all(remaining[n] >= need[n] for n in need)
    return dict(admit=okay,status='COMPLETE_CHAIN_FITS' if okay else 'INSUFFICIENT_CHAIN_BUDGET',
                required=need,available={n:remaining[n] for n in need},cost_sources=costs,
                includes_complete_validation=True,physical_failure=False,
                episode_budget_or_safety_limits_changed=False)
