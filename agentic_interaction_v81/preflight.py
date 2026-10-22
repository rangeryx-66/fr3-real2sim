"""Read-only V8 baseline, native-range and FIT-support preflight.

No simulation imports, physics-file reads, action publication or coefficient
changes. This is not a physical feasibility or identification certificate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
BASELINE = '2f021639aea65e6ae859bac9a3385d49db10212f'


def baseline_audit(root=ROOT):
    """Verify every committed V8 source byte, not just a report reference."""
    from agentic_interaction_v8.cli import audit
    result = audit()
    # Untracked V8.1 modules are expected. A changed frozen tracked path is not.
    changed = subprocess.check_output(
        ['git', 'diff', BASELINE, '--name-only', '--',
         'interaction_runtime', 'scripts', 'configs', 'config',
         'agentic_interaction_v8', '.agents/skills/agentic-real2sim'],
        cwd=root, text=True).splitlines()
    if changed:
        raise ValueError('V8_BASELINE_MODIFIED:' + repr(changed))
    return dict(baseline_commit=BASELINE, modified_frozen_paths=changed,
                **{k: result[k] for k in ('physical_source_files',
                    'frozen_revolute_source_files', 'release_source_files', 'failures')})


def native_range(joint_type, limits, target):
    if joint_type not in ('revolute', 'prismatic'):
        raise ValueError('SUPPORTED_JOINT_TYPE_REQUIRED')
    lo, hi = map(float, limits)
    if not all(math.isfinite(v) for v in (lo, hi, target)) or not lo < hi:
        raise ValueError('FINITE_ORDERED_NATIVE_LIMITS_REQUIRED')
    return dict(joint_type=joint_type, units='rad' if joint_type == 'revolute' else 'm',
                target_native=target, limits_native=[lo, hi],
                inside_native_limits=lo <= target <= hi,
                physical_reachability_verified=False)


def support_requirements(dt=1/240, window=121, edge=121,
                         desired_retained_s=3.0):
    """Mirror the frozen processing support, without changing its filtering.

    A planning lower bound assumes contiguous, valid, resolved sliding samples.
    Actual eligible samples must still come from the completed production worker.
    """
    if not math.isfinite(dt) or dt <= 0 or not math.isfinite(desired_retained_s) or desired_retained_s <= 0:
        raise ValueError('POSITIVE_FINITE_SAMPLE_AND_SUPPORT_DURATION_REQUIRED')
    if type(window) is not int or window < 7 or window % 2 != 1:
        raise ValueError('ODD_SG_WINDOW_REQUIRED')
    if type(edge) is not int or edge < 0:
        raise ValueError('NONNEGATIVE_BOUNDARY_EXCLUSION_REQUIRED')
    edge = max(edge, window//2 + 2)
    # process() rejects n < 2*edge+5; retained span uses N-1 time intervals.
    retained = max(5, math.ceil(desired_retained_s/dt) + 1)
    samples = 2*edge + retained
    return dict(physics_hz=1/dt, window=window, edge_samples=edge,
                centered_support_samples=window//2+2,
                minimum_process_records=2*edge+5,
                requested_retained_span_s=desired_retained_s,
                required_contiguous_records=samples,
                minimum_observed_span_s=(samples-1)*dt,
                not_a_guarantee='Requires actual transmission-qualified contiguous sliding; startup, gaps, stiction and warnings may remove support.')


def fit_feedback(model):
    """Explain missing support from actual frozen-worker output; never refit.

    Deliberately makes no decision about which next physical action to execute.
    Roles cannot establish measured direction/excitation; those require telemetry.
    """
    trajectories=[]
    for s in model.get('segments', []):
        if not s['role'].startswith('FIT'):
            continue  # independent VALIDATION cannot fill a FIT shortfall
        blocks=[]
        for b in s.get('blocks', []):
            q=b.get('prismatic_sliding_qualification')
            eligible=int(q['qualified_samples']) if q and q.get('applied') else int(b.get('valid', 0))
            blocks.append(dict(records=b.get('records', 0), eligible_samples=eligible,
                               status=b.get('status', 'processed'), processing=b.get('processing')))
        trajectories.append(dict(id=s['id'], role=s['role'], quality=s.get('quality'),
                                 eligible_samples=sum(b['eligible_samples'] for b in blocks), blocks=blocks))
    selected=model.get('selected') or model.get('constant_model') or {}
    names=selected.get('parameter_names', [])
    supported=selected.get('conditionally_supported', selected.get('supported', []))
    values=selected.get('parameters', [])
    estimates={name: value if i < len(supported) and supported[i] else None
               for i,(name,value) in enumerate(zip(names,values))}
    return dict(FIT_trajectories=trajectories,
                eligible_trajectory_count=sum(t['eligible_samples'] > 0 for t in trajectories),
                any_declared_FIT_without_support=any(t['eligible_samples'] == 0 for t in trajectories),
                conditional_parameters=estimates,
                measured_bidirectionality='NOT_ESTABLISHED_BY_ROLE_LABELS; inspect eligible signed-velocity telemetry',
                independent_excitation='NOT_ESTABLISHED_BY_PARAMETER_FIT; inspect regressor information and phase-matched residual',
                validation_used=False, original_model_modified=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path)
    a=p.parse_args()
    out=dict(status='OFFLINE_PREFLIGHT_ONLY', baseline=baseline_audit(),
             SG121_support=support_requirements(), physical_episode_started=False)
    if a.model:
        raw=a.model.read_bytes()
        out.update(model_sha256=hashlib.sha256(raw).hexdigest(),
                   FIT_feedback=fit_feedback(json.loads(raw)))
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
