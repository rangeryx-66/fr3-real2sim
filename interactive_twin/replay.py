"""Restore the same pre-physics estimated-model safety gate during replay.

The external command sequence remains fixed. This model is neither simulator
truth nor a newly fitted held-out model; it was available in the reference before
P1 began. A timestamp prevents using that estimate earlier than it existed.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def _accepted_support(fit, support):
    support = np.asarray(support, dtype=float)
    if support.ndim != 3 or support.shape[1:] != (4, 4) or len(support) < 12 or not np.isfinite(support).all():
        raise ValueError('REPLAY_SAFETY_MODEL_SUPPORT_INCOMPLETE')
    if float(np.max(np.linalg.norm(support[:, :3, 3]-support[0, :3, 3], axis=1))) < .005:
        raise ValueError('REPLAY_SAFETY_MODEL_EXCITATION_INSUFFICIENT')
    if fit.get('joint_type') != 'revolute' or fit.get('confidence', 0) <= .9:
        raise ValueError('REPLAY_SAFETY_MODEL_NOT_ACCEPTED_REVOLUTE')
    axis = np.asarray(fit['revolute']['axis'], dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or not np.isclose(np.linalg.norm(axis), 1., atol=1e-4):
        raise ValueError('REPLAY_SAFETY_MODEL_INVALID_AXIS')
    rotation = Rotation.from_matrix(support[-1, :3, :3] @ support[0, :3, :3].T).as_rotvec()
    return support, 1. if rotation @ axis >= 0. else -1.


def _adopted_safety_model(path):
    """Restore D's frozen model with this physics run's own safety anchor.

    D observations certify the estimate only. They are never appended to the
    current measured trajectory or replayed as object/robot state.
    """
    record = json.loads(path.read_text())
    if record.get('GT_inputs') is not False or record.get('support_scope') != 'prior_completed_D_before_current_physics':
        raise ValueError('REPLAY_ADOPTED_MODEL_MUST_BE_PRIOR_EE_ONLY_D')
    sources = {}
    for kind in ('estimate', 'memory'):
        source = Path(record['source_'+kind+'_path'])
        expected = record.get('source_'+kind+'_sha256')
        if not source.is_absolute() or not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise ValueError('REPLAY_ADOPTED_SOURCE_HASH_MISMATCH:'+kind)
        sources[kind] = json.loads(source.read_text())
    fit = record['fit']
    memory = sources['memory']
    if memory.get('GT_inputs') is not False or fit != sources['estimate']:
        raise ValueError('REPLAY_ADOPTED_MODEL_SOURCE_MISMATCH')
    matches = [row for row in memory['fit_history'] if row.get('accepted') and row.get('fit') == fit]
    if len(matches) != 1:
        raise ValueError('REPLAY_ADOPTED_MODEL_ACCEPTED_SUPPORT_AMBIGUOUS')
    count = int(matches[0]['observation_count'])
    expected_support = memory['supporting_observations'][:count]
    if len(expected_support) != count or record['supporting_observations'] != expected_support:
        raise ValueError('REPLAY_ADOPTED_MODEL_SUPPORT_MISMATCH')
    support, follow_sign = _accepted_support(fit, expected_support)
    initial = np.asarray(record['initial_ee'], dtype=float)
    activation = float(record['activation_time_s'])
    if initial.shape != (4, 4) or not np.isfinite(initial).all() or not np.allclose(initial[3], [0, 0, 0, 1]):
        raise ValueError('REPLAY_ADOPTED_INITIAL_EE_INVALID')
    if not np.allclose(initial[:3, :3] @ initial[:3, :3].T, np.eye(3), atol=2e-4) or not np.isclose(np.linalg.det(initial[:3, :3]), 1., atol=2e-4):
        raise ValueError('REPLAY_ADOPTED_INITIAL_EE_INVALID')
    if not np.isfinite(activation) or activation < 0:
        raise ValueError('REPLAY_ADOPTED_ACTIVATION_INVALID')
    return {'activation_time_s': activation, 'initial_ee': initial, 'fit': fit,
            'follow_sign': follow_sign, 'support_observation_count': len(support),
            'support_scope': record['support_scope'], 'adopted_sidecar': str(path),
            'source_estimate_sha256': record['source_estimate_sha256'],
            'source_memory_sha256': record['source_memory_sha256']}


def saved_safety_model(path):
    path = Path(path)
    adopted = path.parent / 'adopted_estimate.json'
    if adopted.is_file():
        # Invalid sidecars fail closed; never silently fall back to a different
        # short-probe model when D's adopted model cannot be authenticated.
        return _adopted_safety_model(adopted)
    memory = json.loads(path.read_text())
    if memory.get('GT_inputs') is not False:
        raise ValueError('REPLAY_SAFETY_MODEL_MUST_BE_EE_ONLY')
    fits = [row for row in memory['fit_history'] if row.get('accepted')]
    attempts = [row for row in memory['attempt_history']
                if row.get('result') == 'RELIABLE_REVOLUTE_ESTIMATE']
    if len(fits) != 1 or len(attempts) != 1:
        raise ValueError('REPLAY_REQUIRES_ONE_PRE_PHYSICS_ESTIMATE_NO_POST_HELDOUT_REFINEMENT')
    record = fits[0]
    count = int(record['observation_count'])
    support = memory['supporting_observations'][:count]
    if len(support) != count:
        raise ValueError('REPLAY_SAFETY_MODEL_SUPPORT_INCOMPLETE')
    fit = record['fit']
    support, follow_sign = _accepted_support(fit, support)
    return {'activation_time_s': float(attempts[0]['end_s']),
            'initial_ee': support[0], 'fit': fit,
            'follow_sign': follow_sign,
            'support_observation_count': count,
            'support_scope': 'accepted estimate support only; later physics/heldout observations excluded'}
