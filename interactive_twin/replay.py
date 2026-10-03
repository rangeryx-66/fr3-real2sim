"""Restore the same pre-physics estimated-model safety gate during replay.

The external command sequence remains fixed. This model is neither simulator
truth nor a newly fitted held-out model; it was available in the reference before
P1 began. A timestamp prevents using that estimate earlier than it existed.
"""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def saved_safety_model(path):
    memory = json.loads(Path(path).read_text())
    if memory.get('GT_inputs') is not False:
        raise ValueError('REPLAY_SAFETY_MODEL_MUST_BE_EE_ONLY')
    fits = [row for row in memory['fit_history'] if row.get('accepted')]
    attempts = [row for row in memory['attempt_history']
                if row.get('result') == 'RELIABLE_REVOLUTE_ESTIMATE']
    if len(fits) != 1 or len(attempts) != 1:
        raise ValueError('REPLAY_REQUIRES_ONE_PRE_PHYSICS_ESTIMATE_NO_POST_HELDOUT_REFINEMENT')
    record = fits[0]
    count = int(record['observation_count'])
    support = np.asarray(memory['supporting_observations'][:count], dtype=float)
    if len(support) != count or count < 12:
        raise ValueError('REPLAY_SAFETY_MODEL_SUPPORT_INCOMPLETE')
    if float(np.max(np.linalg.norm(support[:, :3, 3]-support[0, :3, 3],axis=1))) < .005:
        raise ValueError('REPLAY_SAFETY_MODEL_EXCITATION_INSUFFICIENT')
    fit = record['fit']
    if fit.get('joint_type') != 'revolute' or fit.get('confidence', 0) <= .9:
        raise ValueError('REPLAY_SAFETY_MODEL_NOT_ACCEPTED_REVOLUTE')
    axis = np.asarray(fit['revolute']['axis'])
    rotation = Rotation.from_matrix(support[-1, :3, :3] @ support[0, :3, :3].T).as_rotvec()
    return {'activation_time_s': float(attempts[0]['end_s']),
            'initial_ee': support[0], 'fit': fit,
            'follow_sign': 1. if rotation @ axis >= 0. else -1.,
            'support_observation_count': count,
            'support_scope': 'accepted estimate support only; later physics/heldout observations excluded'}
