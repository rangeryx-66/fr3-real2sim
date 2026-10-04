"""Discrete response-disagreement policy with strict held-out separation.

Reuses frozen scoring/noise/profile/adequacy criteria. A preferred simulator
predictor is kept distinct from uniquely identifiable friction parameters.
"""
import numpy as np
from interactive_twin.sysid import _signature
from interactive_twin_refinement.physics import score
from interactive_twin_observable.probes import action_travel


def action_score(reference, predicted, noise):
    if reference['split'] == 'heldout' or predicted['split'] == 'heldout':
        raise ValueError('HELDOUT_ACCESS_DENIED')
    d = score(reference, predicted, noise)
    r, p = d['post_drive_drift_reference'], d['post_drive_drift_prediction']
    drift = abs(r['net_drift_m']-p['net_drift_m']) if r and p else 0.
    n = len(d['noise_normalized_metrics'])
    d['selection_loss'] = (n*d['normalized_loss']+(drift/noise.position_m)**2)/(n+1)
    return d


def update(candidates, bank, observed, specs, noise, policy):
    if any(k not in specs for k in observed): raise ValueError('NON_TRAINING_ACTION_INPUT')
    rows = []; unused = []
    names = []
    for name, log in observed.items():
        if name == 'reverse' and action_travel(log, specs[name])['excursion_m'] < .001:
            unused.append({'name': name, 'reason': 'REVERSE_UNAVAILABLE_BELOW_UNCHANGED_1MM'}); continue
        names.append(name)
    train = [k for k in names if specs[k]['split'] == 'train']
    validation = [k for k in names if specs[k]['split'] == 'validation']
    for item in candidates:
        cid = item['candidate_id']; predictions = bank[cid]
        if any(k not in predictions or not predictions[k]['safe'] for k in names): continue
        scores = {k: action_score(observed[k], predictions[k]['log'], noise) for k in names}
        rows.append({**item, 'train_loss': float(np.mean([scores[k]['selection_loss'] for k in train])) if train else 0.,
                     'validation_loss': float(np.mean([scores[k]['selection_loss'] for k in validation])) if validation else None,
                     'actions': scores})
    if not rows: return {'rows': [], 'support': [], 'unused': unused, 'train': train, 'validation': validation}
    minimum = min(r['train_loss'] for r in rows)
    support = [r for r in rows if r['train_loss'] <= minimum+policy['profile_loss_delta']]
    decision_support = support
    if validation:
        floor = min(r['validation_loss'] for r in support)
        decision_support = [r for r in support if r['validation_loss'] <= floor+policy['profile_loss_delta']]
    return {'rows': rows, 'support': support, 'decision_support': decision_support,
            'unused': unused, 'train': train, 'validation': validation}


def choose_next(state, bank, observed, specs, noise):
    rows = state.get('decision_support', state['support']); options = []
    for name, spec in specs.items():
        if name in observed: continue
        cells = [bank[r['candidate_id']].get(name) for r in rows]
        safe = bool(cells) and all(c and c['safe'] for c in cells)
        record = {'name': name, 'safe_in_remaining_models': safe, 'eligible': safe}
        if not safe:
            record['reason'] = 'PREDICTED_SAFETY_OR_INCOMPLETE_ROLLOUT'; options.append(record); continue
        excursions = [action_travel(c['log'], spec)['excursion_m'] for c in cells]
        record['predicted_excursion_range_m'] = [min(excursions), max(excursions)]
        if name == 'reverse' and min(excursions) < .001:
            record.update(eligible=False, reason='REVERSE_NOT_RELIABLY_OBSERVABLE_AT_1MM'); options.append(record); continue
        vectors = np.array([_signature(c['log'], noise) for c in cells])
        disagreement = float(np.mean(np.var(vectors, axis=0)))
        record.update(prediction_variance_noise_units=disagreement,
                      information_score_per_second=float(np.log1p(disagreement)/spec['duration_s']),
                      reason='OBSERVABLE_MODEL_DISAGREEMENT')
        options.append(record)
    eligible = [o for o in options if o['eligible']]
    # Low-speed first is a frozen safe seed; every subsequent decision uses
    # only measured prior responses and prospective twin predictions.
    if not observed:
        selected = next((o for o in eligible if o['name'] == 'low_forward'), None)
    else:
        selected = max(eligible, key=lambda o: o['information_score_per_second'], default=None)
    return {'selected': selected['name'] if selected else None, 'options': options,
            'support_count': len(rows), 'reference_future_response_read': False}


def finish(state, config, original_count):
    rows = state['rows']; support = state['support']
    if not rows or not {'low_forward', 'high_forward'} <= set(state['train']) or not state['validation']:
        return {'status': 'UNIDENTIFIABLE_PROBE_SAFETY_OR_EXCITATION_BUDGET', 'methods': {},
                'feasible_parameter_set': [{k:r[k] for k in ('structure_id','tau_c','b')} for r in support]}
    def best(items):
        floor = min(r['train_loss'] for r in items)
        return min((r for r in items if r['train_loss'] <= floor+config['selection']['profile_loss_delta']), key=lambda r:r['validation_loss'])
    prior = [r for r in rows if all(r[k] == config['wrong_prior'][k] for k in ('tau_c', 'b'))]
    fixed = next((r for r in prior if r['structure_id'] == 'S0'), None)
    joint = best(rows); structural = best(prior) if prior else None
    adequate = joint['validation_loss']**.5 <= config['selection']['max_validation_rms_noise_units']
    # Response mismatch cannot manufacture a narrowly identified parameter set.
    feasible = support if adequate else rows
    pairs = sorted({(r['tau_c'], r['b']) for r in feasible})
    status = ('GRID_PREFERRED_NOT_CONTINUOUSLY_IDENTIFIED' if len(pairs)==1 else 'UNIDENTIFIABLE_PARAMETER_AMBIGUITY') if adequate else 'UNIDENTIFIABLE_MODEL_RESPONSE_RESIDUAL'
    methods = {'fixed_refined_wrong_prior': fixed, 'uncertain_structure_same_prior': structural,
               'uncertain_structure_calibrated': joint}
    return {'status': status, 'methods': {k:v for k,v in methods.items() if v is not None},
            'unavailable_methods': [k for k,v in methods.items() if v is None],
            'profile_support_count': len(support), 'initial_candidate_count': original_count,
            'feasible_set_count': len(feasible), 'validation_adequate': bool(adequate),
            'profile_support': [{k:r[k] for k in ('candidate_id','structure_id','tau_c','b','train_loss','validation_loss')} for r in support],
            'feasible_parameter_set': [{'tau_c':c, 'b':b} for c,b in pairs],
            'parameter_intervals': {k:[min(r[k] for r in feasible),max(r[k] for r in feasible)] for k in ('tau_c','b')},
            'parameter_semantics': 'effective simulator resistance; no calibrated torque scale; profile support is not a confidence interval',
            'test_read': False, 'ground_truth_read': False, 'unused': state['unused']}
