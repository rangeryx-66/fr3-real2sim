"""Independent FIT-only lack-of-fit audit, not an execution/estimation gate.

Separate the reported constrained/ridge estimate's residual from lack of fit
within its SAME global model family. No per-group parameters, truth tuning,
validation measurements or replacement coefficients enter the production model.
"""
import numpy as np
from prismatic_estimator_audit_v75 import matrix
VERSION='V75-same-family-penalty-separated-diagnostic-v2'


def diagnose(rows,model,replicates=256,seed=7503):
    if not rows:return dict(status='UNKNOWN_NO_QUALIFIED_FIT',execution_gate=False)
    X=matrix(rows,model);y=np.array([r['tau']for r in rows]);rank=np.linalg.matrix_rank(X)
    if rank<X.shape[1]:return dict(status='UNKNOWN_RANK_DEFICIENT',execution_gate=False)
    beta=np.asarray(model['parameters']);unregularized=np.linalg.lstsq(X,y,rcond=None)[0]
    original=y-X@beta;penalty_effect=X@(beta-unregularized)
    residual=original+penalty_effect  # y-X@unregularized, diagnostic ONLY
    numerical=64*np.finfo(float).eps*max(1.,float(np.max(abs(y))))
    velocity=np.array([r['qd']for r in rows]);acc=np.array([r['qdd']for r in rows])
    rng=np.random.default_rng(seed);groups=[]
    for label,mask in [('opening',velocity>0),('closing',velocity<0),
                       ('slower',abs(velocity)<=np.median(abs(velocity))),
                       ('faster',abs(velocity)>np.median(abs(velocity))),
                       ('accelerating',acc>=0),('decelerating',acc<0)]:
        blocks={}
        for i in np.flatnonzero(mask):
            r=rows[i];blocks.setdefault((r['segment'],int(r['t'])),[]).append(i)
        if len(blocks)<3:
            groups.append(dict(label=label,status='UNKNOWN_FEWER_THAN_THREE_BLOCKS'));continue
        means=np.array([residual[v].mean()for v in blocks.values()])
        raw_means=np.array([original[v].mean()for v in blocks.values()])
        boot=np.array([rng.choice(means,len(means),replace=True).mean()for _ in range(replicates)])
        low,high=np.quantile(boot,[.025,.975])
        groups.append(dict(label=label,time_blocks=len(means),
            production_residual_mean_N=float(raw_means.mean()),
            same_family_lack_of_fit_mean_N=float(means.mean()),
            empirical_interval_N=[float(low),float(high)],
            repeatable_same_family_structure=bool(low>numerical or high< -numerical)))
    structured=any(g.get('repeatable_same_family_structure')for g in groups)
    # Negative unconstrained coefficients are a physical/conditioning warning,
    # not coefficients to substitute into the nonnegative production model.
    unphysical=np.flatnonzero(unregularized< -numerical).tolist()
    status=('CHECK_SAME_FAMILY_RESIDUAL_STRUCTURE'if structured else
        'UNKNOWN_PHYSICAL_CONSTRAINT_OR_EXCITATION'if unphysical else
        'CONDITIONAL_ONLY_NO_LACK_OF_FIT_DETECTED')
    return dict(version=VERSION,status=status,groups=groups,global_diagnostic_parameters=unregularized.tolist(),
        unphysical_diagnostic_coefficients=unphysical,
        regularization_and_constraint_effect_RMS_N=float(np.sqrt(np.mean(penalty_effect**2))),
        production_residual_RMS_N=float(np.sqrt(np.mean(original**2))),
        same_family_lack_of_fit_RMS_N=float(np.sqrt(np.mean(residual**2))),
        production_coefficients_modified=False,validation_used=False,hidden_truth_used=False,
        single_global_family_not_groupwise_refits=True,execution_gate=False,
        physical_materiality='UNKNOWN_WITHOUT_INDEPENDENT_MEASUREMENT_ERROR_MODEL',
        calibrated_sensor_accuracy_or_success_probability=False,
        limitations=['No detected lack of fit does not prove parameter accuracy',
            'Finite-group empirical intervals omit sensor/contact systematic errors',
            'Only qualified sliding support is tested; static friction is not identified'])
