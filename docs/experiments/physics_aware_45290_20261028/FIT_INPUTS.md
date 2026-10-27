# FIT-only inputs and limits of the measured model

`measured_input.from_episode` reads original estimator output, identification
measurement settings and recorded telemetry. It never reads plant configuration,
authored physics or parameter readbacks. It uses original predeclared FIT /
transmitting-prefix eligibility and the original prismatic full-window sliding
qualification. VALIDATION samples cannot change model or empirical uncertainty.

This study predeclares stationary rail laws. Every model-based method uses the
original worker's existing constant candidate; the main selected model remains
reported independently. No estimator math is changed. The demand model combines
original conditional covariance with the empirical absolute 99th-percentile
unfiltered FIT residual. This is a conservative empirical margin, not a calibrated
total physical confidence interval or a guarantee outside the sampled range.

At least two eligible trajectories, actual bidirectional velocities, rank three
and original conditional parameter support are needed to use that candidate.
Otherwise normal reactive control continues. No new episode or safety-stop
authority is created. The interrupted first pilot has just one eligible FIT
trajectory and is correctly rejected as a usable M0 despite its plausible fit.
It has no held-out validation and cannot support a parameter-accuracy claim.

The offline prior-data audit was rerun using the exact same original declared
intervals and sliding qualification (17418 eligible samples, versus the initial
17690 samples before that qualification). The corrected sensitivity numbers are
in PRECHECK.md. Both audit artifacts are retained. The finding is unchanged:
ordinary low-speed near-steady motion weakly excites mass; a small filtered
residual alone cannot establish physical mass accuracy.

Nineteen policy/plant/command/FIT tests pass, including contamination of held-out
forces without any change to the selected control inputs. Native physical
qualification and A/B/C results are still pending.
