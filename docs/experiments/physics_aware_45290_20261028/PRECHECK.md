# Offline precheck: measured limits and unresolved gates

Historical closed 45290 data are used here only to audit sensor and model
sensitivity. No prior successful grasp pose is imported into a new trial.

Production FIT has three independent regressors. With the original declared
FIT intervals and complete sliding-filter support, 17418 samples qualify;
normalized design condition is 12.96. The unfiltered effort residual absolute
95th percentile is 0.02867 N. Filtered residual is much smaller, but the
common SG filter and correlated samples do not establish calibrated physical
confidence intervals.

Replaying a 6 kg mass difference on the measured acceleration gives only
0.00274 N RMS and 0.00152 N absolute 95th-percentile response. The old long
near-steady segments are weak mass experiments. By contrast, +58.5 N s/m
damping gives 0.2162 N RMS and +0.45 N Coulomb friction gives 0.45 N response.
New FIT probes need repeated interior speed changes and acceleration excitation;
changing estimator coefficients or fitting VALIDATION data is not authorized.

The old native friction reporting API is available on the loaded samples.
Observed used tangential/normal ratios are not identified coefficients:
non-sliding contact implies a lower bound on available traction. Actual slip,
material bindings/combine modes and calibrated preload-to-normal response must
still be checked physically.

An explicitly **oracle-only theoretical** grid, assuming unvalidated normal
loads equal to requests, produces:

| Theory condition | Request N | Speed mm/s | Acceleration mm/s² |
|---|---:|---:|---:|
| nominal | 0.5 | 6 | 12 |
| increased damping | 2.5 | 4 | 12 |
| increased mass only | 0.5 | 6 | 12 |
| increased rail friction | 1.0 | 6 | 12 |
| reduced contact friction | 2.0 | 6 | 12 |
| combined change | 2.5 | 2 | 4 |

These are policy-sensitivity calculations, not measured G1 loads, successful
actions, real slip predictions, or completed A/B/C comparisons. The mass-only
case has changed predicted force but identical control outputs. Several other
conditions change preload while preserving the fastest motion: if higher
preload has no measured cost, that alone cannot establish improved efficiency.
No invented squeeze/damage penalty is used to manufacture an advantage.

Next gate: finite native contact/preload/braking qualification and a fresh
observed reference, before freezing the changed-condition short comparisons.
M0/M1 probing can be performed in dedicated identification episodes; all
held-out manipulation episodes then start independently from the same state.
Only measured models/calibration transfer, never progress or physical contact
adaptation. Count full probing cost separately and in total, and distinguish
single-use from explicitly amortized model reuse. This prevents physical
contact practice from being silently credited as knowledge transfer.
