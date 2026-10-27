# Low-preload physical slip and same-World recovery

Execution SHA `23c73e6e7b05893162f11865ea036a9f86935839`; analysis code
is unchanged from `eb6d6ae`. Initial plant commitment is
`02ab1eb782d8f312f366c26285a69b3703aba952a1166c075a20e040275bcebf`.
Closed raw-evidence seal:
`c2b4d765d8a85c946c3d1f7bfec019a73f609e76640c8ddc2dc137801e323eb6`.

At request 0.5 N the fingers established contact, with segment pad-report
medians 0.42048 / 0.58939 N. Several native 0.5 s windows showed TCP travel
0.43–0.45 mm while drawer travel was approximately 0.0016 mm, and derived
material-point tangential speed about 0.7 mm/s. Native transmitting effort
in those windows was about 0.24–0.25 N. This is actual contact/relative-motion
evidence, not a coefficient-to-failure rule. Drawer coordinate at action end
was 0.003329 mm; maximum during that failed attempt was approximately
0.007623 mm. It did not perform the requested 20 mm operation.

The original controller classified `PRISMATIC_STARTUP_KINEMATIC_TANGENTIAL_SLIP_EVIDENCE`,
ended the current action, released, retreated and parked without a hard latch.
The World remained alive. Fresh actual wrist observation and a newly checked
native plan preceded recovery at request 1 N. Both acquisitions used automatic
family `[0,0,0,0]`; geometry was reobserved rather than replayed, so this is
supporting physical sensitivity evidence, not a perfect single-variable trial.

| Actual segment | Measured end coordinate mm | Result |
|---|---:|---|
| 0.5 N FIT forward | 0.003329 | TRANSMISSION_FAILURE |
| 1 N recovery FIT forward | 20.80804 | TARGET_REACHED |
| 1 N recovery FIT reverse | 3.42965 | TARGET_REACHED |
| Original independent VALIDATION | 16.85483 | TARGET_REACHED |

Maximum recorded coordinate was 20.80826 mm, reverse travel approximately
17.3784 mm. There were two physical acquisitions and one same-World recovery.
The 0.5 N result remains unqualified for this controlled grasp; recovery at
1 N is not relabeled as 0.5 N success. There was no object reset or episode
stitching. Native raw hard-stop samples were zero. Final actual release,
clearance and parking were verified, followed by owned PID/GPU release.

Pose-derived contact-sampled material sliding paths during the failed segment
were approximately 1.963 / 1.949 mm; TCP/handle drift peaked at 1.836 mm.
These are separate quantities. Unsigned pose-derived paths include numerical
jitter and are not persistent contact-point tracks or proof of complete
grasp separation. Subsequent successful segments retain their own estimates;
no cross-method slip or smoothness advantage is claimed from these runs.

The frozen pre-validation model hash was
`83da628287f43b96247141e2c768e84cd2f7266161c69f0637929efedf1eb7ca`.
Its original independent predictor used held-out compensated wrist effort
and one initial measured state, without subsequent state correction.
Over the derivative-supported 3.8625 s interior (928 samples), inverse-effort
RMSE was 0.00005069 N, coordinate RMSE 0.08052 mm and velocity RMSE
0.08599 mm/s; prediction remained within FIT support. This does not establish
parameter GT accuracy or a free-running forecast without measured wrench.

Launcher-to-original-completion wall time was 3266.91 s, simulation time
274.6375 s and recorded Agent waiting 357.45 s. Continuous video covers
272.8 s at 640×480, 1364 frames. Video, raw native data, failed actions,
recovery observations, three curve artifacts and separate analysis hashes
are preserved privately. This establishes contact sensitivity and recovery,
not Updated Physics superiority; the registered A/B/C comparisons remain next.
