# Requested preload, actual normals and usable traction

`contact_calibration` uses native normal impulses and separate native friction
anchor reports from COMPLETE, transmitting FIT segments. Independent
VALIDATION never improves this calibration. Each request is associated with its
recorded acquisition identity. Neither requested preload nor a plant material
coefficient substitutes for measured normal load.

Stationary FORCE_HOLD loads and aperture are also reported separately. This
is loaded grasp contact, never an unloaded-motion permission. Opposing finger
normals can cancel at the wrist; six-axis wrist F/T is not a measurement of
their scalar pinch-load sum. These are native simulated contact measurements,
not hardware force feedback from the G1 position interface.

For multiple contacts on a finger, normal impulse MAGNITUDES are added before
computing a used-friction ratio. Taking the norm after summing opposing normals
could produce an arbitrarily inflated ratio. The original supervisor's pad
reports are retained separately. The axis-tangent normal budget accounts for
each measured normal direction. Normal load redistribution between fingers is
represented by joint summed-load statistics, not by adding unrelated marginal
quantiles.

The smaller finger-wise used-ratio 95th percentile is a demonstrated traction
lower bound; it is NOT an identified friction point estimate or upper bound.
It combines with measured request/normal calibration to predict an empirical
axis capacity. Transfer assumes the same G1/handle/contact geometry and stable
normal-load mapping. Held-out normals, slip and wrench test that assumption;
native supervision still owns every actual contact. Input commitments include
the raw contact-file hashes. No coefficient or request can declare a slip.

FIT startup wrist-effort windows plus the FIT residual also enter the demand
model as an observed startup bound. This preserves the distinction between
breakaway and sliding demand without fitting held-out data or pretending to
identify a precise static-friction coefficient.

A read-only diagnostic of the closed, failed qualification 03 has 2831 eligible
contact samples at request 1 N. The joint scalar normal sum's 5th percentile is
1.99418 N; original pad-report medians are 0.94908 / 1.06322 N. Used-traction
lower bound is 0.28769, giving a same-geometry predicted axis capacity of
0.57371 N. These measurements do not qualify the failed episode's exit or
establish any Physics-Aware advantage. Further preload settings require actual
native qualification; no high-preload damage or artificial cost is invented.
