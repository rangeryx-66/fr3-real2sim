# Constructor-only native contact treatment

The opt-in engineering study leaves original ABC, actuators, collision checks,
load limits and estimator coefficients unchanged. The active reference pilot
runs the original source revision separately; it is not hot-patched.

The plant constructor may bind a registered native physics material to both
actual finger colliders and selected handle colliders, with verified bindings
and the same average combine mode. Alternatively, an explicit baseline setting
inherits original materials without claiming their effective coefficient.
The existing native rail law supplies dynamic friction; a registered static /
dynamic ratio changes only native friction properties while retaining viscous
friction. No task outcome, slip flag, object motion, joint drive or external
effort is scripted. Readbacks stay in private plant outputs.

For matched independent comparisons, geometry may come from one observation
acquired and frozen during this study BEFORE its first physical acquisition.
Reference, pixels and fresh unexecuted automatic-candidate metadata are hashed.
Each new instance checks initial object/station state, performs real wrist
sensing, preserves that sensing evidence and its measured processing cost,
then reuses only the declared common geometric input. The original automatic
family is recomputed with native IK/collision/closure admission; no saved grasp
transform or joint path is supplied. After contact, this initial-state input
becomes stale and recovery requires current-state observation.

This is a controlled manipulation comparison, not an unseen-object or fresh
vision benchmark. Formal policy inputs exclude plant files and readbacks.
Actual material/contact sensitivity and safe stop/exit still require simulator
qualification; mocked tests and constructor configuration are not physical
proof. No operational benefit has yet been established.

Validation: 12 policy/treatment tests; existing 17 coordinated-control and 101
V8.7 tests passed during integration. Frozen-source audit passed (606 physical,
71 revolute, 956 release entries). A short real reference probing episode is
in progress; no formal A/B/C result is reported here.
