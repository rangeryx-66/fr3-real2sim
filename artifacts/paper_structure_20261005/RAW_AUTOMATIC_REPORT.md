# Internal structure-update comparison

Diagnostic cohort previously exposed: 6 assets x 2 configurations; no fresh unseen claim.

|Method|task success /12|actual >=5 deg /12|held-out reached|successful assets /6|
|---|---:|---:|---:|---:|
|B0|0/12|2/12|0|0/6|
|B1|0/12|2/12|3|0/6|
|B2|0/12|2/12|3|0/6|

Operation, model acceptance, and reconstruction errors are separate columns in MAIN_TABLE.csv.
Predictions use measured EE rotation or translation as phase: **conditional geometric prediction**, not autonomous dynamics or physics prediction.
Object-truth logs and relative transforms are evaluation-only; no object state is returned to controllers.
B0: frozen observed-direction compliant interaction, no global hinge update; local linear prediction is evaluated offline.
B1: one discovery estimate, frozen later. B2: existing online/refined update; rejected update falls back to discovery, never selected using final held-out test.
The historical 0.30 mm EE reconstruction metric is not object structure truth or a universal task gate.
No grasp/contact/proxy/base/physics/fitter/safety parameters were retuned.
All pre-contact failures stay in system denominators; paired post-grasp state matching is separately reported.

## Observation upper bound
See OBSERVATION_UPPER_BOUND.csv. Object-oracle fits are diagnostics, not method success.

## Paper evidence boundary
These comparisons are internal ablations, not official Act2See/Tac-Man reproduction.
>=5 degrees demonstrates small interaction, not full door opening.
Full relative drift is available only to the independent simulator evaluator; bilateral contact does not establish zero slip.
Final independent unseen tests, real-world friction measurement and a new perception stack are outside this round.
