# Task-relevant structure acceptance

SIM_TO_SIM physical interaction; approximate interaction proxy. No physics fitting.

## SYSTEM (all frozen episodes)

| Stage | Count / all |
|---|---:|
| reachable | 8/12 |
| mobile_recovered | 0/12 |
| bilateral_grasp | 5/12 |
| valid_interaction | 3/12 |
| end_to_end_success | 0/12 |

## STRUCTURE (stable grasp and useful interaction)

| Metric | Count / valid interactions |
|---|---:|
| accepted_or_provisional_discovery | 3/3 |
| entered_refinement | 3/3 |
| heldout_validation_reached | 3/3 |
| operational_structure | 0/3 |
| high_fidelity_structure | 1/3 |
| heldout_improved | 2/3 |
| end_to_end_success | 0/3 |

## Safety and interpretation

Physical contact-plane drift remains a hard safety condition. Model prediction mismatch triggers pause/refit/step reduction. The contact-plane signal is not complete real-world slip observability.
HIGH_FIDELITY keeps the original 0.30 mm criterion. OPERATIONAL additionally requires an excluded action predicted reliably and a safe estimated-model continuation. A predictive acceptance alone is not manipulation success.
Held-out structure metrics use measured EE rotation as phase, a frozen axis/axis line, and only the first held-out pose as anchor. They are conditional geometric prediction, not autonomous physics prediction.

## Per asset

| Asset | Operational | High fidelity | Manipulation |
|---|---:|---:|---:|
| 47601 | 0 | 0 | 0/2 |
| 45385 | 0 | 1 | 0/2 |
| 47388 | 0 | 0 | 0/2 |
| 45916 | 0 | 0 | 0/2 |
| 45671 | 0 | 0 | 0/2 |
| 45922 | 0 | 0 | 0/2 |

## Failures

- INTERACTION_OR_REACHABILITY: UNOBSERVABLE: 2
- PHYSICAL_SAFETY: DANGEROUS_LOADED_CONTACT:gripper_link2:/World/articulated_pose/asset/Geometry/l_world/l_0/abstract_0_1/l_1/tn__l_1_original36_aL/tn__l_1_original36_aL: 1
- STRUCTURE_ACCEPTANCE: OPERATIONAL_PREDICTION_REJECTED: 3
- DEPLOYMENT: NO_MOBILE_RECOVERY: 4
- INTERACTION_OR_REACHABILITY: BILATERAL_HOLD_NOT_ESTABLISHED: 2

## DEV qualification (excluded from TEST denominator)

| Asset | Stop/result | Independent validation | Soft refits | Operational | High fidelity |
|---|---|---:|---:|---:|---:|
| 45600 | SUCCESS | True | 1 | True | False |
| 38516 | LOW_JOINT_MARGIN | False | 0 | False | False |
| 45403 | LOW_JOINT_MARGIN | False | 0 | False | False |
| 45134 | FINAL_TRUE_RELATIVE_SLIP | True | 2 | False | False |

## Reproduce

```bash
python scripts/run_operational_structure_benchmark.py --config configs/operational_structure.yaml
```

Use a new output and deadline for another run; frozen evidence is immutable.
