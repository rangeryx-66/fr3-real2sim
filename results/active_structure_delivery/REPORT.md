# 本轮结论

**已恢复短弧 discovery → bounded refinement，但尚未证明 fresh TEST 上的结构重建泛化。**

冻结的新集合为 6 个 PhysX-Mobility 资产 × 2 个配置，共 12 场景。代码/config/预算在 TEST 前冻结，旧暴露资产没有进入主分母。

- Deployment：4/12；fixed-base 失败中的 mobile recovery 为 0/8。
- 真实双侧抓持：3 个配置；抓持并获得 ≥5 mm 有效运动：2/4 reachable。
- Discovery：2/2 useful interaction 进入 PROVISIONAL；均来自 45134。
- 最终 refinement acceptance：0/2；端到端：0/12；至少一个/两个配置成功的资产均为 0/6。
- GT 事后类型正确率：2/2 已产生可辨识运动的轨迹；不代表最终结构验收成功。
- 7320、45621 原成功回归：2/2，通过约 5.62° / 5.58° 实际开门。

## 首个阻塞

1. 8 个配置未通过 fixed + SE(2) 部署；其中两台洗碗机在 setup settle 阶段已被动打开约 39–41°，没有用 GT reset 或增大阻力压住运动。
2. 47419：一个配置稳定双侧抓持失败；另一个配置成功抓持但四次 probe 累计 EE 运动只有约 0.257 mm，判为 UNOBSERVABLE。它在机器人运动前已被动回到闭门限位，不能将这段被动变化记为交互成果。
3. 45134：两个配置分别完成 28 / 8 个 refinement 小步；实际开门约 2.53° / 1.58° 后，原模型一致性误差约 1.005 / 1.003 mm 触发安全停机。对应接触面漂移仅约 0.106 / 0.018 mm，GT 事后真实相对位移约 0.814 / 1.004 mm，不能仅凭旧 `SLIP` 标签断言脱手。

首个配置的在线轴方向误差为约 0.20°，但轴线误差约 60 mm；第二个配置方向误差约 9.16°。最新在线估计没有通过独立最终验收，也没有展示一致的结构误差改善。当前证据显示短弧结构不确定性与接触顺应仍然混杂；停止继续调 fitter。

**最终 0.30 mm、接触/力、关节余量等门槛均未放宽。Held-out 结构预测为 NOT REACHED，没有用训练残差冒充测试 RMSE。**

运行命令、主表、事后轴/轴线诊断和图表见下文。连续视频在服务器原运行目录，交付目录另附代表性视频。grasp/contact/mobile baseline 文件未修改；实现提交为 `b339503`。

---

# Provisional discovery → bounded refinement: fresh TEST

SIM_TO_SIM real physical contact. Original PhysX-Mobility visuals and frozen approximate interaction proxy. No physics fitting.

Frozen 6 fresh assets × 2 configs. Excludes all previous 25 prepared IDs and the preceding 57-asset source pool, including every previous main asset.

| Metric | Numerator / denominator |
|---|---:|
| deployment_coverage | 4/12 |
| interaction_coverage | 2/4 |
| discovery_coverage | 2/2 |
| refinement_success | 0/2 |
| end_to_end_success | 0/12 |

Assets with >=1 success: 0/6; both configs: 0/6.

## Per asset

| Asset | Reachable | Useful motion | Provisional/accepted | Refined | End-to-end | Failures |
|---|---:|---:|---:|---:|---:|---|
| 12606 | 0/2 | 0/2 | 0 | 0 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |
| 48452 | 0/2 | 0/2 | 0 | 0 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |
| 47419 | 2/2 | 0/2 | 0 | 0 | 0/2 | BILATERAL_GRASP_FAIL;ARTICULATION_UNOBSERVABLE |
| 12614 | 0/2 | 0/2 | 0 | 0 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |
| 45134 | 2/2 | 2/2 | 2 | 0 | 0/2 | REFINEMENT_SAFETY_STOP;REFINEMENT_SAFETY_STOP |
| 45448 | 0/2 | 0/2 | 0 | 0 | 0/2 | DEPLOYMENT_UNREACHABLE;DEPLOYMENT_UNREACHABLE |

## Interpretation and boundaries

- PROVISIONAL grants only bounded information gathering; it never counts as accepted reconstruction.
- All final geometry thresholds and physical safety guards remain enabled. Candidate directions/estimates use measured EE and robot/contact signals only.
- Refinement has one fixed budget, 1 mm segments, incremental IK/collision checks and hypothesis consensus. No asset-specific offsets or retries.
- GT metrics are computed only after saved estimates and stopped physical execution. GT angles never drive online stopping.
- T0 is the dataset structure prior, near oracle in simulation. It is not a visual reconstruction claim.
- Geometric held-out prediction and autonomous full-start same-command replay are separate metrics; incomplete replay has no complete RMSE.
- Existing contact-plane/slip supervisor is simulator-based and not proof of complete slip observability on real hardware.
- 7130 remains an excluded diagnostic failure: the existing RGB-D module tracks after-grasp patches, not a validated settle-to-grasp relocalization/replanning API; no GT reset or special fix is introduced.

## Run

```bash
python scripts/run_active_structure_benchmark.py --config configs/active_structure.yaml
```

For replication, use a new output and deadline. The original run, method and TEST manifest are immutable. Videos are continuous under each native candidate folder.

<!-- POSTRUN_EVIDENCE -->

## Diagnostic DEV: actual bounded refinement (outside TEST denominator)

| Asset | Discovery | Segments | Extra EE rotation | Door opening, post-eval | Stop | Min margin |
|---|---|---:|---:|---:|---|---:|
| 45600 | PROVISIONAL_REVOLUTE | 52 | 4.347° | 5.106° | SUSTAINED_RELATIVE_SLIP (J5) | 0.15278 |
| 38516 | PROVISIONAL_REVOLUTE | 51 | 2.153° | 2.726° | LOW_JOINT_MARGIN (J5) | 0.05000 |
| 45403 | PROVISIONAL_REVOLUTE | 51 | 2.696° | 3.601° | LOW_JOINT_MARGIN (J5) | 0.04999 |

All three reached active refinement and repeated robust joint fitting. None reached the independent final validation/acceptance stage before a frozen safety stop. Door motion >=5° alone is not an accepted reconstruction or end-to-end success.

![DEV measured rotation and safety residual](dev_refinement_progress.png)

## Regression controls

- 7320: SUCCESS; actual opening 5.621°, true final relative slip 0.145 mm.
- 45621: SUCCESS; actual opening 5.585°, true final relative slip 0.501 mm.

## Fresh TEST: provisional to refinement evidence

| Episode | Steps | Door motion (post-eval) | First stop | Model error | Contact-plane drift | Final true relative slip | Final accepted |
|---|---:|---:|---|---:|---:|---:|---|
| test_45134_00 | 28 | 2.529° | SUSTAINED_RELATIVE_SLIP | 1.005 mm | 0.106 mm | 0.814 mm | False |
| test_45134_01 | 8 | 1.581° | SUSTAINED_RELATIVE_SLIP | 1.003 mm | 0.018 mm | 1.004 mm | False |

### Post-stop GT diagnostics (latest online estimate is not final T2)

| Episode | Discovery axis error | Latest online axis error | Discovery axis-line error | Latest online axis-line error |
|---|---:|---:|---:|---:|
| test_45134_00 | 0.136° | 0.200° | 53.769 mm | 59.711 mm |
| test_45134_01 | 9.159° | 9.159° | 82.600 mm | 82.600 mm |

`online_refinement_diagnostics.csv` separately records discovery versus latest online axis/axis-line error. The latest online fit is not an accepted T2 when independent validation was not completed.

## Asset and run integrity

![Original dataset asset preflight](fresh_asset_overview.png)

- All frozen code/config hashes still match; method and asset/config manifests precede TEST execution. No TEST-dependent tuning or replacement.
- `deployment_diagnostics.csv` includes post-stop passive startup motion and every mobile-search result. Passive door opening is not credited to robot interaction.
- Dataset visual geometry is retained. Contact uses the existing approximate interaction proxies; this is not a hardware fidelity claim.

## Prediction evidence

If no final model is accepted, held-out structural prediction is **NOT REACHED**. No training residual is substituted for held-out RMSE; no T2 superiority is claimed.

## Videos

- `dev/45600/contact_baseline.mp4`: continuous approach, real closure, provisional discovery and refinement ending at the model-consistency safety stop.
- `controls/7320/contact_baseline.mp4`: successful original regression; explicitly not an unseen TEST success.
- Preflight-only failures have actual render images, not fabricated execution videos.

## Interpretation

The premature discovery gate was removed without relaxing final acceptance. DEV failure has moved to J5 margin and the frozen model-consistency supervisor. Fresh TEST deployment coverage must be read independently: episodes that cannot reach a grasp say nothing about whether their articulation fitter would succeed. Do not tune the fitter further based on these outcomes.

## Candidate-level safety provenance

`candidate_attempts.csv` retains all 8 actual candidate executions, including 2 collision stops. A later selected candidate does not erase earlier unsafe-approach stops.
