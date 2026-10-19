# 结构不确定性下的条件动力学校准

**SIM_TO_SIM_BLIND_SYSID；条件预测 A 和完整任务 B 分开验收。**

本轮不修改成功抓取、接触判据、proxy、base、控制器、安全上限或 mobile recovery。

## 主表

P4 使用预先冻结的双脉冲波形。主表为从相同抓持快照单独运行 P4 的结果；前序 P1–P3 自主演化后的 P4 结果另行保留。RMSE 是逐坐标 RMS。

|条件|模型|同初态 P4 RMSE mm|增量 RMSE mm|停驱动漂移 mm|漂移误差 mm|tau_c / b|完整任务 B|实测开门 °|
|---|---|---:|---:|---:|---:|---|---|---:|
|original|固定精化结构＋错误 prior|0.0905|0.0688|0.0333|0.0151|0.012 / 1.0|成功|5.565|
|original|结构候选＋相同 prior|0.0636|0.0472|0.0590|0.0408|0.012 / 1.0|成功|5.578|
|original|结构候选＋校准物理|0.0118|0.0104|0.0288|0.0106|0.0 / 0.0|成功|5.596|
|additional_nonzero|固定精化结构＋错误 prior|0.0545|0.0465|0.0333|0.0271|0.012 / 1.0|成功|5.569|
|additional_nonzero|结构候选＋相同 prior|0.0286|0.0256|0.0590|0.0528|0.012 / 1.0|成功|5.583|
|additional_nonzero|结构候选＋校准物理|0.0108|0.0077|0.0176|0.0114|0.004 / 0.0|成功|5.588|

## 可辨识性与限制

tau_c / b 是所测试网格中由训练与验证选出的有效模拟参数，不等于精确测量的真实铰链摩擦。
停驱动片段仍保持真实抓持及机器人阻尼/重力补偿，不是完全自由的门。接触冲量没有复制；每次仿真仅开始时初始化，随后由物理自主演化。

- original: UNIDENTIFIABLE_MODEL_RESPONSE_RESIDUAL; unresolved tested ranges = {'tau_c': [0.0, 0.012], 'b': [0.0, 1.0]}. 这些范围不是统计置信区间。
- additional_nonzero: UNIDENTIFIABLE_MODEL_RESPONSE_RESIDUAL; unresolved tested ranges = {'tau_c': [0.0, 0.012], 'b': [0.0, 1.0]}. 这些范围不是统计置信区间。

## 前序动作累计误差诊断

|条件|模型|P1–P3 后 P4 绝对 RMSE mm|同初态 P4 最终位移误差 mm|启动误差 s|速度 RMSE mm/s|
|---|---|---:|---:|---:|---:|
|original|固定精化结构＋错误 prior|0.1090|0.1407|0.2083|0.0302|
|original|结构候选＋相同 prior|0.0717|0.0769|0.0542|0.0301|
|original|结构候选＋校准物理|0.0273|0.0497|0.0583|0.0211|
|additional_nonzero|固定精化结构＋错误 prior|0.0695|0.0846|0.1625|0.0257|
|additional_nonzero|结构候选＋相同 prior|0.0342|0.0241|0.1000|0.0270|
|additional_nonzero|结构候选＋校准物理|0.0372|0.0280|0.0583|0.0197|

同初态 P4 是模型选择冻结后的初始化复核；没有利用它重新选择结构、物理参数或阈值。前序累计状态误差减小不能独自证明单动作动力学预测改善。

## 完整任务与失败

完整任务保持原始 home/warmup → approach → closure → manipulation。逐模型保留失败，未用 A 的初始化绕过 B。

- original / 固定精化结构＋错误 prior: REPLAY_COMPLETE; bilateral=True; min margin=0.0635 rad; final slip=0.1761 mm.
- original / 结构候选＋相同 prior: REPLAY_COMPLETE; bilateral=True; min margin=0.0635 rad; final slip=0.2101 mm.
- original / 结构候选＋校准物理: REPLAY_COMPLETE; bilateral=True; min margin=0.0635 rad; final slip=0.0940 mm.
- additional_nonzero / 固定精化结构＋错误 prior: REPLAY_COMPLETE; bilateral=True; min margin=0.0635 rad; final slip=0.1790 mm.
- additional_nonzero / 结构候选＋相同 prior: REPLAY_COMPLETE; bilateral=True; min margin=0.0635 rad; final slip=0.2020 mm.
- additional_nonzero / 结构候选＋校准物理: REPLAY_COMPLETE; bilateral=True; min margin=0.0635 rad; final slip=0.1160 mm.

### test_45621_00 保留的停止结果

{"stage": "transfer_structure_observability", "reason": "TRANSFER_STRUCTURE_UNOBSERVABLE", "detail": "'revolute'", "measured_opening_travel_m": 0.038709312808536615, "measured_reverse_travel_m": 0.0006894695321076671, "extra_trajectory_requested": false, "threshold_changed": false}

初始 COM / 惯量装配一致性：True，共 45 个候选检查。

数据：comparison.csv、summary.json、每个 episode 下的 selection_frozen.json、common_start_heldout/results.json、updated_twins/*/conditional_twins.json 与连续 contact_baseline.mp4。

单一复现命令见 docs/interactive_twin_conditional.md。
