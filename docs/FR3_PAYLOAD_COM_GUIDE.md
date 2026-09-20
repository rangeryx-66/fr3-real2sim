# FR3 PayloadID 质心估计复现与排错手册

本文面向接手 `/data1/home/rangeryx/fr3_moveit_grasp` 的开发者或另一个 agent。目标是用当前已经验证的数据链估计 payload 的 **mass + center of mass (COM)**，并避免重新出现几十毫米的 COM 偏差。

> **先看结论**
>
> 这套代码没有靠“调质心”得到好结果。最终有效的是修正完整数据链：
>
> 1. PhysX hand-COM Jacobian 必须平移到 `fr3_hand_tcp`；
> 2. quasi-static gate 必须使用 `q` finite difference，不能使用当前语义异常的 reported `dq`；
> 3. empty/payload 的实际 `q` 不完全相同时，必须用 FR3 robot-only Drake model 补偿；
> 4. empty baseline 必须匹配 payload 的**实际夹爪开度**；
> 5. 必须剔除 torque transient、contact loss 和 payload relative slip；
> 6. COM 只有通过 observability 和 uncertainty gate 才能写入 USD，否则使用 geometry fallback。
>
> 若 COM 偏差稳定接近 **23.9–24 mm**，首先检查 Jacobian 原点。若误差随物体和姿态变化到几十毫米，首先检查是否绕过了 Drake `q` mismatch compensation、实际 opening 匹配或抓持稳定门禁。

本文只覆盖 mass + COM。当前 inertia 仍使用 reconstruction mesh 的 geometry fallback，并按可信 mass 缩放。

## 1. 当前验证结果和生产策略

最终跨物体验证结果：

| Object | Valid poses | Mass error | COM error | COM σ | Production decision |
|---|---:|---:|---:|---:|---|
| mustard | 8 | 0.119% | **1.02 mm** | 2.85 mm | `IDENTIFIED_Q_COMPENSATED` |
| soup | 8 | 0.099% | **0.86 mm** | 4.23 mm | `IDENTIFIED_Q_COMPENSATED` |
| sugar | 8 | 0.072% | **2.55 mm** | 1.20 mm | `IDENTIFIED_Q_COMPENSATED` |
| banana | 7 | 0.828% | 5.97 mm | **18.58 mm** | reject COM; geometry fallback |
| mug | 5 | 0.360% | 17.44 mm | 14.55 mm | reject COM; geometry fallback |
| raisin | 7 | 1.293% | 33.30 mm | 12.84 mm | reject COM; geometry fallback |

“拟合出一个 COM 数字”不代表它可以写入 USD。例如 banana 的误差小于 10 mm，但估计不确定度为 18.58 mm，所以生产逻辑仍拒绝这个 COM。

生产输出规则：

```text
mass_source = IDENTIFIED                       # 仅当 mass confidence 通过
com_source = IDENTIFIED_Q_COMPENSATED          # 仅当完整 COM confidence 通过
inertia_source = GEOMETRY_FALLBACK
```

若 COM confidence 失败：

```text
mass_source = IDENTIFIED                       # mass 可独立通过
com_source = GEOMETRY_FALLBACK
inertia_source = GEOMETRY_FALLBACK
```

禁止静默使用 GT mass、GT COM 或 GT inertia。

## 2. 不要用错入口

最终生产分析路径是：

```text
payload/empty capture
  -> pose-id pairing
  -> qFD static gate
  -> TCP Jacobian origin correction
  -> actual-opening FR3 Drake q compensation
  -> official filtering
  -> robust mass fit
  -> fixed-mass first-moment/COM fit
  -> confidence gate
```

核心文件：

| File | Purpose |
|---|---|
| `real2sim/payload_jacobian.py` | 把 Jacobian 从 hand COM 原点平移到 `fr3_hand_tcp` |
| `calibration/quasistatic_velocity_gate.py` | 从 `q(t)` finite difference 得到 gate 使用的实际 configuration velocity |
| `calibration/strict_pair_protocol.py` | settled window、opening、slip 和 pair quality 检查 |
| `calibration/compare_q_baselines.py` | raw subtraction、Drake q compensation 和诊断 baseline 对比 |
| `calibration/evaluate_cross_object.py` | 当前跨物体生产分析入口 |
| `real2sim/payload_id_official_mass_com.py` | Scalable Real2Sim filtering 和 mass+COM-only optimization |
| `real2sim/payload_skill_v2.py` | 静态 orientation/pose 设计 |
| `calibration/launch_payload_fresh_gate.py` | 使用 current-window stability gate 的采集入口 |

### 2.1 最常见的错误入口

下面的历史 convenience 入口可以单独验证官方 filter 和 mass+COM estimator：

```bash
python -m real2sim.payload_id_official_mass_com \
  --run <run-dir> \
  --official-root "$OFFICIAL_PAYLOAD_ROOT" \
  --output <result.json>
```

但它**不等价于最终跨物体生产路径**。最终 per-pose Drake `q` mismatch compensation 在 `calibration/compare_q_baselines.py` / `calibration/evaluate_cross_object.py` 中完成。

如果另一个对话只调用这个历史入口，很容易重新得到厘米级 COM。生产结果必须读取：

```python
result["methods"]["drake"]["fit"]
```

不要读取 `raw` 作为生产结果，也不要根据 GT 误差在 `raw`、`drake` 和 local-empty 方法之间事后挑最好值。

## 3. 环境

服务器项目：

```bash
cd /data1/home/rangeryx/fr3_moveit_grasp

export ISAAC_PY=/data1/home/rangeryx/isaaclab-arena/.venv/bin/python
export OFFICIAL_ID_PY=/data1/home/rangeryx/official_payload_env/bin/python
export OFFICIAL_SITE=/data1/home/rangeryx/isaaclab-arena/.venv/lib/python3.12/site-packages
export OUT=/data1/home/rangeryx/fr3_moveit_grasp/results/my_com_run

mkdir -p "$OUT"
```

若确实需要下载依赖，先按服务器约定：

```bash
source ./proxyon.sh
```

不要为复现 COM 修改抓取器、force/friction、静态姿态集合、estimator 或门限。

## 4. 正确的端到端运行顺序

以下以 `soup`、seed 1031 为例。

### 4.1 采集 payload

```bash
$ISAAC_PY -u calibration/launch_payload_fresh_gate.py \
  --target soup \
  --seed 1031 \
  --mode ANYGRASP \
  --gpu 3 \
  --port 8311 \
  --domain 131 \
  --payload-v2 \
  --mass-com-only \
  --skip-scan \
  --calibration-force 60 \
  --calibration-mu 1 \
  --output "$OUT/soup/payload" \
  --capture-payload
```

这里的 60 N、μ=1 是 PayloadID 诊断采集配置，用于减少 excitation 中的滑移。它不是普通抓取 benchmark 的默认配置，也不能替代 contact/slip guard。

完成后检查：

```text
$OUT/soup/payload/capture.json
$OUT/soup/payload/excitation_center.json
```

要求：

- `accepted` 至少 4 个，建议 7–8 个；
- 每个 accepted record 包含 `pose_id`、`path`、`opening_mean_m`；
- NPZ 中有实际 `q`、torque、timestamp、`actual_opening_m`、`T_TCP_object`、contact/guard 字段；
- static pose 覆盖正负 roll/pitch 和若干组合，不是主要绕 yaw。

若少于 4 个有效姿态，停止拟合，先查看 capture rejection reason。不要为了得到结果放宽 qFD、torque-settled 或 slip gate。

### 4.2 从 payload 读取实际夹爪 opening

不能直接使用 command opening。用 accepted window 的实际 opening：

```bash
OPENING_MM=$($ISAAC_PY -c '
import json, sys
d = json.load(open(sys.argv[1]))
a = d["accepted"]
assert len(a) >= 4, f"only {len(a)} accepted poses"
print(1000.0 * sum(x["opening_mean_m"] for x in a) / len(a))
' "$OUT/soup/payload/capture.json")

echo "actual opening: ${OPENING_MM} mm"
```

### 4.3 采集 matched-opening empty baseline

empty 必须复用 payload 的 pose center、pose set、seed 和实际 opening：

```bash
CROSS_PAYLOAD_SOURCE="$OUT/soup/payload" \
$ISAAC_PY -u calibration/launch_payload_fresh_gate.py \
  --target soup \
  --seed 1031 \
  --mode ANYGRASP \
  --gpu 3 \
  --port 8311 \
  --domain 131 \
  --payload-v2 \
  --mass-com-only \
  --skip-scan \
  --calibration-force 60 \
  --calibration-mu 1 \
  --output "$OUT/soup/empty" \
  --baseline-only \
  --center-q "$OUT/soup/payload/excitation_center.json" \
  --gripper-opening-mm "$OPENING_MM"
```

检查：

- empty 和 payload 至少有 4 个相同 `pose_id`；
- opening mismatch `< 0.0002 m`；
- empty/payload 两边 qFD `< 0.002 rad/s`；
- torque 已进入 settled window；
- payload `T_TCP_object` 无明显滑移。

实际 `q_empty` 与 `q_loaded` 相差 1–5 mrad 时，不要整组硬丢弃；保留实际 q，后续做 robot-only torque compensation。

### 4.4 按 pose ID 配对

empty 与 payload 是两个独立进程，timestamp 原点无关。只要有 `static_pose_id` / `pose_id`，就必须按 pose ID 配对。

`evaluation_cases.json` 的逻辑结构：

```json
[
  {
    "name": "soup",
    "payload_dir": "/data1/home/rangeryx/fr3_moveit_grasp/results/my_com_run/soup/payload",
    "pairs": [
      {
        "pose_id": 0,
        "empty_path": "/absolute/path/to/empty_pose00.npz",
        "payload_path": "/absolute/path/to/payload_pose00.npz"
      },
      {
        "pose_id": 1,
        "empty_path": "/absolute/path/to/empty_pose01.npz",
        "payload_path": "/absolute/path/to/payload_pose01.npz"
      }
    ]
  }
]
```

优先复用 `calibration/run_payload_fresh_gate_validation.py` 中从两个 `capture.json` 构造 pair 的实现，不要重新按时间插值猜配对。

### 4.5 运行最终 q-compensated 分析

```bash
export CROSS_OUTPUT="$OUT/analysis"
export PYTHONPATH="$OFFICIAL_SITE"
export OPENBLAS_NUM_THREADS=1

$OFFICIAL_ID_PY calibration/evaluate_cross_object.py
```

脚本调用 `calibration/compare_q_baselines.py`，生成每个物体结果和 `summary.json`。

生产读取：

```python
import json

with open("results/fr3_q_compensation/soup.json") as f:
    result = json.load(f)

fit = result["methods"]["drake"]["fit"]
mass_kg = fit["mass_kg"]
com_tcp_m = fit["center_of_mass_m"]
```

`raw` 只用于证明 q compensation 的收益。`empty_affine_exploratory` / local empty regression 只用于定位 Drake robot model bias，不进入默认生产结果。

## 5. 三个最关键的修复

### 5.1 Jacobian 原点：23.9 mm 系统误差

文件：`real2sim/payload_jacobian.py`

入口：

```python
from real2sim.payload_jacobian import tcp_jacobian_record

corrected_record = tcp_jacobian_record(record)
```

当前 convention 必须是：

```text
TCP_ORIGIN_BASE_AXES_V2
```

旧数据把 PhysX hand COM 处的 translational Jacobian 当作 `fr3_hand_tcp` Jacobian。FR3 hand URDF inertial origin 相对 link/TCP 的偏移是：

```text
[-0.0000376, 0.0119128, 0.0207260] m
norm ~= 23.9 mm
```

修复步骤：

1. 从 `config/fr3.urdf` 读取 `fr3_hand/inertial/origin`；
2. 使用 `T_B_hand` 把 offset 旋转到 base frame；
3. 将 translational Jacobian 从 hand COM 原点平移到 TCP 原点；
4. rotational Jacobian 保持一致；
5. estimator 输出 COM 的 frame 明确标为 `fr3_hand_tcp`。

GT torque diagnostic-only 路径在修复前后：

| Object | Before | After |
|---|---:|---:|
| soup | 23.9042 mm | 0.0045 mm |
| mug | 23.9047 mm | 0.0013 mm |
| banana | 23.9032 mm | 0.0076 mm |
| mustard | 23.9531 mm | 0.2331 mm |

因此：

- 误差固定在约 24 mm：大概率仍在用 legacy Jacobian；
- 误差约 48 mm：可能 correction 被重复应用；
- legacy record 缺 `T_B_hand` 时应报 `JACOBIAN_ORIGIN_UNKNOWN`，不能猜测原点继续跑。

### 5.2 reported dq 不是当前 gate 应使用的实际速度

文件：`calibration/quasistatic_velocity_gate.py`

正确入口：

```python
from calibration.quasistatic_velocity_gate import configuration_velocity

dq_gate = configuration_velocity(t, q)
```

审计结果：

- PhysX articulation reported `dq`: `0.038–0.047 rad/s`；
- 同一窗口的 `q` finite difference: 约 `4.5e-5 rad/s`；
- controller target velocity: 0；
- TCP/FK 变化和 torque drift 都表明机器人实际静止。

旧 gate 使用 reported dq 时 3 个姿态共 `0/18` 窗口通过；改用 q finite difference 后 `18/18` 通过。

当前规定：

```text
quasi-static gate source = finite difference of timestamped q samples
threshold = 0.002 rad/s
```

reported dq 仍保存用于审计，但不参与 quasi-static gate。不要通过把 threshold 放宽到 0.05 rad/s 来掩盖错误信号。

审计曲线位于：

```text
results/fr3_dq_semantics/validation/dq_timeseries.png
```

### 5.3 actual-q Drake compensation

empty/payload 的实际 q 不完全相同时，payload torque 必须使用：

```text
delta_tau_corrected =
    tau_loaded
  - tau_empty
  - [tau_robot(q_loaded) - tau_robot(q_empty)]
```

`tau_robot` 由 FR3 robot-only Drake plant 计算；plant 中 finger joint 固定到 payload 的实际 opening。该 correction 不使用 payload GT。

几 mrad q mismatch 的实际影响：

| Object | Raw → corrected residual RMS | J3 bias raw → corrected | J4 bias raw → corrected | Max q mismatch |
|---|---:|---:|---:|---:|
| mustard | 0.01683 → 0.00491 N·m | -0.00864 → -0.00084 | 0.04029 → 0.00201 | 2.083 mrad |
| soup | 0.01977 → 0.01112 N·m | -0.01751 → -0.00247 | 0.03475 → -0.00285 | 1.589 mrad |
| sugar | 0.01829 → 0.00859 N·m | 0.01264 → -0.00205 | 0.02301 → 0.00185 | 2.060 mrad |
| banana | 0.01501 → 0.01146 N·m | 0.00811 → 0.00370 | 0.02746 → -0.01357 | 1.571 mrad |
| mug | 0.01831 → 0.01160 N·m | 0.01954 → 0.00690 | 0.02092 → -0.00227 | 1.418 mrad |

早期诊断中：

- banana raw COM error `103.84 mm`，Drake corrected `3.45 mm`；
- mug raw `6.82 mm`，Drake corrected `5.22 mm`；
- local empty interpolation 曾把 mug 降到 `1.79 mm`，但 banana 反而为 `28.09 mm`。

因此 local empty interpolation 只保留为诊断，不允许按物体或 GT 误差挑选为生产方法。

## 6. Static window 与抓持稳定性

当前 `strict_pair_protocol.py` 的关键策略：

```python
q_pair_max_rad = 0.0015
q_ref_max_rad = 0.003
dq_max_rad_s = 0.002
opening_max_m = 0.0002
relative_translation_max_m = 0.003
relative_rotation_max_rad = radians(5)
settled_window_s = 0.5
min_samples = 96
attempts_per_pose = 3
min_clean_poses = 4
torque_half_window_max_Nm = 0.05
```

后续 q-compensation 诊断不再因为 1–5 mrad q position mismatch 把数据全部丢弃，而是做 robot-only correction。但是以下 gate 不能放宽：

- q finite-difference `< 0.002 rad/s`；
- torque settled；
- actual opening matched；
- continuous valid grasp/contact；
- `T_TCP_object` 无明显 relative slip；
- timestamp 单调且样本完整。

payload capture 开始条件是当前 `FREE_SPACE_STABLE`。历史版本按“从 close 开始累计的位姿变化”判断，会把已经完成自然 settling、当前实际稳定的物体继续拒绝。当前 fresh gate 只看最近约 0.8 s：

- 双指接触持续；
- 物体已离桌；
- 最近窗口 relative drift 不超过 3 mm / 5°；
- velocity threshold 持续满足约 300 ms。

自然 settling 可以发生，但采样窗口本身必须稳定。

## 7. Mass + COM estimator

当前 fit 分两阶段：

1. 对 `[m, h_x, h_y, h_z]` 做 robust Huber fit，得到 mass；
2. 固定 mass，只拟合 first moment `h`；
3. `COM = h / m`；
4. 不同时优化 rotational inertia。

复用的 Scalable Real2Sim 官方组件：

```text
robot_payload_id.eric_id.drake_torch_dynamics.MassAndComInertialParameter
robot_payload_id.utils.filtering.process_joint_data
robot_payload_id.utils.filtering.filter_time_series_data
```

官方上游记录 commit：

```text
c52e31cf26c83b33aee5e56f805e1d4d710fd549
```

滤波必须保持与当前官方适配一致：torque zero-phase filter，order 12，cutoff 1.6 Hz。不要在另一个对话里临时换滤波器或对 velocity 裸差分后参与动态 fit。

## 8. COM frame 与写入 USD

fit 输出的 `center_of_mass_m` 位于：

```text
fr3_hand_tcp frame
```

它不是 reconstruction mesh frame，也不是 object canonical frame。

若需要写入 object/USD local frame，使用采样时实测的 `T_TCP_object` 做明确的刚体变换。不要在 regressor 内把 TCP frame 和 object frame 混合，也不要把 quaternion 顺序隐式转换。

特别检查：

- ROS / SciPy quaternion 常用 `xyzw`；
- Isaac 某些 API 返回 `wxyz`；
- 变换方向是 `T_TCP_object` 还是 `T_object_TCP`；
- translation 是否以 m 为单位；
- COM correction 是否已经应用过一次。

建议在 USD metadata 中同时写：

```json
{
  "mass_source": "IDENTIFIED",
  "com_source": "IDENTIFIED_Q_COMPENSATED",
  "com_estimation_frame": "fr3_hand_tcp",
  "com_uncertainty_m": [0.0, 0.0, 0.0],
  "inertia_source": "GEOMETRY_FALLBACK"
}
```

实际字段以当前 asset schema 为准，关键是来源和 frame 必须可审计。

## 9. Confidence gate

至少检查以下量，不能只看 condition number：

| Check | Current acceptance rule | Failure action |
|---|---|---|
| Valid poses | at least 4; preferably 7–8 | recapture, no fit |
| qFD | max `< 0.002 rad/s` | resettle/recapture pose |
| Torque settled | 0.5 s, ≥96 samples, half-window drift `< 0.05 N·m` | reject window |
| Opening mismatch | `< 0.2 mm` | recapture matched-opening empty |
| Relative slip | ≤3 mm and ≤5° in window; contact guard passes | reject window/capture |
| Regressor rank | `>= 4` | `LOW_OBSERVABILITY` |
| Condition | `<= 1e4`, also inspect `sigma_min` | `LOW_OBSERVABILITY` |
| Mass sigma | `<= 15%` of estimated mass | reject identified mass |
| Combined COM sigma | `<= 0.012 m` | geometry COM fallback |
| COM physical range | finite, norm `< 0.25 m` | reject fit |

`sigma_min`、rank、condition、residual RMS 和 parameter covariance 必须一起报告。uncertainty 比点估计更重要：banana 正是因为 sigma 过大而不能启用 measured COM。

## 10. “偏离几十 mm”排错树

### 症状 A：几乎固定偏差 23.9–24 mm

最可能原因：Jacobian 原点仍在 hand COM，或 TCP correction 没应用。

检查：

```text
output convention == TCP_ORIGIN_BASE_AXES_V2
record contains T_B_hand
tcp_jacobian_record() is called exactly once
```

用 evaluation-only GT payload torque 直接喂 estimator：

- 仍然约 24 mm：frame/regressor 路径仍错；
- 已恢复到亚毫米：estimator/frame 正确，继续查真实 torque baseline。

### 症状 B：GT torque 路径亚毫米，但真实 torque 为 10–100 mm

最可能原因：

- 没使用 Drake actual-q correction；
- empty/payload opening 不一致；
- 按 timestamp 而非 pose ID 错配；
- torque 尚未 settle；
- payload slip。

按顺序检查：

1. `methods.drake.fit` 是否真的被读取；
2. `q_empty`、`q_loaded` 和 `robot_q_correction` 是否写入每个 pose；
3. `opening_empty` 与 `opening_loaded` 差值；
4. pair 的 `pose_id` 是否相同；
5. qFD / torque drift / relative slip rejection reason。

### 症状 C：reported dq 约 0.04 rad/s，所有静态窗口失败

原因：使用了错误语义的 native dq。

修复：使用 `configuration_velocity(t, q)`；不要把 0.002 threshold 放宽到 0.05。

### 症状 D：raw 差，Drake 明显改善

原因：1–5 mrad actual-q mismatch 造成 robot gravity torque difference。

动作：固定使用 Drake corrected torque。不要硬拒绝所有 q mismatch，也不要只用 raw subtraction。

### 症状 E：点估计小于 10 mm，但 uncertainty 大

原因：pose/gravity-direction coverage 不足或物体抓持可观测性差。

动作：

- 确认正负 roll/pitch 和组合姿态；
- 不要主要绕 gravity axis/yaw；
- 检查 rank 和 `sigma_min`；
- 若仍失败，使用 geometry fallback，不要用 GT 调阈值。

### 症状 F：同物体不同开度出现系统漂移

原因：empty baseline opening 不匹配，或 Drake model 没把 finger joint 固定到实际 opening/2。

动作：从 payload accepted windows 读取实际 opening，重采 empty，并检查 `plant_at_opening()`。

### 症状 G：不同姿态估计互相矛盾

检查：

- `T_TCP_object` 是否在窗口或跨姿态发生 slip；
- contact guard 是否持续通过；
- torque 是否仍有 transient；
- quaternion 是 `xyzw` 还是 `wxyz`；
- 使用的是 `T_TCP_object` 还是其逆。

## 11. 我们已经踩过、不要再重复的坑

1. **把变量名当语义。** 名为 `dq` 不代表它就是物理实际 joint velocity。
2. **假设到达同一 `q_ref` 就能直接相减。** 实际 q 相差几 mrad 已足以污染 J3/J4。
3. **使用 command opening 代替 actual opening。** 必须从 accepted payload window 读取实际值。
4. **把 hand COM Jacobian 当 TCP Jacobian。** 这是固定约 24 mm 误差的根因。
5. **按 GT 误差选择 raw/Drake/local-empty 方法。** 这是测试集泄漏；生产方法固定为 Drake correction。
6. **同时拟合完整 inertia。** 低可观测 inertia 会污染 COM；当前 inertia 不参与 fit。
7. **只看 condition number。** 必须同时看 rank、`sigma_min`、residual 和 covariance。
8. **用累计历史 settling 位移拒绝当前稳定 payload。** 当前 gate 只看最近稳定窗口。
9. **按两个进程的相对 timestamp 配对。** 有 pose ID 时必须按 pose ID。
10. **通过放宽 qFD/contact/slip gate 保留数据。** q position mismatch 可以建模补偿，动态窗口和滑移不能修回来。
11. **看到 COM error 小就忽略 sigma。** banana 的失败说明 uncertainty gate 不能省略。
12. **把 GT 用进 correction 或 fallback。** GT 只能在 fit 完成后 evaluation。

## 12. 给另一个 agent 的最小执行清单

把下面这段作为交接约束：

```text
1. 确认当前代码有 TCP_ORIGIN_BASE_AXES_V2 和 q finite-difference gate。
2. 先跑 GT torque diagnostic-only；若 estimator 不能亚毫米恢复 COM，停止真实数据实验并修 frame/regressor。
3. payload capture 至少 4 个 accepted pose，建议 7–8；保存 actual opening。
4. empty 使用同 seed、同 excitation_center、同 pose set、同 actual opening。
5. empty/payload 按 pose_id 配对，不按进程 timestamp。
6. static gate 只用 qFD < 0.002 rad/s；native dq 仅保存审计。
7. torque 使用最后 0.5 s settled mean 和当前官方 filter。
8. 生产 subtraction 必须使用 actual-q、actual-opening Drake robot-only correction。
9. 两阶段 fit：先 mass，再 fixed-mass first moment/COM；不拟合 inertia。
10. 生产读取 methods.drake.fit；禁止按 GT 为每个物体挑方法。
11. rank/condition/sigma/residual/slip 任一不通过，COM 使用 geometry fallback。
12. 写 USD 时记录 mass/com/inertia source 和 COM frame。
```

## 13. 完成标准

可以批准 measured COM 写入 USD 的条件：

- GT torque diagnostic 路径保持亚毫米级；
- corrected torque residual 明显低于 raw subtraction；
- 至少 4 个、最好 7–8 个有效姿态；
- 多数稳定规则物体 COM error `< 5–10 mm`；
- COM uncertainty 与真实误差量级一致且通过 12 mm gate；
- mass 保持当前约 0.1–1.3% 误差水平；
- 所有参数有明确 source 和 frame；
- 没有 GT 进入 fitting、compensation、method selection 或 fallback。

若正确链路后仍普遍大于 10–15 mm，不要继续做 force/friction、excitation amplitude 或 threshold sweep。冻结为：

```text
mass = identified
COM = geometry fallback
inertia = geometry fallback
```

## 14. 证据文件

- `results/fr3_torque_rootcause/REPORT.md`：Jacobian/frame 根因与 GT torque diagnostic。
- `results/fr3_dq_semantics/REPORT.md`：dq 语义审计与空载静态验证。
- `results/fr3_q_compensation/REPORT.md`：raw、Drake 与 local empty baseline 对照。
- `results/fr3_payload_capture_recovery/FINAL_REPORT.md`：最终跨物体结果与 fallback 决策。
- `docs/VALIDATION.md`：公开 release 的精简验证摘要。

最后更新：2026-09-20。
