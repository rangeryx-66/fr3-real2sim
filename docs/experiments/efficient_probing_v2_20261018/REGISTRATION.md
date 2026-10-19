# Efficient Agent-guided probing V2

Reference `1c98f9b`; one normal-closed-start research episode on **45936**. A separate short robot-only interface check is permitted before it; no door interaction or saved-grasp fixture is used in that check.

Fresh rigid-wrist RGB-D/SAM3, incremental observed geometry, unchanged native-G1 scoring/closure and Pink IK. Known hinge/state, registered station, native safety collisions, simulated wrist mount, ideal load cell and approximate actuators remain explicit dependencies. Reuse the existing constant-resistance scene configuration, fixed throughout the episode; its hidden dynamics are not inputs to the estimator or Agent decisions.

Agent inspects images/depth/cloud boards before selecting views and approves finite interaction actions. Up to four view adjustments, two handle acquisitions, 22 Agent operations, and 1,800 wall seconds. Aim toward 10–15° pulling then 2–5° closing; extend toward 20° only with actual feasible progress. No forced contact-mode sequence, saved camera/grasp replay, automatic valid-episode retry or safety relaxation.

The motion menu offers 6/10/12 mm/s reference speeds with bounded acceleration and optional smooth variation, all clipped below the unchanged phase-aware policy. Initial candidate actions are chosen after fresh observations; FIT versus VALIDATION labels are declared before execution. Complete held-out segments never enter fitting or derivative windows. Constant model: `tau = I_eff*qdd + b*qd + C*sign(qd)`, using compensated wrist F/T and measured motion only. Unsupported inertia/damping remain unknown.

Budget **1,000,000,000 bytes**. Single compressed numeric-array stream; native 240 Hz during physical interaction and transients, 10 Hz unloaded diagnostics. F/T, hinge state, sensor/tool poses and required compensation inputs retain float64; native robot q/dq and commands use float32. No raw contact dictionaries or duplicated processed streams. Wrist RGB-D only at actual checkpoints; losslessly compress checkpoint arrays once after execution and delete re-creatable intermediates. At most four seconds of low-resolution event clips, 12 MB clip cap. Space pressure prunes optional geometry visualizations, never essential numeric data. No raw archive or automatic backup copy; temporary local Agent inspection images are removed afterward.

After each completed interaction segment, an asynchronous measurement-only worker updates the constant model, conditional uncertainty, excitation and held-out forward prediction. It receives no object mass, inertia, friction or GT torque. Numeric estimates and safety feedback inform subsequent Agent choices only at verified unloaded boundaries.

Preserve all outcomes, finalize compact storage metrics, release/park where feasible and close owned processes. Publish code and concise derived results only; no raw video/RGB-D/telemetry upload. Stop after the registered research episode.
