# Native G1 CENTER aperture freeze: shared repair and bounded physical diagnosis

**Scope: NATIVE_G1_CLOSURE_DIAGNOSTIC / ORACLE_HANDLE / KNOWN_MODEL_DEVELOPMENT.** Reference `bb3a443`. One new normally closed 45717 diagnostic was executed; no opening trial, no 45638 physical regression, and no further physical attempt. The frozen ten-door benchmark, its scores, and its executable inputs remain unchanged.

**Finding:** premature CENTER closure freeze is real. The minimal shared repair is implemented and verified offline. However, additional closure at the original fixed approach TCP did **not** establish bilateral contact: the contacting left finger loaded up to 2.0274 N while the right finger remained at zero. The unchanged sustained-force protection stopped the diagnostic. This supports **B: contact-geometry failure at this fixed pose**, rather than safe closure continuation (A). The repair has **not** been physically validated or promoted into the frozen skill.

## Actual loaded closure path and the 32 mm plateau

The native execution generator loads the frozen `JawCenteredClosure` from `results/45415_closure_verification_20261011/provenance/active_closure.py`, SHA256 `b522bcef78a1191a4a2a5c756c34d11d7df2e22ddaa5f782be61343947a1704c`. Recorded runtime provenance confirms this path and update implementation; inspection was not based only on an intended filename.

Unilateral contact (`max pad >= 0.05 N`, `min pad < 0.05 N`) switches SLOW_CLOSE to CENTER. Only SLOW_CLOSE schedules an aperture decrement. CENTER applies bounded TCP centering and `min(previous command, measured aperture)`, which can passively ratchet the command downward but does not continue timed closing. In the original 45717 episode, the controller stayed in CENTER for 52.05 s after 47.95 s of SLOW_CLOSE. Its 100 s closure budget expired without bilateral holding.

Original first contact was at 35.916 mm. The final command/measured aperture was 32.097/32.164 mm, only 0.067 mm tracking difference. Thus the plateau was principally the state-machine command freeze plus timeout, not a commanded 13.75 mm target that the actuator failed to reach. The static 13.75 mm score was never a safe physical closure target. Original bounded centering was permitted up to 2 mm.

The native G1 adapter converts aperture to the official single drive-joint position target and uses five physical coupled/mimic joints. Legacy LOW_PRELOAD/FORCE_HOLD effort requests become a finite-drive position preload; they are not PiPER-style direct effort hold. The actual native position drive, stiffness, damping, finite effort authority and coupling were retained.

## Minimal shared change and offline comparison

New version: `scripts/jaw_centered_closure_progressive.py`. Exactly one condition changes relative to the frozen source:

```diff
- if self.state=='SLOW_CLOSE': self.opening=max(0.,self.opening-self.p['slow_closure_m_s']*dt)
+ if self.state in ('SLOW_CLOSE','CENTER'): self.opening=max(0.,self.opening-self.p['slow_closure_m_s']*dt)
```

The existing 1 mm/s slow rate, non-reopening latch, TCP-centering direction/bounds, LOW_PRELOAD/FORCE_HOLD transitions and all protections remain unchanged. No frozen configuration was redirected to the new module. Grasp scoring, Pink, opening control, native actuator, object geometry and physics were not modified.

| Recorded-input replay | Updates | Old-command reproduction error | New position command | TCP centering targets |
|---|---:|---:|---|---|
| 45717 | 24,000 | 0 | Monotonic closing | Identical at every update |
| 45638 | 16,965 | 0 | Monotonic closing | Identical at every update |

For 45717, 10 s after first CENTER contact, the old/new commands are 32.097/25.912 mm. For 45638, the same recorded inputs still yield LOW_PRELOAD/FORCE_HOLD through the original logic. These are **counterfactual command replays** with old measured aperture/load inputs, not predictions of physical reachability or safe loading.

## One fixed-TCP physical diagnostic

The original automatically generated plan, station, normal closed initialization and approach were reused. From approach completion onward the arm command was held constant, with no TCP-centering compensation. After first unilateral contact and a 0.5 s observation hold, extra closure was commanded in 0.25 mm increments interpolated at **0.5 mm/s**, below the frozen slow rate. No jump to 13.75 mm was made. The original temporal force policy, collision, contact, joint, actuator and loaded-TCP protections remained active.

| Evidence | Measured result |
|---|---:|
| First unilateral contact: command / measured aperture | 35.915 / 35.913 mm |
| Final command / measured aperture | 34.398 / 35.407 mm |
| Minimum measured aperture | 35.402 mm |
| Maximum raw left / right pad load | 2.0274 / 0 N |
| Bilateral contact / FORCE_HOLD | Not established |
| Positive native contact pair | Left finger–registered handle `original-35` only |
| Arm command change after lock | 0 rad |
| Maximum measured TCP compliance: position / orientation | 0.424 mm / 0.0533° |
| Maximum handle-relative translation change | 0.386 mm |
| Maximum TCP speed after unilateral contact | 1.139 mm/s |
| Minimum arm joint margin | 0.6614 rad |
| Maximum native coupling error | 0.000111 rad |
| Final native drive position / target | 0.54181 / 0.55109 rad |
| Maximum incidental door angle | 0.00214°; no manipulation success claim |
| Simulation / wall time | 132.221 / 693.301 s |

A raw sample above 2 N first triggered the existing soft warning and paused further aperture advancement. The existing completed-window means then reached **2.00718, 2.01460 and 2.02223 N**, producing `HARD_FORCE_STOP_SUSTAINED` after three overload windows. This was **not** a single-sample cutoff. Right-pad load stayed zero throughout. Native records identify the **whole left-finger collider** against the registered handle. The contact point moved only about 0.24 mm and its reported world normal stayed approximately (0.006, 0.274, −0.962). Collider ordering is preserved in the private stream. The `allowed_pad_target` label validates the object target; it does not by itself establish that the contact lies inside a desirable fingertip pad patch. Consequently this result is opposing-contact failure evidence, not a certified pad-geometry diagnosis.

The drive remained far from its 0.85 rad travel bound, with small coupled-joint disagreement. A diagnostic PD-request/finite-cap ratio peaked at 2.85%; this is a calculated proxy, **not exact implicit solver effort or measured hardware torque**. There is no evidence establishing a native actuator/coupling integration defect (C). Contact opposing further motion is supported; whether a different centered pose can obtain bilateral support remains unresolved.

**Important comparison limitation:** the old benchmark reached approximately 32 mm after bounded TCP centering. This diagnostic intentionally held the unshifted approach TCP fixed, and therefore met overload near 35.4 mm. It did not test the new progressive-CENTER policy with centering enabled, and does not establish that every centered pose is unsuitable. It also does not prove which surface/collision approximation caused the missing opposing support. The observed fixed-pose result does not meet the user's prerequisite of demonstrated safe continued closure, so the conditional shared-policy physical test and positive-control test were not launched.

## Evidence integrity and next action

Both 240 Hz streams decoded completely: **31,733 observation records and 31,733 native physics records**, with index counts matching and zero chunk errors. Continuous video decoded successfully: **1280×960, 3,966 frames, 132.2 s**. Raw contacts retain points, normals, collider identities, impulses, drive/coupled states and TCP/body transforms. Placeholder pre-grasp slip fields were not used as evidence: relative displacement above was reconstructed from raw transforms. The actor exported telemetry and exited normally; process cleanup is indexed separately.

See `PRELAUNCH.json`, `SOURCE_INTEGRITY.json`, `OFFLINE_OLD_NEW_SUMMARY.json`, `FIXED_TCP_DIAGNOSTIC_SUMMARY.json`, `EVIDENCE_INDEX.json` and `PRIVATE_ARCHIVE.json` for executable/input hashes, metrics and private evidence locations. The physical runner SHA256 is `d6593c9d0d6dba0bbb4f392c3f12f5cbf8abacce0acfac8ca24be9e9446ddeb8`; generated executable SHA256 is `ab4972d9dabc3a278f17063f137b80ed4d946d7d40d2fce28d0f552e5cb441cc`. New shared closure SHA256: `17d15c31925921d5a0275edc51af3fb71d466e5d38f8ea77c9e4eafeef86f349`.

**Recommended next engineering action:** retain the one-line repair as an isolated, offline-verified version; investigate simultaneous opposing finger support and contact geometry at the actual centered pose before its physical promotion. Do not squeeze harder or increase force/effort. No planner rewrite or additional door experiment was performed in this round. Videos and raw telemetry remain private; only code, compact metrics and evidence indexes are published.
