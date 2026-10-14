# Visual input reliability: bounded paired development round

Reference **9ae41fb**. User-registered physical order: **45600 → 45606 → 45671**.
One valid normal-closed-start attempt per case, sequentially. No physical retry,
manual pose correction, replacement asset or between-case tuning. The previous
four-case visual result (2/4) and Oracle/ten-case baselines remain preserved.

Before physics, inspect the saved four-case RGB-D, including 45638 regression:
identical-byte SAM3 inference repeated within a model and across three fresh
loads; raw query scores separated from original 0.25 confidence filtering and
later mask rejection. The four missing convs.3 tensors belong to the lowest
resolution branch removed by scalp=1; an all-zero intervention leaves active
raw logits unchanged. No checkpoint substitution or warning veto is introduced.

Shared visual-only update:

- If full-frame detection fails, attempt **one** observed-depth proposal/crop per
  episode, with the same prompt and 0.25 threshold. A vertical observed panel,
  measured robot exclusion and protruding-depth connected components generate
  the proposal; image elongation/area rank selects it, with half its long-axis
  extent as image context. No GT crops, instance masks or asset-specific prompt.
  The existing single camera re-observation remains available; no second crop
  after the one crop budget has been used. Count every inference/crop/view.
- Extend SAM seeds over connected observed depth within a shared local image
  context (0.1 long-axis extent + 4 px). Otsu on positive seed protrusion heights
  separates the front handle layer from mixed panel/back-layer pixels; retain
  the inherited 3 mm panel RANSAC tolerance. These operations estimate input
  geometry, not new physical execution acceptance conditions.
- Score three declared hidden depths, **0.75/1/1.25 × observed local width**.
  Keep observed front/lateral data fixed. Existing mean/lower-quartile weights
  aggregate performance across all hypotheses at a common pose/aperture and
  existing pose perturbations. Do not choose the best imagined backside or
  require every sample to pass. These priors are not calibrated confidence.
- Use frozen candidate families, contact/task weights, registered station and
  450 s planning budget. Fresh physical inputs regenerate grasp/approach/IK;
  archived/new offline grasp poses are never executable inputs.

Unchanged: JawCenteredClosure, native finite-effort G1 position actuation,
preload, Pink/opening reference, one-second hold, task timing, physical assets,
collisions, forces, joint margins and loaded-TCP protection. Original 30°
planning target remains; physical trial stops at approximately +3°. Outcome is
actual gain ≥2° after GRASP_VERIFIED, not commanded motion or 30° success.

Remain explicit known-model dependencies: registered initial station, known
hinge and dynamic body/articulation state, robot/camera calibration, original
scene collision checks, native collider IDs for reporting. This is calibrated
scene-camera RGB-D, not autonomous wrist-camera observation. GT handle surface **comparisons** run only in the offline evaluator; original native
scene geometry remains in existing collision checks/physics, never visual
generation/contact scores or GT pose correction.

Keep fresh RGB-D, continuous video, 240 Hz records and sensor metadata private.
Use sufficient free GPU capacity without exclusivity. Repair initialization
infrastructure errors without consuming physical attempts. Close each completed
actor after cooperative export. Publish code/compact results and stop after the
three registered cases, including any perception or physical failure.
