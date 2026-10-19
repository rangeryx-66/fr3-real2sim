# Actual-observation resume — 2026-10-06

The first 7320 run completed release, mobile scanning, wrist-only reobservation,
fresh planning and actual regrasp. Six clean initial-state images were saved.
It stopped at discovery with `NoneType is not subscriptable`: the orchestration
had set a confidence forecast anchor while the articulation estimate was None.
Reset now leaves that anchor None until a model exists. Known-model monitoring,
fitter thresholds, contact control, physical safety and gains are unchanged.

Past images are persistent measurements. Closed-state reuse checks current
sensor RGB-D registration, asset identity, intrinsics and original image QA.
The 1 mm check applies ONLY to whether old observations can be reused; failure
falls back to fresh capture and does not stop or veto regrasp. No robot/object
state, attachment or contact impulse is restored. Reused files retain original
poses, source directories and RGB hashes, and are labelled not newly acquired.

An actual-data check recovered eight distinct initial-state views: six from
the current run and two from the preceding low-view run. The latter registered
with 0.197 mm RMSE and 0.008 mm pose displacement. Raw images are byte copies.
Camera novelty retains the 4 cm spatial bound and 8° orientation criterion;
a >=4 cm transverse stereo baseline is also valid without rotation. Same-ray
zoom duplicates remain rejected. This does not relax image or physical QA.

Azimuth-group ordering reduces needless side-to-side camera travel without
changing the candidate set. Physical approach/closure is reexecuted on restart;
only verified observations are reusable. Non-initial states are not restored.

Two additional input issues were verified against actual data/official code:

- SAM3 found no `cabinet with drawers` instances in two genuine frontal frames.
  A bounded `entire cabinet` fallback with the SAME checkpoint and 0.25 threshold
  yielded a complete mask at score 0.699. Original failures remain preserved.
- Official ArtGS `readInfo_2states`, `train_predict` and `train` sample each
  state's camera list independently. The adapter now retains non-common
  calibrated training views and separately reserves held-out observations.
  Per-output `official_runtime/arguments` prevents parallel assets overwriting
  native predicted joint metadata. Official training code is unchanged.

These repairs do not prove multistate interaction or reconstruction success.
The complete physical/data/backend experiment must still run. The original
start and spent time remain deducted; the absolute 11:00 cutoff is unchanged.
