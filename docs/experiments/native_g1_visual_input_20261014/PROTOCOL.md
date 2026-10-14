# Native G1 visual input: matched four-case development batch

Reference `30f5b30`. Cases in fixed order: **45671, 45638, 45600, 45606**.
This round replaces grasp/contact-ranking geometry only. Frozen Oracle results,
closure, preload, Pink, actuator/physics, one-second hold and protections remain.

Fresh closed-start full-frame RGB, optical-Z depth, K/extrinsics and robot state
feed RGB-only SAM3. No instance masks, GT handle cropping, stored grasp poses or
GT pose corrections. Nine longitudinal observed profiles use a declared prior:
unseen depth equals local observed transverse width. The observed panel ring is
fit from depth; scoring-only panel thickness is 10 mm. Neither prior changes
execution colliders. The frozen ranking API's `visual/native` surface labels
both mean observed geometry plus this prior in this adapter.

Rank uses the same weights, native G1 pads, candidate families, 18-candidate
shortlist and frozen perturbations. These are not visual confidence bounds.
GT/native object geometry enters existing collision feasibility and native
contact identity only. The registered station, known hinge/dynamic state and
robot/camera calibration remain known-model dependencies.

Shadow captures/plans occur before physical outcomes. Each physical run acquires
fresh RGB-D again, regenerates its grasp/approach/IK, reobserves at pregrasp and
uses measured-q robot exclusion/common-visible RGB-D correction. Initial
perception gets one bounded recapture; no GT fallback. Planning budget 450 s.
No changes between physical cases; one valid closed-start episode each:
visual grasp, original one-second hold, approximately +3° opening. ≥2° gained
AFTER acquisition is the paired short outcome, not a 30° or held-out score.

Actors are sequential; sufficiently free GPU memory, not GPU exclusivity. All
finished actors close after cooperative export. Raw RGB-D, video and 240 Hz
contact/robot evidence remain private. Publish code, hashes and compact outcomes.

Pre-physical observation repair: the original overview could hide the handle
behind the home arm. One bounded alternate view fits a visible vertical depth
plane after measured-q robot exclusion, then observes from camera-FOV-fitted standoff,
300 mm on the side opposite the robot centroid, and 100 mm above its observed
center. An optional full-RGB SAM3 cabinet mask restricts the depth volume before fitting; absence is logged and uses the same raw-depth plane/FOV framing rather than preventing re-observation. the same auxiliary prompt is used on all cases. This fixed rule uses no GT handle or per-asset offsets. Camera pose is
then held constant for common-visible pregrasp correction. The initial failed
observation is preserved and consumed no contact episode.
