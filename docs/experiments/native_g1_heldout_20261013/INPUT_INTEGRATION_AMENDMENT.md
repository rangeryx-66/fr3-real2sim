# Evaluation input integration amendment

The ten-object registration at `b7f9039` predates all IK planning and physical outcomes. Asset IDs, registered handles, order, authored geometry and frozen manipulation code remain unchanged. No object is replaced and no physical attempt is repeated.

Two preparation defects are retained in the evidence:

1. The new authored offline export retained `original-XX` in collider identifiers, whereas the frozen matcher expects importer-normalized identifiers. All cases failed association before any IK call. A uniform identifier-only correction preserved geometry/transforms; the malformed input and error logs remain in `input_identity_error_before_IK`. Every valid planning invocation uses the same frozen planner with `wall_s=300`.
2. The old generic asset preparer retained the microwave-example `door_link=l_1`. Official attempts on 45526 and 45638 failed during scene initialization because their authored moving visual links are different. These are initialization/software failures, with no valid grasp result. Both official reservations remain consumed; no retry is authorized. Exit code 0 from the application's automatic shutdown does not imply successful scene initialization.

After these initialization failures, later launches were paused. The already running 45667 episode was left untouched. Only not-yet-reserved case metadata was corrected using the single visual descendant of the authored active joint, matching the registered handle's link. Geometry, joint origins/limits, scores, plans, station/grasp poses, controller gains, preload and protections were not changed. Original manifests and both freeze snapshots are preserved. The original scheduling process completes 45667; the continuation scheduler skips all already-attempted cases and launches only the remaining registered first-six cases. This is independent-episode scheduling, not manual continuation of a scene.

This input repair occurred after official initialization outcomes. **The batch must not be described as an uninterrupted, unchanged-input execution batch.** The manipulation algorithm is frozen, but the input integration was amended. Report the six-case overall outcome and initialized physical cases separately; do not turn initialization failures into physical grasp failures or remove them from the registered denominator. The four later cases remain reserved.

All ten generated executable bodies have SHA256 `9c4132d61f88487e9282f2ee324b82f017c4f60b633bd633fe84cafa72140165`; entry runner SHA256 remains `c08fb22edcf6cffdddef32c3ddaf37a8123f6f7f59025d5e785e8305b97065e9`. Native configuration SHA256 remains `68fb6af5f4efd432b2355cabe37876a67f6c5ea3eac0b224603f08fb18d62939`.

GPU 2 became empty before the first launch. Hardware-only GPU selection changed from the provisional GPU 4 to GPU 2 before any official attempt; no shared-GPU authorization was needed. Per-case ownership/inventory snapshots are private evidence.
