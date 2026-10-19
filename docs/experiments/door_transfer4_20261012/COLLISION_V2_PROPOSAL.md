# Collision representation proposal — not implemented in this batch

Current experiments use original-v1 colliders. No geometry, mass, inertia,
friction, dynamics or mounting height is changed by the shared controller update.

The offline audit identifies positive visual rear gaps but zero/negative native
rear gaps for several bow handles. Whole-mesh convex hulls can fill the bow's
cavity and move the first supporting plane toward a wedge/front edge. This is a
representation limitation, distinct from an actual parallel-jaw hardware limit
or a failed IK search. It does not prove that every alternative front grasp is
infeasible.

A separately versioned `handle-collision-v2` experiment should use one family-wide
geometry rule: partition each protruding handle into connected support/bar
components and apply bounded convex decomposition per component. Retain original
visual meshes, rigid links/joints, object mass/inertia, materials, friction,
actuator limits and mounting. Preserve stable source-mesh IDs and archive both
original-v1 and proposed-v2 signatures. Apply the same rule to all eligible bow
handles, without per-asset hand-drawn proxies.

Evaluate v2 first against visual/native silhouette and rear-space agreement,
common-aperture pad support and contact-plane locations. Then compare a separately
registered contact batch using the SAME frozen controller/configuration. Do not
change representation and control simultaneously or relabel original-v1 results.
The current four-case batch provides no physical evidence for v2.
