"""Robot-q self exclusion for release observations, never reconstruction images.

Uses official robot envelopes and camera calibration. No instance IDs, object
pose/type/axis, contact ownership, or collision-geometry modifications.
"""
import numpy as np
import cv2


def robot_projection_mask(depth,K,T_world_camera,geometries,poses):
    """Conservative visible robot silhouette; keep observed surfaces in front.

    The closest depth of each projected convex robot envelope is an occlusion
    bound, not a penetration tolerance. Missing observations cannot prove safe
    release. Raw RGB-D and reconstruction masks are never changed.
    """
    depth=np.asarray(depth);K=np.asarray(K);C=np.linalg.inv(T_world_camera)
    excluded=np.zeros(depth.shape,bool)
    for name,mesh in geometries.items():
        if name not in poses:continue
        T=C@poses[name];v=np.asarray(mesh.vertices)@T[:3,:3].T+T[:3,3]
        # Legacy observation cameras are outside the robot. A crossing-near-plane
        # envelope is not projectable as a finite convex footprint.
        if len(v)<3 or np.any(v[:,2]<=0):continue
        pixels=(v@K.T);pixels=pixels[:,:2]/pixels[:,2,None]
        if not np.all(np.isfinite(pixels)):continue
        polygon=cv2.convexHull(np.rint(pixels).astype(np.int32))
        coverage=np.zeros(depth.shape,np.uint8);cv2.fillConvexPoly(coverage,polygon,1)
        excluded|=coverage.astype(bool)&np.isfinite(depth)&(depth>=v[:,2].min())
    return excluded
