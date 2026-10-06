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


def common_release_points(before,after,anchor,radius=.10):
    """Compare surfaces visible in BOTH current frames, not newly revealed pixels.

    Opening a finger reveals object pixels; adding those only to the after-cloud
    biases ICP even when the object does not move. Union of measured-q robot
    envelopes excludes that visibility change without using instance masks.
    """
    from articulated_interaction_skill.capture import backproject
    K=np.asarray(before['K']);T=np.asarray(before['T_world_camera'])
    if not np.allclose(K,after['K']) or not np.allclose(T,after['T_world_camera']):
        raise RuntimeError('RELEASE_CAMERA_MOVED_REOBSERVE_REQUIRED')
    A=np.asarray(before['depth_m']);B=np.asarray(after['depth_m'])
    valid=np.isfinite(A)&np.isfinite(B)&(A>.02)&(A<3.)&(B>.02)&(B<3.)&~np.asarray(before['robot_q_self_mask'],bool)&~np.asarray(after['robot_q_self_mask'],bool)
    P,_=backproject(A,K,T,valid,stride=2);Q,_=backproject(B,K,T,valid,stride=2)
    if len(P)!=len(Q):raise RuntimeError('RELEASE_COMMON_VISIBILITY_CORRESPONDENCE_UNAVAILABLE')
    keep=(np.linalg.norm(P-anchor,axis=1)<radius)&(np.linalg.norm(Q-anchor,axis=1)<radius)
    return P[keep],Q[keep]
