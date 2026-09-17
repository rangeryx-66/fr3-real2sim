"""Deterministic mesh-derived grasp families for a parallel-jaw hand.

The generator does not use asset names or semantic classes.  It finds local
opposed surface pairs and expands each pair into approach/depth variants.  The
resulting poses are new execution candidates; AnyGrasp poses, scores and ranks
remain unchanged.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation

from grasp_refinement import PAD_AREA_M2, torque_metrics
from hand_geometry import PAD_X, PAD_Z
from mesh_hand_geometry import evaluate, target_mesh

SEED = 20260913
SURFACE_SAMPLES = 4096
MAX_RAW_PAIRS = 768
OUTPUT_LIMIT = 16
PRECHECK_LIMIT = 64
MAX_WIDTH_M = 0.078
LOCAL_WALL_MAX_M = 0.028
MIN_WIDTH_M = 0.0025
DEPTH_OFFSETS_M = (-0.006, 0.0, 0.006)
TILT_DEG = (-35.0, -18.0, 0.0, 18.0, 35.0)
PALM_CLEARANCE_M = 0.005
MIN_EDGE_ESCAPE_M = 0.0015
MIN_FORCE_CLOSURE_OPPOSITION = 0.90
MAX_COM_LINE_LEVER_M = 0.050
MAX_GRAVITY_ROLL_LEVER_M = 0.045
MAX_FIRST_CONTACT_ASYMMETRY_M = 0.0045
ROOT = Path(__file__).resolve().parents[1]
_PALM = None


def _palm_model():
    global _PALM
    if _PALM is None:
        palm = trimesh.load_mesh(ROOT / "franka_description/meshes/robot_ee/franka_hand_white/collision/hand.stl")
        vertices = np.asarray(palm.vertices) - [0.0, 0.0, 0.1034]
        _PALM = (vertices.min(0) - PALM_CLEARANCE_M, vertices.max(0) + PALM_CLEARANCE_M, ConvexHull(vertices).equations)
    return _PALM


def _palm_clear(mesh, hand, target):
    lower, upper, equations = _palm_model()
    target_to_hand = np.linalg.inv(hand) @ target
    vertices = mesh.vertices @ target_to_hand[:3, :3].T + target_to_hand[:3, 3]
    near = vertices[((vertices >= lower) & (vertices <= upper)).all(1)]
    if not len(near):
        return True
    inside = ((near @ equations[:, :3].T + equations[:, 3]) <= PALM_CLEARANCE_M).all(1)
    return not bool(inside.any())


def _surface_samples(mesh, count=SURFACE_SAMPLES):
    rng = np.random.default_rng(SEED)
    faces = rng.choice(len(mesh.faces), count, p=mesh.area_faces / mesh.area)
    uv = rng.random((count, 2))
    flip = uv.sum(axis=1) > 1.0
    uv[flip] = 1.0 - uv[flip]
    triangles = mesh.triangles[faces]
    points = triangles[:, 0] + uv[:, :1] * (triangles[:, 1] - triangles[:, 0]) + uv[:, 1:] * (
        triangles[:, 2] - triangles[:, 0]
    )
    return points, mesh.face_normals[faces], faces


def _opposed_pairs(mesh):
    points, normals, face_ids = _surface_samples(mesh)
    competing_distance, competing_ids = cKDTree(points).query(
        points, k=48, distance_upper_bound=MAX_WIDTH_M
    )
    rows = []
    for ray in range(len(points)):
        chosen = None
        for distance, index in zip(competing_distance[ray, 1:], competing_ids[ray, 1:]):
            if not np.isfinite(distance) or distance < MIN_WIDTH_M or index >= len(points):
                continue
            direction = (points[index] - points[ray]) / distance
            ray_alignment = float(direction @ -normals[ray])
            opposition = float(normals[ray] @ normals[index])
            if ray_alignment >= 0.80 and opposition <= -0.45:
                chosen = int(index), float(distance), opposition, ray_alignment
                break
        if chosen is None:
            continue
        index, width, opposition, ray_alignment = chosen
        other = points[index]
        closing = other - points[ray]
        closing /= np.linalg.norm(closing)
        midpoint = (points[ray] + other) / 2.0
        rows.append(
            dict(
                point_a=points[ray],
                point_b=other,
                midpoint=midpoint,
                closing=closing,
                width=width,
                normal_opposition=opposition,
                ray_alignment=ray_alignment,
                face_a=int(face_ids[ray]),
                face_b=int(face_ids[index]),
            )
        )
    rows.sort(key=lambda x: (x["width"], x["face_a"], x["face_b"]))
    return rows[:MAX_RAW_PAIRS]


def _family(width, vertical_closing):
    if width <= LOCAL_WALL_MAX_M and vertical_closing <= 0.45:
        return "INNER_OUTER_WALL"
    if width <= 0.018 and vertical_closing > 0.25:
        return "TILTED_EDGE"
    return "OPPOSED_SURFACE"


def _pose(midpoint_B, closing_B, approach_B, contact_depth_m, closing_sign):
    y = closing_B * closing_sign
    z = approach_B / np.linalg.norm(approach_B)
    x = np.cross(y, z)
    x /= np.linalg.norm(x)
    z = np.cross(x, y)
    hand = np.eye(4)
    hand[:3, :3] = np.column_stack([x, y, z])
    hand[:3, 3] = midpoint_B - hand[:3, :3] @ np.array([0.0, 0.0, contact_depth_m])
    return hand


def _deduplicate(rows, limit=None):
    selected = []
    for row in rows:
        hand = row["hand"]
        duplicate = False
        for old in selected:
            other = old["hand"]
            dt = np.linalg.norm(hand[:3, 3] - other[:3, 3])
            dr = (Rotation.from_matrix(hand[:3, :3]).inv() * Rotation.from_matrix(other[:3, :3])).magnitude()
            if dt < 0.0035 and dr < np.deg2rad(10.0):
                duplicate = True
                break
        if not duplicate:
            selected.append(row)
            if limit is not None and len(selected) >= limit:
                break
    return selected


def _distance_to_pad_exit(point_xz, direction_xz):
    """Distance from a contact centroid to the pad boundary along slip."""
    direction = np.asarray(direction_xz, dtype=float)
    norm = np.linalg.norm(direction)
    if norm < 1e-9:
        return float("inf")
    direction /= norm
    bounds = (PAD_X, PAD_Z)
    distances = []
    for value, component, (lower, upper) in zip(point_xz, direction, bounds):
        if component > 1e-9:
            distances.append((upper - value) / component)
        elif component < -1e-9:
            distances.append((lower - value) / component)
    return float(min(distances)) if distances else float("inf")


def _family_feasibility(hand, geometry, torque):
    """Conservative geometry-only retention test for every generated family.

    The checks approximate antipodal force closure and reject contacts whose
    gravity slip direction exits the finite Franka pad too quickly.  They use
    only mesh, COM and hand geometry; no asset names or semantic classes.
    """
    gravity_hand = np.asarray(hand)[:3, :3].T @ np.array([0.0, 0.0, -1.0])
    slip_xz = gravity_hand[[0, 2]]
    centroids = []
    escape = []
    for side in geometry["sides"]:
        points = np.asarray(
            [
                contact["point_TCP"]
                for contact in side.get("contacts", [])
                if contact.get("in_first_patch") and contact.get("in_friction_cone")
            ],
            dtype=float,
        )
        if not len(points):
            centroids.append(None)
            escape.append(0.0)
            continue
        centroid = points.mean(axis=0)
        centroids.append(centroid.tolist())
        escape.append(_distance_to_pad_exit(centroid[[0, 2]], slip_xz))
    reasons = []
    if torque["normal_opposition"] < MIN_FORCE_CLOSURE_OPPOSITION:
        reasons.append("WEAK_FORCE_CLOSURE")
    if torque["COM_contact_line_lever_m"] > MAX_COM_LINE_LEVER_M:
        reasons.append("COM_LINE_LEVER")
    if torque["gravity_roll_lever_m"] > MAX_GRAVITY_ROLL_LEVER_M:
        reasons.append("GRAVITY_ROLL_TORQUE")
    if torque["first_contact_asymmetry_m"] > MAX_FIRST_CONTACT_ASYMMETRY_M:
        reasons.append("FIRST_CONTACT_ASYMMETRY")
    if min(escape) < MIN_EDGE_ESCAPE_M:
        reasons.append("EDGE_ESCAPE_MARGIN")
    return dict(
        passed=not reasons,
        reasons=reasons,
        contact_centroids_TCP_m=centroids,
        gravity_slip_direction_pad_xz=slip_xz.tolist(),
        edge_escape_m=escape,
        min_edge_escape_m=float(min(escape)),
        thresholds=dict(
            min_edge_escape_m=MIN_EDGE_ESCAPE_M,
            min_force_closure_opposition=MIN_FORCE_CLOSURE_OPPOSITION,
            max_COM_contact_line_lever_m=MAX_COM_LINE_LEVER_M,
            max_gravity_roll_lever_m=MAX_GRAVITY_ROLL_LEVER_M,
            max_first_contact_asymmetry_m=MAX_FIRST_CONTACT_ASYMMETRY_M,
        ),
    )


def generate(name, target):
    """Generate geometry-only grasp families in base frame.

    ``target`` is active ``T_B_target``. Generated candidates are fallback-only;
    the function never changes the ordering of an external AnyGrasp proposal.
    """
    target = np.asarray(target, dtype=float)
    mesh = target_mesh(name)
    pairs = _opposed_pairs(mesh)
    gravity_B = np.array([0.0, 0.0, -1.0])
    candidates = []
    family_counts = {}
    for pair_id, pair in enumerate(pairs):
        closing_B = target[:3, :3] @ pair["closing"]
        vertical = float(abs(closing_B @ gravity_B))
        if vertical > 0.75:
            # A near-vertical closing axis forces a near-horizontal approach;
            # for a table-supported object that cannot clear the strict table
            # collision check.  This is a support geometry rule, not a class rule.
            continue
        base = gravity_B - closing_B * float(gravity_B @ closing_B)
        if np.linalg.norm(base) < 0.35:
            continue
        base /= np.linalg.norm(base)
        family = _family(pair["width"], vertical)
        family_counts[family] = family_counts.get(family, 0) + 1
        midpoint_B = target[:3, :3] @ pair["midpoint"] + target[:3, 3]
        com_B = target[:3, :3] @ np.asarray(mesh.center_mass) + target[:3, 3]
        lever = float(np.linalg.norm(np.cross(com_B - midpoint_B, closing_B)))
        height_fraction = float(
            np.clip((pair["midpoint"][2] - mesh.bounds[0, 2]) / max(np.ptp(mesh.bounds[:, 2]), 1e-6), 0.0, 1.0)
        )
        for tilt in TILT_DEG:
            approach_B = Rotation.from_rotvec(closing_B * np.deg2rad(tilt)).apply(base)
            if approach_B[2] > -0.30:
                continue
            for depth in DEPTH_OFFSETS_M:
                # Finger links are geometrically symmetric.  A closing-axis
                # sign flip is the same physical contact family and would
                # otherwise consume half of the finite diversity budget.
                for sign in (1.0,):
                    hand = _pose(midpoint_B, closing_B, approach_B, depth, sign)
                    cheap_cost = (
                        0.15 * np.clip(lever / 0.08, 0.0, 1.0)
                        + 0.10 * np.clip(abs(pair["width"] - 0.012) / 0.040, 0.0, 1.0)
                        + 0.08 * np.clip((0.006 - depth) / 0.012, 0.0, 1.0)
                        + 0.17 * (1.0 - height_fraction)
                        + 0.10 * np.clip(abs(tilt) / 35.0, 0.0, 1.0)
                        + 0.10 * np.clip((pair["normal_opposition"] + 1.0) / 0.55, 0.0, 1.0)
                        + 0.10 * np.clip((1.0 - pair["ray_alignment"]) / 0.20, 0.0, 1.0)
                        + 0.20 * np.clip(vertical / 0.75, 0.0, 1.0)
                    )
                    candidates.append(
                        dict(
                            family=family,
                            pair_id=pair_id,
                            width_m=pair["width"],
                            contact_depth_m=depth,
                            tilt_deg=tilt,
                            closing_sign=int(sign),
                            vertical_closing=vertical,
                            contact_height_fraction=height_fraction,
                            cheap_COM_lever_m=lever,
                            cheap_cost=float(cheap_cost),
                            hand=hand,
                        )
                    )
    candidates.sort(key=lambda x: (x["cheap_cost"], x["family"], x["pair_id"], x["tilt_deg"], x["contact_depth_m"], x["closing_sign"]))
    # Preserve approach/depth diversity before the unchanged exact 1 mm gate.
    strata = [
        ([x for x in candidates if x["tilt_deg"] == 0.0 and x["contact_depth_m"] == 0.0], 24),
        ([x for x in candidates if x["tilt_deg"] == 0.0 and x["contact_depth_m"] != 0.0], 16),
        ([x for x in candidates if x["tilt_deg"] != 0.0 and x["contact_depth_m"] == 0.0], 16),
        ([x for x in candidates if x["tilt_deg"] != 0.0 and x["contact_depth_m"] != 0.0], 8),
    ]
    selected = []
    for pool, quota in strata:
        for row in _deduplicate(pool, max(16, quota * 4)):
            if any(
                np.linalg.norm(row["hand"][:3, 3] - old["hand"][:3, 3]) < 0.0035
                and (
                    Rotation.from_matrix(row["hand"][:3, :3]).inv()
                    * Rotation.from_matrix(old["hand"][:3, :3])
                ).magnitude()
                < np.deg2rad(10.0)
                for old in selected
            ):
                continue
            selected.append(row)
            quota -= 1
            if quota == 0:
                break
    selected = selected[:PRECHECK_LIMIT]
    extents = np.ptp(mesh.vertices, axis=0)
    checked = []
    retention_rejections = {}
    for row in selected:
        hand = row["hand"]
        if not _palm_clear(mesh, hand, target):
            continue
        geometry = evaluate(name, hand, target)
        torque = torque_metrics(name, hand, target, geometry)
        if not geometry["passed"] or not torque["valid"]:
            continue
        retention = _family_feasibility(hand, geometry, torque)
        if not retention["passed"]:
            for reason in retention["reasons"]:
                retention_rejections[reason] = retention_rejections.get(reason, 0) + 1
            continue
        patch = min(side["normal_patch_area_m2"] for side in geometry["sides"])
        patch_deficit = np.clip(1.0 - patch / PAD_AREA_M2, 0.0, 1.0)
        support_clearance = row["contact_height_fraction"] * extents[2]
        support_deficit = np.clip((0.035 - support_clearance) / 0.035, 0.0, 1.0)
        row["stability_cost"] = float(
            0.65 * torque["torque_cost"]
            + 0.15 * patch_deficit
            + 0.08 * np.clip(geometry["first_contact_asymmetry_m"] / 0.010, 0.0, 1.0)
            + 0.07 * np.clip(row["vertical_closing"] / 0.75, 0.0, 1.0)
            + 0.05 * support_deficit
        )
        row["mesh_hand_geometry"] = geometry
        row["torque"] = torque
        row["family_feasibility"] = retention
        row["geometry_passed"] = True
        checked.append(row)
    checked.sort(key=lambda x: (x["stability_cost"], x["cheap_cost"], x["pair_id"]))
    selected = checked[:OUTPUT_LIMIT]
    for rank, row in enumerate(selected):
        hand = row.pop("hand")
        row.update(
            source="MESH_FAMILY",
            family_rank=rank,
            parent_rank=None,
            rank=None,
            score=None,
            refinement_id=0,
            T_B_TCP=hand.tolist(),
            geometry_pending=False,
            offset_translation_TCP_m=[0.0, 0.0, 0.0],
            offset_rotation_TCP_deg=[0.0, 0.0, 0.0],
        )
    planar = np.sort(extents[:2])
    local_wall_count = sum(pair["width"] <= LOCAL_WALL_MAX_M for pair in pairs)
    height_to_min_planar = float(extents[2] / max(planar[0], 1e-6))
    return dict(
        method="deterministic mesh local opposed-pair family expansion; no semantic branches",
        seed=SEED,
        surface_samples=SURFACE_SAMPLES,
        raw_opposed_pairs=len(pairs),
        local_wall_pairs=local_wall_count,
        raw_family_counts=family_counts,
        mesh_checked=min(PRECHECK_LIMIT,len(candidates)),
        mesh_passed=len(checked),
        output_count=len(selected),
        target_extents_m=extents.tolist(),
        height_to_min_planar=height_to_min_planar,
        gripper_max_width_m=MAX_WIDTH_M,
        palm_clearance_m=PALM_CLEARANCE_M,
        family_feasibility_thresholds=dict(
            min_edge_escape_m=MIN_EDGE_ESCAPE_M,
            min_force_closure_opposition=MIN_FORCE_CLOSURE_OPPOSITION,
            max_COM_contact_line_lever_m=MAX_COM_LINE_LEVER_M,
            max_gravity_roll_lever_m=MAX_GRAVITY_ROLL_LEVER_M,
            max_first_contact_asymmetry_m=MAX_FIRST_CONTACT_ASYMMETRY_M,
        ),
        family_feasibility_rejections=retention_rejections,
        # Kept for result-schema compatibility. Family candidates are always
        # a fallback and never precede an AnyGrasp candidate.
        prefer_before_parent=False,
        candidates=selected,
    )
