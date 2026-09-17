import os
from pathlib import Path

import numpy as np

from grasp_family_expansion import generate


ROOT = Path(__file__).resolve().parents[1]


def _pose(name):
    data = np.load(Path(os.environ["FR3_ASSET_DIRECTORY"]) / f"{name}_mesh.npz")
    z = -float(data["vertices"][:, 2].min()) + 0.001
    pose = np.eye(4)
    pose[:3, 3] = [0.5, 0.0, z]
    return pose


def test_no_asset_specific_source_logic():
    source = (ROOT / "src/grasp_family_expansion.py").read_text()
    for forbidden in ("if bowl", "if plate", "if mug", "wooden_bowl", "clay_plates"):
        assert forbidden not in source.lower()


def test_wide_mesh_generates_retention_checked_fallback():
    name = "hot3d__wooden_bowl"
    result = generate(name, _pose(name))
    assert result["target_extents_m"][0] > result["gripper_max_width_m"]
    assert result["local_wall_pairs"] > 0
    assert "family_feasibility_thresholds" in result
    assert sum(result["family_feasibility_rejections"].values()) > 0
    assert not result["prefer_before_parent"]
    assert all(x["family_feasibility"]["passed"] for x in result["candidates"])


def test_output_is_deterministic():
    name = "hot3d__clay_plates"
    first = generate(name, _pose(name))
    second = generate(name, _pose(name))
    assert first["prefer_before_parent"] == second["prefer_before_parent"]
    assert [x["family"] for x in first["candidates"]] == [x["family"] for x in second["candidates"]]
    assert np.allclose(
        [x["T_B_TCP"] for x in first["candidates"]],
        [x["T_B_TCP"] for x in second["candidates"]],
    )


def test_executor_uses_separate_anygrasp_and_fallback_budgets():
    source = (ROOT / "calibration/unseen_backend.py").read_text()
    compact = source.replace(" ", "")
    assert "ANYGRASP_ATTEMPT_BUDGET=3" in source
    assert "FAMILY_FALLBACK_ATTEMPT_BUDGET=2" in source
    assert "ANYGRASP_FIRST_THEN_FAMILY_FALLBACK" in source
    assert "metrics=(family+metrics)" not in compact
    assert "metrics=(metrics+family)" not in compact
