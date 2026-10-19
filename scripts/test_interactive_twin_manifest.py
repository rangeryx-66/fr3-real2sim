"""Protocol integrity checks using metadata-only fixtures, not demo assets."""
import copy
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from interactive_twin.manifest import (build_manifest, controller_episode, save_frozen_manifest,
                                       verify_frozen_manifest, validate_policy)
from interactive_twin.prepare import metadata_height


def fixture(root, asset_id, category="Cabinet", clearance=.018, joint_type="revolute"):
    path = root / asset_id
    (path / "urdf").mkdir(parents=True)
    (path / "urdf" / f"{asset_id}.urdf").write_text(f'''<robot name="fixture">
      <link name="base"/><link name="door"/>
      <joint name="joint" type="{joint_type}"><parent link="base"/><child link="door"/>
      <axis xyz="0 0 1"/><limit lower="0" upper="1.5" effort="1" velocity="1"/>
      </joint></robot>''')
    selection = {"asset_id": asset_id, "joint_name": "joint", "joint_type": joint_type,
                 "moving_link": "door", "handle_link": "door", "mesh": "bar",
                 "axis_root": [0, 1, 0], "outward_normal_root": [1, 0, 0],
                 "dimensions_m": [.12, .025, .015],
                 "sections": [{"fraction": 0, "anchor_root_m": [0, 0, 0], "visual_rear_gap_m": clearance}]}
    (path / "manifest.json").write_text(json.dumps({
        "asset_id": asset_id, "category": category,
        "interaction_geometry": {"selection": selection},
        "source_url": "unit-test fixture, not a dataset or benchmark result",
    }))
    return path


def raises(kind, call):
    try:
        call()
    except kind:
        return
    raise AssertionError(f"expected {kind.__name__}")


def main():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        fixture(root, "7320", "Microwave")
        fixture(root, "111", "Cabinet", .020)
        fixture(root, "222", "Cabinet", .019)
        fixture(root, "333", "Microwave", .018)
        fixture(root, "444", "Fridge", .017)
        fixture(root, "555", "Cabinet", .016)
        fixture(root, "666", "Drawer", joint_type="prismatic")
        fixture(root, "777", "Cabinet", clearance=.001)
        a = build_manifest([root])
        b = build_manifest([root])
        assert a["manifest_sha256"] == b["manifest_sha256"]
        assert a["selection_complete"] and a["test_denominator_assets"] == 4
        assert a["test_denominator_episodes"] == 12
        assert a["test_asset_ids"] == ["111", "444", "333", "222"]
        assert "7320" not in a["test_asset_ids"]
        assert len(a["inventory"]) == 8  # Geometry rejections stay in inventory.
        rejected = {r["asset_id"]: r["exclusion_reasons"] for r in a["inventory"]}
        assert "NO_REVOLUTE_JOINT" in rejected["666"]
        assert "NO_SUPPORTED_VISUAL_BAR_HANDLE" in rejected["777"]
        assert "NOT_SELECTED_BY_FROZEN_GEOMETRY_ORDER" in rejected["555"]
        # Execution reports are deliberately not a selection input.
        (root / "111" / "report.json").write_text('{"status":"FAILED","success":false}')
        assert build_manifest([root])["manifest_sha256"] == a["manifest_sha256"]
        episodes = [e for e in a["episodes"] if e["asset_id"] == "111"]
        assert len({json.dumps(e["deployment"], sort_keys=True) for e in episodes}) == 3
        assert [e["deployment"]["along_handle_offset_m"] for e in episodes] == [0, .01, -.01]
        observed = controller_episode(a, "test_111_01")
        serialized = json.dumps(observed)
        assert "joint_position" not in serialized and "joint_type" not in serialized
        assert "limits" not in serialized and "joint_name" not in serialized
        assert "initialization_only" not in observed
        frozen = root / "frozen_benchmark.json"
        save_frozen_manifest(a, frozen)
        save_frozen_manifest(a, frozen)
        verify_frozen_manifest(frozen, verify_inputs=True)
        changed = copy.deepcopy(a)
        changed["test_asset_ids"] = ["555"]
        raises(ValueError, lambda: verify_frozen_manifest(changed))
        altered_policy = build_manifest([root], policy={"seed": 1})
        raises(FileExistsError, lambda: save_frozen_manifest(altered_policy, frozen))
        manifest = root / "111" / "manifest.json"
        manifest.write_text(manifest.read_text() + " ")
        raises(ValueError, lambda: verify_frozen_manifest(a, verify_inputs=True))
        raises(ValueError, lambda: validate_policy({"test_asset_count": 3}))
        raises(ValueError, lambda: validate_policy({"minimum_joint_margin_rad": .03}))
        raises(ValueError, lambda: validate_policy({"budgets": {"grasp_candidates": 13}}))
        raises(ValueError, lambda: validate_policy({"budgets": {"probe_directions": 5}}))
        assert metadata_height({"dimension": "60*40*90"}) == .90
        assert metadata_height({"dimension": [50, 40, 30]}) == .30
        raises(ValueError, lambda: metadata_height({"dimension": None}))
        insufficient = build_manifest([root / "7320"])
        assert not insufficient["selection_complete"] and insufficient["missing_test_assets"] == 4
    print("interactive twin manifest checks passed")


if __name__ == "__main__":
    main()
