"""Physical-frame invariance and no-GT tests for independent twin compilation."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from articulated_demo.kinematics import URDFChain, transform
from interactive_twin.twin import write_twins, audit_initial_geometry, sanitized_estimate

URDF = """<robot name="synthetic_transform_test">
<link name="base"><visual><geometry><box size="0.3 0.3 0.1"/></geometry></visual></link>
<link name="fixture"/>
<joint name="fixture_mount" type="fixed"><parent link="base"/><child link="fixture"/>
 <origin xyz="0.1 -0.2 0.4" rpy="0.25 0.1 -0.35"/></joint>
<link name="door">
 <visual><origin xyz="0.3 0.1 -0.2" rpy="0.1 0.2 0.3"/><geometry><box size="0.6 0.04 0.8"/></geometry></visual>
 <collision><geometry><box size="0.6 0.04 0.8"/></geometry></collision>
 <inertial><origin xyz="0.25 0.1 0.4" rpy="0.15 -0.25 0.2"/><mass value="2.3"/>
  <inertia ixx="0.12" ixy="0.001" ixz="0.002" iyy="0.13" iyz="0.003" izz="0.08"/></inertial>
</link>
<joint name="hinge" type="revolute"><parent link="fixture"/><child link="door"/>
 <origin xyz="-0.1 0.25 0.1" rpy="-0.2 0.3 0.4"/><axis xyz="0 0 1"/>
 <limit lower="0" upper="1.5" effort="3" velocity="0.2"/><dynamics friction="0.002" damping="0.003"/>
</joint>
<link name="handle"><visual><geometry><cylinder radius="0.01" length="0.16"/></geometry></visual>
 <collision><origin xyz="0 0.001 0"/><geometry><cylinder radius="0.01" length="0.16"/></geometry></collision>
 <inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
</link>
<joint name="handle_mount" type="fixed"><parent link="door"/><child link="handle"/>
 <origin xyz="0.45 0.06 0.4" rpy="0.1 0.2 -0.1"/></joint>
<link name="handle_tip"><collision><geometry><sphere radius="0.01"/></geometry></collision></link>
<joint name="tip_mount" type="fixed"><parent link="handle"/><child link="handle_tip"/><origin xyz="0 0 0.08"/></joint>
</robot>"""


class TwinArtifacts(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.asset = self.root / "reference"
        (self.asset / "urdf").mkdir(parents=True)
        self.urdf = self.asset / "urdf/test.urdf"
        self.urdf.write_text(URDF)
        (self.asset / "manifest.json").write_text(json.dumps({"asset_id": "test", "joint_name": "hinge",
            "moving_link": "door", "door_link": "handle", "moving_links": ["door", "handle", "handle_tip"],
            "source_joint_axis": "0 0 1", "source_joint_limits_rad": {"lower": 0, "upper": 1.5}}))
        self.world = transform([0.42, -0.25, .1], [.7, -.4, .3])
        axis = np.array([.1, -.9, .3]); axis /= np.linalg.norm(axis)
        self.estimate = {"joint_type": "revolute", "confidence": .99,
            "revolute": {"axis": axis.tolist(), "point_on_axis": [.2, -.3, .15],
                         "radius_m": .25, "position_rmse_m": .0001, "rotation_rmse_rad": .0002}}
        self.poses = []
        T0 = transform([.4, -.2, .3], [.1, -.1, .2])
        p = np.array(self.estimate["revolute"]["point_on_axis"])
        for angle in np.linspace(0, .09, 25):
            H = np.eye(4); H[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            H[:3, 3] = p - H[:3, :3] @ p
            self.poses.append((H @ T0).tolist())

    def tearDown(self):
        self.tmp.cleanup()

    def compile(self, **kwargs):
        return write_twins(self.asset, self.root / "twins", self.estimate, self.world,
            initial_physics_prior={"tau_c": .001, "b": .002}, ee_poses=self.poses, **kwargs)

    def test_initial_solid_and_inertia_poses_unchanged(self):
        before = self.urdf.read_bytes()
        summary = self.compile()
        paths = {name: Path(row["urdf"]) for name, row in summary["versions"].items()}
        audit = audit_initial_geometry(paths["T0"], paths["T1"], self.world)
        self.assertLess(audit["max_translation_error_m"], 1e-12)
        self.assertLess(audit["max_rotation_error_rad"], 1e-12)
        self.assertEqual(self.urdf.read_bytes(), before)
        for version in paths:
            chain = URDFChain(paths[version])
            self.assertEqual(chain.joints["door"].name, "hinge")
            self.assertEqual(chain.joints["handle"].parent, "door")
        # The existing moving-link origin is reexpressed; handle/body solids did
        # not move. This distinction is essential to the native loader.
        self.assertFalse(np.allclose(URDFChain(paths["T0"]).root_to_link("door", {}),
                                     URDFChain(paths["T1"]).root_to_link("door", {})))

    def test_estimated_world_axis_and_motion_correct(self):
        summary = self.compile()
        chain = URDFChain(summary["versions"]["T1"]["urdf"])
        joint = chain.joints["door"]
        W_joint = self.world @ chain.root_to_link(joint.parent, {}) @ joint.origin
        desired = self.estimate["revolute"]
        np.testing.assert_allclose(W_joint[:3, 3], desired["point_on_axis"], atol=1e-12)
        np.testing.assert_allclose(W_joint[:3, :3] @ joint.axis, desired["axis"], atol=1e-12)
        handle0 = self.world @ chain.root_to_link("handle", {})
        handle1 = self.world @ chain.root_to_link("handle", {"hinge": .06})
        axis = np.array(desired["axis"]); center = np.array(desired["point_on_axis"])
        H = np.eye(4); H[:3, :3] = Rotation.from_rotvec(axis * .06).as_matrix()
        H[:3, 3] = center - H[:3, :3] @ center
        np.testing.assert_allclose(handle1, H @ handle0, atol=1e-12)

    def test_identifiable_physics_updated_without_mass_change(self):
        result = {"status": "IDENTIFIABLE_ON_FROZEN_GRID", "accepted_parameters": {"tau_c": .013, "b": .031},
                  "parameter_intervals": {"tau_c": [.01, .02], "b": [.02, .04]}, "J_eff_prior_fixed": .02}
        summary = self.compile(physics_estimate=result)
        self.assertTrue(summary["versions"]["T2"]["physics_updated"])
        tree = ET.parse(summary["versions"]["T2"]["urdf"])
        dynamics = tree.getroot().find("joint[@name='hinge']/dynamics")
        self.assertEqual(float(dynamics.get("friction")), .013)
        self.assertEqual(float(dynamics.get("damping")), .031)
        metadata = json.loads((Path(summary["versions"]["T2"]["asset_root"])/"twin.json").read_text())
        self.assertAlmostEqual(metadata["native_physics_application"]["viscousFrictionCoefficient"], .031*np.pi/180.)
        self.assertEqual(tree.getroot().find("link[@name='door']/inertial/mass").get("value"), "2.3")

    def test_unidentifiable_does_not_promote_best_candidate(self):
        result = {"status": "UNIDENTIFIABLE_UNDER_CURRENT_PROBE", "accepted_parameters": None,
                  "best_grid_candidate_diagnostic": {"tau_c": 1, "b": 2},
                  "parameter_intervals": {"tau_c": [0, 1], "b": [0, 2]}}
        summary = self.compile(physics_estimate=result)
        self.assertFalse(summary["versions"]["T2"]["physics_updated"])
        tree = ET.parse(summary["versions"]["T2"]["urdf"])
        self.assertEqual(float(tree.getroot().find("joint[@name='hinge']/dynamics").get("friction")), .001)

    def test_range_is_measured_not_full_limits(self):
        summary = self.compile()
        observed = summary["observed_range"]
        self.assertAlmostEqual(observed["maximum"], .09)
        self.assertFalse(observed["full_joint_limits_identified"])
        meta = json.loads((self.root / "twins/T1/twin.json").read_text())
        self.assertAlmostEqual(meta["operational_joint_window"]["upper"], .1745329252)
        self.assertEqual(meta["operational_joint_window"]["source"], "frozen policy_not_physical_limit")
        self.assertFalse(meta["geometry_relations"]["geometry_improvement_measured"])

    def test_reject_gt_nonzero_start_and_reference_output(self):
        dirty = {**self.estimate, "evaluation": {"GT_axis": [0, 0, 1]}}
        with self.assertRaisesRegex(ValueError, "GROUND_TRUTH_IN_ESTIMATED_PAYLOAD"):
            sanitized_estimate(dirty)
        with self.assertRaisesRegex(ValueError, "NONZERO_INITIAL_STATE"):
            self.compile(q_initial=.02)
        with self.assertRaisesRegex(ValueError, "OUTSIDE_REFERENCE_ASSET"):
            write_twins(self.asset, self.asset / "twins", self.estimate, self.world)


if __name__ == "__main__":
    unittest.main()
