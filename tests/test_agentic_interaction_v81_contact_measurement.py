import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from agentic_interaction_v81.contact_measurement import relative_contact_speed


class ContactMeasurementTests(unittest.TestCase):
    def contact(self):
        return dict(finger='f',allowed_pad_target=True,force_n=1,
                    contact_point_world_m=[.3,0,0],native_normal_world=[0,0,1])

    def test_co_rotating_contact_has_no_tangential_slip(self):
        A=np.eye(4);B=A.copy();B[:3,:3]=Rotation.from_rotvec([0,0,.001]).as_matrix()
        self.assertAlmostEqual(relative_contact_speed({'f':A},{'f':B},A,B,[self.contact()],1/240),0)

    def test_true_relative_translation_and_missing_reports(self):
        A=np.eye(4);B=A.copy();B[0,3]=.001
        self.assertAlmostEqual(relative_contact_speed({'f':A},{'f':B},A,A,[self.contact()],1/240),.24)
        self.assertIsNone(relative_contact_speed({'f':A},{'f':B},A,A,[],1/240))
        self.assertIsNone(relative_contact_speed({'f':A},{'f':B},A,A,[self.contact()],.1))


if __name__=='__main__':unittest.main()
