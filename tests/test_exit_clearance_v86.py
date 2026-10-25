"""Disjoint solids inside native contact offset must not pass parking search."""
import unittest
from types import SimpleNamespace
import numpy as np
from agentic_interaction_v81.exit_clearance import ParkingClearance,contact_offset
try:import fcl
except ImportError:fcl=None


class ClearanceTests(unittest.TestCase):
    def test_body_offset_uses_maximum_without_shape_order_guess(self):
        self.assertEqual(contact_offset(dict(path='shape',runtime_body_shapes={'contact_offsets':[[.0004,.0006]]})),.0006)

    def test_unknown_offset_fails_closed(self):
        with self.assertRaises(ValueError):contact_offset(dict(path='shape'))

    @unittest.skipIf(fcl is None,'native FCL required')
    def test_physx_contact_envelope_rejected_but_larger_clearance_passes(self):
        def shape(path,body,x):
            return SimpleNamespace(path=path,body=body,bounds=np.array([[x-.005,-.005,-.005],[x+.005,.005,.005]]),
                object=fcl.CollisionObject(fcl.Box(.01,.01,.01),fcl.Transform(np.array([x,0.,0.]))))
        finger=shape('finger','gripper_link2',0.)
        panel=shape('panel','panel',.0107)
        entries={p:{'path':p,'runtime_body_shapes':{'contact_offsets':[[.0006]]}}for p in ['finger','panel']}
        scene=SimpleNamespace(robot=[finger],scene=[panel],entries=entries,model=SimpleNamespace(allowed=set()),
            check=lambda *args:(True,'GEOMETRICALLY_DISJOINT'))
        checked=ParkingClearance(scene)
        self.assertEqual(fcl.collide(finger.object,panel.object),0)
        self.assertFalse(checked.check({},np.eye(4),False)[0])
        # Contact-mode queries and the actual online guard retain original behavior.
        self.assertTrue(checked.check({},np.eye(4),True)[0])
        panel.object.setTranslation(np.array([.012,0.,0.]));panel.bounds[:,0]+=.0013
        self.assertTrue(checked.check({},np.eye(4),False)[0])


if __name__=='__main__':unittest.main()
