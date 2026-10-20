import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from interaction_runtime.registered_observation_v71 import initialize


class InitialImageTest(unittest.TestCase):
    def runtime(self,folder,latched=None,reference=None,loads=(0,0),settled=True):
        return SimpleNamespace(g={'wrist_config':{'settle_s':.5},'wrist_routes':[]},
            output=Path(folder),robot=SimpleNamespace(get_joint_positions=lambda:np.zeros(13)),
            move_path=Mock(),hold=Mock(),actual=lambda:0.,grasp_reference=reference,
            filtered=lambda:loads,policy=SimpleNamespace(data={'force':{'unloaded_pad_n':.05}}),
            supervisor=SimpleNamespace(latched=latched,force=SimpleNamespace(settled=lambda:settled)))

    def test_original_feasible_route_is_preserved(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.runtime(d);ops=SimpleNamespace(observe=Mock(return_value={'fresh':True}))
            route={'q_path':[[0]*7,[.1]*7]}
            with patch('interaction_runtime.registered_observation_v71.initial_route',return_value=(route,[])):
                initialize(r,ops)
            r.move_path.assert_called_once_with([[.1]*7],'PREGRASP');ops.observe.assert_called_once()
            self.assertEqual(r.g['wrist_routes'],[route])
            self.assertFalse(json.loads((Path(d)/'INITIAL_OBSERVATION_INFRASTRUCTURE.json').read_text())['fallback_used'])

    def test_no_route_observes_measured_unloaded_wrist_without_motion(self):
        with tempfile.TemporaryDirectory() as d:
            r=self.runtime(d);ops=SimpleNamespace(observe=Mock(return_value={'fresh':np.float64(1)}))
            with patch('interaction_runtime.registered_observation_v71.initial_route',return_value=(None,[{'reason':'NO_ARM_PATH'}])):
                initialize(r,ops)
            r.move_path.assert_not_called();r.hold.assert_called_once_with(.5);ops.observe.assert_called_once()
            packet=json.loads((Path(d)/'INITIAL_OBSERVATION_INFRASTRUCTURE.json').read_text())
            self.assertTrue(packet['fallback_used']);self.assertFalse(packet['grasp_or_view_target_supplied'])

    def test_unloaded_guard_is_not_bypassed(self):
        for args in [dict(latched={}),dict(reference=object()),dict(loads=(.06,0)),dict(settled=False)]:
            with tempfile.TemporaryDirectory() as d:
                r=self.runtime(d,**args);ops=SimpleNamespace(observe=Mock())
                with patch('interaction_runtime.registered_observation_v71.initial_route',return_value=(None,[])):
                    with self.assertRaisesRegex(RuntimeError,'VERIFIED_UNLOADED'):initialize(r,ops)
                ops.observe.assert_not_called();r.move_path.assert_not_called()


if __name__=='__main__':unittest.main()
