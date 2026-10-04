"""Protocol semantics for the independent robot-only prerequisite."""
import copy
import unittest
import numpy as np
from interactive_twin_robot_response.calibration import predict, evaluate


def record():
    dt = 1/240; t = np.arange(0, 4, dt)
    r = np.minimum(np.maximum(t-.5, 0), 1.5)*.0005
    T = np.repeat(np.eye(4)[None], len(t), axis=0)
    return {'time_s': t.tolist(), 'dt_s': dt, 'source': 'synthetic', 'T': T.tolist(),
            'direction_world': [1., 0, 0], 'reference_world_m': np.c_[r, r*0, r*0].tolist(),
            'active': ((t >= .5)&(t < 2)).tolist(), 'initial_T': np.eye(4).tolist(),
            'initial_velocity_m_s': [0., 0, 0]}


CFG = {'K_n_m': 150., 'D_ns_m': 100., 'train_window_s': [0., 2.],
       'validation_window_s': [2., 4.]}


class RobotResponseTests(unittest.TestCase):
    def test_command_latency_is_causal(self):
        r = record(); y = predict(r, [.075, .02], CFG); t = np.array(r['time_s'])
        self.assertEqual(float(np.max(np.abs(y[t < .575]))), 0.)
        self.assertGreater(y[-1, 0], 0.)

    def test_stop_removes_stiffness_without_teleport(self):
        r = record(); y = predict(r, [0., .02], CFG); t = np.array(r['time_s'])
        i = np.flatnonzero(t >= 2)[0]
        self.assertGreater(y[-1, 0], y[i, 0])
        self.assertLess(y[-1, 0], r['reference_world_m'][-1][0])
        self.assertLess(abs(y[-1, 1]), 1e-20)

    def test_model_does_not_read_gt_or_future_measured_motion(self):
        r = record(); y = predict(r, [0., .02], CFG)
        altered = copy.deepcopy(r)
        altered['T'] = (np.ones((len(r['T']), 4, 4))*999).tolist()
        altered['GT_hinge'] = {'axis': [1., 0, 0], 'torque': 999}
        np.testing.assert_array_equal(predict(altered, [0., .02], CFG), y)

    def test_parameter_uncertainty_is_propagated_not_bias_inflated(self):
        r = record(); y = predict(r, [0., .02], CFG)
        T = np.array(r['T']); T[:, 0, 3] = y[:, 0]+.001
        r['T'] = T.tolist()
        noise = {'response_covariance': (np.eye(9)*1e-12).tolist()}
        report = evaluate([r], [0., .02], np.zeros((2, 2)), CFG, noise)[0]
        np.testing.assert_allclose(np.array(report['total_scales'])[:, 0], 1e-6)
        self.assertGreater(report['segments']['validation']['normalized_rms'], 100.)


if __name__ == '__main__':
    unittest.main()
