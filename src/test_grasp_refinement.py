import numpy as np
from grasp_refinement import local_samples, apply_offset, SEED

def main():
    a = local_samples(); b = local_samples()
    assert a.shape == (256, 6) and np.array_equal(a, b)
    assert np.array_equal(a[0], np.zeros(6))
    assert np.max(abs(a[:, :2])) <= .005 and np.max(abs(a[:, 2])) <= .010
    assert np.max(abs(a[:, 3:5])) <= 10 and np.max(abs(a[:, 5])) <= 15
    H = np.eye(4); H[:3, 3] = [1, 2, 3]
    assert np.allclose(apply_offset(H, np.zeros(6)), H)
    print('GRASP_REFINEMENT_TEST_OK', SEED)

if __name__ == '__main__': main()
