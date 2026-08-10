import numpy as np

from pendulum_game_controlled import compute_lqr_gain

M, m, l, g, d_cart, d_pend = 5, 0.5, 0.5, 9.81, 0.15, 0.15
Q = np.diag([1.0, 1.0, 10.0, 1.0])
R = np.array([[1.0]])


def linearized_matrices():
    A = np.array([
        [0, 1, 0, 0],
        [0, -d_cart / M, m * g / M, -d_pend / (M * l)],
        [0, 0, 0, 1],
        [0, -d_cart / (l * M), (M + m) * g / (l * M), -(M + m) * d_pend / (m * l**2 * M)],
    ])
    B = np.array([[0], [1 / M], [0], [1 / (l * M)]])
    return A, B


def test_closed_loop_is_stable():
    K = compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R)
    A, B = linearized_matrices()

    closed_loop = A - B @ K
    eigenvalues = np.linalg.eigvals(closed_loop)

    assert (eigenvalues.real < 0).all()


def test_gain_shape():
    K = compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R)
    assert K.shape == (1, 4)
