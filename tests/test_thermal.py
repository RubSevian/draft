import numpy as np
import pytest
from draft.tasks.thermal import estimate_temperature


def test_constant_loss_matches_analytic_solution_and_cooling():
    t = np.array([0., 1., 10., 100.])
    temp, loss = estimate_temperature(t, np.ones((4, 1))*2, np.array([10.]),
        np.array([20.]), ambient_C=25, initial_C=25, resistance_K_W=2, capacity_J_K=5)
    np.testing.assert_allclose(temp[:, 0], 25 + 16*(1-np.exp(-t/10)))
    cool, _ = estimate_temperature(t, np.zeros((4,1)), np.array([10.]),
        np.array([20.]), ambient_C=25, initial_C=60, resistance_K_W=2, capacity_J_K=5)
    np.testing.assert_allclose(cool[:,0], 25 + 35*np.exp(-t/10))


def test_invalid_time_rejected():
    with pytest.raises(ValueError):
        estimate_temperature([0,0], np.zeros((2,1)), [60], [20], ambient_C=25,
            initial_C=25, resistance_K_W=1, capacity_J_K=450)
