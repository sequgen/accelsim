import pytest
import numpy as np
from accelsim.kinematics import *

# Assert that a free-falling accelerometer measures no acceleration
def test_free_fall():

    # Define input
    g = 9.81  # Acceleration of gravity (in m/s^2)

    def a(t):
        '''Acceleration in the inertial system'''
        return np.matrix([[.0],
                          [-g]])

    def R(t, R0=np.matrix([[0], [0]]), v0=np.matrix([[0], [0]]), a0=np.matrix([[0], [-g]])):
        '''Relative displacement'''
        return R0 + v0 * t + 0.5 * a0 * t**2

    def omega(t):
        '''Relative orientation matrix'''
        return np.identity(2)

    # Calculate accelerations
    ts = np.linspace(0, 1, 500)
    accels = a_ni(ts, a, R, omega)

    # Assert they are as expected
    tol = 0.05
    assert(np.max(accels) < tol)
