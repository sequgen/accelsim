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

def test_uniform_rotation():
    R0 = 1  # Radius of rotation
    w = 2*np.pi  # Angular speed

    def a(t):
        '''Acceleration in the inertial system'''
        return np.matrix([[.0],
                          [.0]])

    def R(t):
        '''Relative displacement'''
        return R0 * np.matrix([[np.cos(w*t)],
                               [np.sin(w*t)]])

    def omega(t):
        '''Relative orientation matrix'''
        return np.matrix([[+np.cos(w*t), np.sin(w*t)],
                          [-np.sin(w*t), np.cos(w*t)]])

    ts = np.linspace(0, 1, 500)
    accels = a_ni(ts, a, R, omega)
    accels_x = accels[0, ]
    accels_y = accels[1, ]

    expected_a_x = w ** 2 * R0
    expected_a_y = 0

    assert( accels_x == pytest.approx(expected_a_x, abs = 1e-2) )
    assert( accels_y == pytest.approx(expected_a_y, abs = 5e-2) )


def test_uniform_rotation_absolute():
    R0 = 1  # Radius of rotation
    w = 2*np.pi  # Angular speed

    def a(t):
        '''Acceleration in the inertial system'''
        return np.matrix([[.0],
                          [.0]])

    def R(t):
        '''Relative displacement'''
        return R0 * np.matrix([[np.cos(w*t)],
                               [np.sin(w*t)]])

    ts = np.linspace(0, 1, 500)
    accels = a_ni_abs(ts, a, R)
    expected_a = w ** 2 * R0

    assert(accels == pytest.approx(expected_a, abs=1e-2))

def test_uniform_rotation_rollercoaster():
    R0 = 1  # Radius of rotation
    w = 2*np.pi / 25  # Angular speed


    def a(t):
        '''Acceleration in the inertial system'''
        return np.matrix([[.0],
                          [.0]])

    def R(t):
        '''Relative displacement'''
        return R0 * np.matrix([[np.cos(w*t)],
                               [np.sin(w*t)]])


    # Times to plot
    ts = np.linspace(0, 1, 500)
    accels = a_ni_auto(ts, a, R)
    accels_x = accels[0, ]
    accels_y = accels[1, ]

    expected_a_x = 0 
    expected_a_y = - w ** 2 * R0

    assert(accels_x == pytest.approx(expected_a_x, abs=1e-2))
    assert(accels_y == pytest.approx(expected_a_y, abs=5e-2))
