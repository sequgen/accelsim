import numpy as np
from scipy.misc import derivative
from scipy.linalg import norm
from accelsim.decorators import arrayify, arrayify_abs

@arrayify
def a_ni(t,
         a=lambda t: np.matrix([[0], [-9.8]]),
         R=lambda t: np.matrix(([0], [0])),
         omega=lambda t: np.identity(2)):
    '''Vectorial acceleration in the non-intertial system'''

    # Auxiliary functions
    def V(t, dx=1e-7, **kwargs):
        '''Numerical estimation of relative speed'''
        return derivative(R, x0=t, dx=dx, **kwargs)

    def A(t, dx=1e-7, **kwargs):
        '''Numerical estimation of relative acceleration'''
        return derivative(V, x0=t, dx=dx, **kwargs)

    # Rotate external acceleration
    rot_a = omega(t)*a(t)

    # Calculate and rotate relative acceleration
    rot_ni = omega(t)*A(t)

    # Apply formula
    return rot_a - rot_ni

@arrayify_abs
def a_ni_abs(t,
             a=lambda t: np.matrix([[0], [-9.8]]),
             R=lambda t: np.matrix(([0], [0]))):
    '''Absolute acceleration in the non-intertial system'''

    # Auxiliary functions
    def V(t, dx=1e-7, **kwargs):
        '''Numerical estimation of relative speed'''
        return derivative(R, x0=t, dx=dx, **kwargs)

    def A(t, dx=1e-7, **kwargs):
        '''Numerical estimation of relative acceleration'''
        return derivative(V, x0=t, dx=dx, **kwargs)

    return norm(a(t) - A(t))


def a_ni_auto(t,
              a=lambda t: np.matrix([[0], [-9.8]]),
              R=lambda t: np.matrix(([0], [0]))):
    '''Vectorial acceleration in the non-intertial system'''

    # Auxiliary functions
    def V(t, dx=1e-7, **kwargs):
        '''Numerical estimation of relative speed'''
        return derivative(R, x0=t, dx=dx, **kwargs)

    def vx(t):
        ''' Component x of velocity'''
        return float(V(t)[0])

    def vy(t):
        ''' Component y of velocity'''
        return float(V(t)[1])

    def omega(t):
        '''Relative orientation matrix'''
        magnitude = np.sqrt(vx(t)**2 + vy(t)**2)
        matrix = np.matrix([[vx(t), vy(t)],
                            [-vy(t), vx(t)]])

        return 1 / magnitude * matrix

    return a_ni(t, a, R, omega)
