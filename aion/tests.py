import numpy as np
from trajectory import Trajectory


"""
Test the functions created from the coefficents found.
"""


def theta_approx(x, y):
    return (
        1.1331172768630184
        + 0.0337170229983295 * x
        + -0.07822480760293148 * y
        + -0.010386903450326593 * x**2
        + -0.00030007103195798433 * y**2
        + -0.0042478354516679185 * x * y
    )


def v_approx(x, y):
    return (
        6.453655233490886
        + -0.31621564232862487 * x
        + 1.0962750645654968 * y
        + 0.0012312720547558165 * x**2
        + -0.11192054441863619 * y**2
        + 0.03819377475764463 * x * y
    )
