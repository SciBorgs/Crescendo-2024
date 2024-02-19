import numpy as np
from trajectory import Trajectory


"""
Test the functions created from the coefficents found.
"""


def theta_approx(x, y):
    return (
        1.19902863
        + 0.03853062 * x
        + -0.09096639 * y
        + -0.01398042 * x**2
        + 0.00175755 * y**2
        + -0.00832195 * x * y
    )


def v_approx(x, y):
    return (
        6.97564148
        + -0.18031224 * x
        + 0.13736373 * y
        + 0.12140126 * x**2
        + 0.06011271 * y**2
        + 0.0392426 * x * y
    )
