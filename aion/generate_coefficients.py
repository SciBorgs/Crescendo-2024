from matplotlib import pyplot as plt
import numpy as np
from solver import Solver
from trajectory import Trajectory
from scipy.optimize import curve_fit
import multiprocessing

"""
Finds the coefficents for a quadratic function approximating the needed launch angle as a function of position (x,y)
"""

field_width = 8.2296  # 27 ft
field_length = 16.4592  # 54 ft

speaker_length = 1.05  # m
station_length = 1.75  # m
field_length = 16.54  # m
subwoofer_length = 0.92  # m
min_x = 0  # m
max_x = field_length / 4  # m
min_y = 0  # m
max_y = field_width  # m

# global solver
solver = Solver()


def f(variables, a, b, c, d, e, f):
    x, y = variables
    return a + b * x + c * y + d * x**2 + e * y**2 + f * x * y


# global solver
def optimal_values(point):
    global solver
    sol = Solver().optimal_settings(point[0], point[1])
    if sol is not None:
        angle, velocity = sol
        return (angle, velocity, point[0], point[1])


if __name__ == "__main__":
    cases = [
        (x, y)
        for y in np.arange(min_y, max_y, 0.2)
        for x in np.arange(min_x, max_x, 0.2)
    ]
    results = np.array(
        list(
            filter(
                lambda x: x is not None,
                multiprocessing.Pool(multiprocessing.cpu_count() // 4).map(
                    optimal_values, cases
                ),
            )
        )
    )

    print(results)

    velocity_fit, _ = curve_fit(f, (results[:, 2], results[:, 3]), results[:, 0])
    pitch_fit, _ = curve_fit(f, (results[:, 2], results[:, 3]), results[:, 1])

    def coeffs_to_string(coeffs):
        return (
            str(coeffs[0])
            + " + "
            + str(coeffs[1])
            + " * x + "
            + str(coeffs[2])
            + " * y + "
            + str(coeffs[3])
            + " * x * x + "
            + str(coeffs[4])
            + " * y * y + "
            + str(coeffs[5])
            + " * x * y"
        )

    print(
        "velocity:\n"
        + coeffs_to_string(velocity_fit)
        + "\npitch:\n"
        + coeffs_to_string(pitch_fit)
    )
