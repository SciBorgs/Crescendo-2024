from matplotlib import pyplot as plt
import numpy as np
from trajectory import Trajectory
from scipy.optimize import curve_fit
import multiprocessing

"""
Finds the coefficents for a quadratic function approximating the needed launch angle as a function of position (x,y)
"""

speaker_length = 1.05  # m
station_length = 1.75  # m
field_length = 16.54  # m
subwoofer_length = 0.92  # m
min_x = round(-2 * station_length - (speaker_length / 2))  # m
max_x = round(((speaker_length / 2) + station_length))  # m
min_y = 0  # m
max_y = round(field_length - subwoofer_length)  # m


def f(variables, a, b, c, d, e, f):
    x, y = variables
    return a + b * x + c * y + d * x**2 + e * y**2 + f * x * y


def optimal_values(trajectory: Trajectory):
    angle, velocity = trajectory.get_optimal_settings()
    return (angle, velocity, trajectory.posX, trajectory.posY)


def return_coefficients():
    cases = [
        Trajectory(x, y)
        for y in np.arange(min_y, max_y, 0.2)
        for x in np.arange(min_x, max_x, 0.2)
    ]
    results = np.array(multiprocessing.Pool().map(optimal_values, cases))

    velocity_fit, _ = curve_fit(f, (results[:, 2], results[:, 3]), results[:, 0])
    pitch_fit, _ = curve_fit(f, (results[:, 2], results[:, 3]), results[:, 1])
    return (velocity_fit, pitch_fit)


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


# the code doesn't work without this
if __name__ == "__main__":
    vel_coeffs, pitch_coeffs = return_coefficients()
    print(
        "velocity:\n"
        + coeffs_to_string(vel_coeffs)
        + "\npitch:\n"
        + coeffs_to_string(pitch_coeffs)
    )
