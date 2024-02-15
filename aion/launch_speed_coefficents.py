from matplotlib import pyplot as plt
import numpy as np
from trajectory import Trajectory
from scipy.optimize import curve_fit

"""
Finds the coefficents for a quadratic function approximating the needed launch speed as a function of position (x,y)
"""




speaker_length = 1.05 #m
station_length = 1.75 #m
field_length = 16.54 #m
subwoofer_length = 0.92 #m
min_x = round(-2 * station_length - (speaker_length / 2)) #m
max_x = round(((speaker_length / 2) + station_length)) #m
min_y = 0 #m
max_y = round(field_length - subwoofer_length) #m

def f(variables, a, b, c, d, e, f): 
    x, y = variables
    return a + b*x + c*y + d*x**2 + e*y**2 + f*x*y



def return_coefficients():
    # Sample points
    x_positions = np.array([])
    y_positions = np.array([])
    launch_speeds = np.array([])
  
    x = min_x
    while x < max_x:
        y = min_y
        while y < max_y:
            launch_speed = Trajectory(x, y).get_optimal_settings()[0]
            if launch_speed != 0:
                x_positions = np.append(x_positions, x)
                y_positions = np.append(y_positions, y)
                launch_speeds = np.append(launch_speeds, launch_speed)
            y += 0.1
        x += 0.1

    popt, _ = curve_fit(f, (x_positions, y_positions), launch_speeds) 
    return popt


print(return_coefficients())