import numpy as np
from scipy.optimize import least_squares
from trajectory import Trajectory
import random
import os


"""
Represents the functions for launch speed, and angle as a function of the positon of the robot.

...

Attributes
----------
n : int
The number of datapoints to generate per iteration.
N: int
The number of times to iterate.

Methods
-------
getV()
Returns the needed launch speed at the position (x,y)
getTheta()
Returns the needed launch angle at the position (x,y)
getAlpha()
Returns the needed heading at the position (x,y)
getVelocityVector()
Returns the new state of the robot that the robot should be in at the time of launch. 

"""

#Field constants
SPEAKER_LENGTH = 1.05 #m
STATION_LENGTH = 1.75 #m
FIELD_LENGTH = 16.54 #m
SPEAKER_WIDTH = 0.92 #m
MIN_X = round(-2 * STATION_LENGTH - (SPEAKER_LENGTH / 2))
MAX_X = round(((SPEAKER_LENGTH / 2) + STATION_LENGTH))
MIN_Y = round(0)
MAX_Y = round(FIELD_LENGTH - SPEAKER_WIDTH)

n = 1000 #The number of datapoints to generate per iteration.
N = 10 # The number of times to iterate.

# Define the function f(x, y) = z (This was found to be linear for v, and quadratic for theta.)
def fV(params, x, y):
    a, b, c = params
    return (a * x) + (b * y)  + c

def fTheta(params, x, y):
    a, b, c = params
    return (a * x ** 2) + (b * y ** 2)  + c

# Define the objective function (residuals)
def objectiveTheta(params, x, y, z):
    return fTheta(params, x, y) - z

# Define the objective function (residuals)
def objectiveV(params, x, y, z):
    return fV(params, x, y) - z

def returnFunctions(self):
    # Sample points
    xv_data = np.array([])
    yv_data = np.array([])
    zv_data = np.array([])

    xt_data = np.array([])
    yt_data = np.array([])
    zt_data = np.array([])

    av = np.array([])
    bv = np.array([])
    cv = np.array([])

    at = np.array([])
    bt = np.array([])
    ct = np.array([])
    k = 0
    while k < self.N:
        #Add sample points.
        j = 0
    while j <= 1:
        i = 0
        while i <= self.n:
            if j == 0:
                x = random.randrange(MIN_X, MAX_X)
                y = random.randrange(MIN_Y, MAX_Y)
                z = Trajectory(x, y).results()[0]
                if z != 0:
                    zv_data = np.append(zv_data, z)
                    xv_data = np.append(xv_data, x)
                    yv_data = np.append(yv_data, y)
                    i += 1
            else:
                x = random.randrange(MIN_X, MAX_X)
                y = random.randrange(MIN_Y, MAX_Y)
                z = Trajectory(x, y).results()[1]
                if z > 0 and z < np.pi / 2:
                    zt_data = np.append(zt_data, z)
                    xt_data = np.append(xt_data, x)
                    yt_data = np.append(yt_data, y)
                    i += 1

        j += 1
    # Initial guess for parameters
    initial_guess = [1, 1, 1]

    # Least squares optimization
    resultV = least_squares(self.objectiveV, initial_guess, args=(xv_data, yv_data, zv_data))
    resultTheta = least_squares(self.objectiveTheta, initial_guess, args=(xt_data, yt_data, zt_data))


    optimized_paramsv = resultV.x
    optimized_paramst = resultTheta.x
    av = np.append(av, optimized_paramsv[0])
    bv = np.append(bv, optimized_paramsv[1])
    cv = np.append(cv, optimized_paramsv[2])

    at = np.append(at, optimized_paramst[0])
    bt = np.append(bt, optimized_paramst[1])
    ct = np.append(ct, optimized_paramst[2])
    k += 1



    returnAv = np.average(av)
    returnBv = np.average(bv)
    returnCv = np.average(cv)

    returnAt = np.average(at)
    returnBt = np.average(bt)
    returnCt = np.average(ct)

    launch_speed_coefficients = np.array([returnAv, returnBv, returnCv])
    angle_coefficents = np.array([returnAt, returnBt, returnCt])

    return launch_speed_coefficients, angle_coefficents #Launch speed function coefficients, angle function coefficents.


