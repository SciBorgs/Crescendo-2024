import numpy as np
from scipy.optimize import fsolve

class Trajectory:
    """
    Represents a trajectory object

    ...

    Attributes
    ----------
    posX : float
        The x-coordinate of the robot at the time of launch.
    posY : float
        The y-coordinate of the robot at the time of launch.

    Methods
    -------
    returnTrajectorySettingsGivenState()
        Returns the settings for the launcher to launch the note.
    results()
        Returns the settings if they are valid.
    """

    g = 9.8 #m/s/s
    SPEAKER_LENGTH = 1.05 #m
    SPEAKER_WIDTH = 0.46 #m
    MAX_SPEAKER_HEIGHT = 2.11 #m
    MIN_SPEAKER_HEIGHT = 1.98 #m
    NOTE_THICKNESS = 0.06 #m
    MAX_LAUNCH_SPEED = 13 #m/s
    STARTING_HEIGHT = 0 #m
    
    def __init__(self, posX, posY):
        self.posX = posX
        self.posY = posY


    #x[0]->v, x[1]->theta, x[2]->heading
    def _returnTrajectorySettingsGivenPosition(self, x):
        """
        Returns the trajectory settings for the given robot position.

        Parameters
        ----------
        x: (numpy.ndarray)
            An array representing the launch speed of the shooter, the angle relative to the horizontal it is inclined, and the heading relative to the front center of the speaker.
        """
        return [
            np.tan(x[1]) * ((self.posY - self.SPEAKER_WIDTH) / (np.sin(x[2]))) - (1/2) * self.g * (((self.posY - self.SPEAKER_WIDTH) / (x[0] * np.cos(x[1]) * np.sin(x[2]))) ** 2) - (self.MAX_SPEAKER_HEIGHT - (self.NOTE_THICKNESS / 2)) + (self.STARTING_HEIGHT),
            np.tan(x[1]) * ((self.posY) / (np.sin(x[2]))) - (1/2) * self.g * (((self.posY) / (x[0] * np.cos(x[1]) * np.sin(x[2]))) ** 2) - (self.MIN_SPEAKER_HEIGHT + (self.NOTE_THICKNESS / 2)) + self.STARTING_HEIGHT,
            (np.cos(x[2])) * ((self.posY - self.SPEAKER_WIDTH) / (np.sin(x[2]))),
        ]

    def results(self):
        """
        Returns the trajectory settings for the given robot state if valid.
        """
        solutions = fsolve(self._returnTrajectorySettingsGivenPosition, [1, 1, 1])
        if solutions[0] > self.MAX_LAUNCH_SPEED:
            return [0, 0, 0]
        return solutions


t1 = Trajectory(0, 3)

print(t1.results())