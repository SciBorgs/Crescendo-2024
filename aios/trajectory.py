import casadi
import math

class Trajectory:
    """
    Represents a trajectory object. 
    Returns an optimal setting of angle relative to horizontal
    and shooter launch speed to send the note into the speaker.

    The origin has been chosen as the center of the speaker-back that is being scored into. "x-axis" is parallel to side that speaker is attached to.

    ...

    Attributes
    ----------
    x : float
        The x-coordinate of the robot at the time of launch
    y: float
        The y-coordinate of the robot at the time of launch.
    vx: float
        The change in the x-coordinate with respect to time at launch time.
    vy : float
        The change in the y-coordinate with respect to time at launch time.
    launchSpeed : float
        The current launch speed of the shooter.
    currentAngleState : float
        The angle relative to the horizontal that the shooter is inclined at. 
    angleWeight : float
        The weight in the loss function of a change in angle
    speedWeight : float
        The weight in the loss function of a change in shooter launching speed.

    Methods 
    -------
    getNewShooterState()
        Returns a dict of a new state the motor should be in. (Launch speed and angle of inclination relative to floor).
    """
    def __init__(self, x:float, y:float, vx:float, vy:float, launchSpeed : float, currentAngleState:float, angleWeight:float, speedWeight:float):
        """
        Parameters
        ----------
        x : float
            The x-coordinate of the robot at the time of launch
        y: float
            The y-coordinate of the robot at the time of launch.
        vx: float
            The change in the x-coordinate with respect to time at launch time.
        vy : float
            The change in the y-coordinate with respect to time at launch time.
        launchSpeed : float
            The current launch speed of the shooter.
        currentAngleState : float
            The angle relative to the horizontal that the shooter is inclined at. 
        angleWeight : float
            The weight in the loss function of a change in angle
        speedWeight : float
            The weight in the loss function of a change in shooter launching speed.

        """

        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.launchSpeed = launchSpeed
        self.currentAngleState = currentAngleState
        self.angleWeight = angleWeight
        self.speedWeight = speedWeight
        self.MIN_HEIGHT = 1.98 #m
        self.MAX_HEIGHT = 2.11 #m
        self.GRAVITATIONAL_ACCELERATION = 9.805 #m/s/s
        self.OPENING_GAP = 0.46 #m
        self.SPEAKER_WIDTH = 1.05 #m
        self.NOTE_OUTER_RADIUS = 0.36 #m
        self.NOTE_THICKNESS = 0.06 #m
        self.MAX_LAUNCH_SPEED = 40 #m/s

    def getNewShooterState(self):
        """
        Returns a dict of a new rotational speed and angle for the motor to be.
        """
        
        if self.vx == 0:
            self.vx = 0.001
        #Get heading relative to center of speaker back
        if self.currentAngleState == 0 or self.launchSpeed == 0:
            self.currentAngleState = math.pi / 4
            self.launchSpeed = self.MAX_LAUNCH_SPEED / 2


        #Variable initialization
        opti = casadi.Opti()
        deltaV = opti.variable()
        deltaTheta = opti.variable()


        opti.minimize( 
           ( self.angleWeight * deltaTheta ** 2) + (self.speedWeight * deltaV ** 2)
        )

        opti.subject_to(
            self.MIN_HEIGHT + (self.NOTE_THICKNESS / 2) < 
            (self.launchSpeed + deltaV) * casadi.sin(deltaTheta + self.currentAngleState) * ( (abs(self.y) - self.OPENING_GAP) / (self.vy + ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState))))
            - 0.5 * self.GRAVITATIONAL_ACCELERATION * ((abs(self.y) - self.OPENING_GAP) / (self.vy + ((self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState)))) ** 2
        )

        opti.subject_to(
            self.MAX_HEIGHT - (self.NOTE_THICKNESS / 2) < 
            (self.launchSpeed + deltaV) * casadi.sin(deltaTheta + self.currentAngleState) * ( (abs(self.y)) / (self.vy + ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState))))
            - 0.5 * self.GRAVITATIONAL_ACCELERATION * ((abs(self.y)) / (self.vy + ((self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState)))) ** 2
        )

        opti.subject_to(
            0 <= 
            (self.launchSpeed + deltaV) * casadi.sin(deltaTheta + self.currentAngleState) * ( 1 / (self.vy + ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState))))
            - self.GRAVITATIONAL_ACCELERATION * (abs(self.y) - self.OPENING_GAP) * (1 / (self.vy + ((self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState)))) ** 2
        )

        
        opti.subject_to(
           0 < self.currentAngleState + deltaTheta
        )

        opti.subject_to(
           math.pi / 2 > self.currentAngleState + deltaTheta
        )
        
        opti.subject_to(
           0 < self.launchSpeed + deltaV 
        )

        opti.subject_to(
            self.launchSpeed + deltaV < self.MAX_LAUNCH_SPEED
        )
        

        opti.subject_to((self.SPEAKER_WIDTH - self.NOTE_OUTER_RADIUS) / 2 > self.vx * ((abs(self.y) - self.OPENING_GAP) / (self.vy + ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState)))) + self.x)

        opti.subject_to(
            (- 1 * self.SPEAKER_WIDTH + self.NOTE_OUTER_RADIUS) / 2 <
            self.vx * ((abs(self.y)) / (self.vy + (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState))) + self.x
        )

        opti.subject_to(
            (self.SPEAKER_WIDTH - self.NOTE_OUTER_RADIUS) / 2 > 
            self.vx * ((abs(self.y)) / (self.vy + (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState))) + self.x
        )
        opti.solver('ipopt')

        solutions = opti.solve()


        return {
            'speed' : round(solutions.value(deltaV) + self.launchSpeed, 3), 
            'angle' : round(solutions.value(deltaTheta) + self.currentAngleState, 3),
        }



    





