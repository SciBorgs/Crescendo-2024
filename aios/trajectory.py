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
        self.MIN_HEADING = math.atan(self.OPENING_GAP/ ( (self.SPEAKER_WIDTH / 2) - self.NOTE_OUTER_RADIUS ) )

    def getNewShooterState(self):
        """
        Returns a dict of a new rotational speed and angle for the motor to be.
        """
        
        #Get heading relative to center of speaker back
        try: 
            heading = math.atan(self.y / self.x )
        
        except ZeroDivisionError:
            heading = math.pi / 2
            print(heading)
    
        if heading < self.MIN_HEADING:
            return ValueError("No launch properties at this heading available.")

        #Variable initialization
        opti = casadi.Opti()
        deltaV = opti.variable()
        deltaTheta = opti.variable()

        loss = ( self.angleWeight * deltaTheta ** 2) + (self.speedWeight * deltaV ** 2)
        opti.minimize( 
           loss
        )

        opti.subject_to(
            self.MAX_HEIGHT - self.NOTE_THICKNESS < (self.launchSpeed + deltaV) * casadi.sin(deltaTheta + self.currentAngleState) * ((self.y - self.OPENING_GAP) / ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState) * math.sin(heading) + self.vy)) - 
            0.5 * self.GRAVITATIONAL_ACCELERATION * ((self.y - self.OPENING_GAP)/ ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState) * math.sin(heading) + self.vy))
        )

        opti.subject_to(
            self.MIN_HEIGHT + self.NOTE_THICKNESS > (self.launchSpeed + deltaV) * casadi.sin(deltaTheta + self.currentAngleState) * ((self.y) / ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState) * math.sin(heading) + self.vy)) - 
            0.5 * self.GRAVITATIONAL_ACCELERATION * ((self.y)/ ( (self.launchSpeed + deltaV) * casadi.cos(deltaTheta + self.currentAngleState) * math.sin(heading) + self.vy))
        )

        opti.solver('ipopt')

        solutions = opti.solve()
    

        return {
            'speed' : round(solutions.value(deltaV) + self.currentSpeedState, 3), 
            'angle' : round(solutions.value(deltaTheta) + self.currentAngleState, 3),
        }