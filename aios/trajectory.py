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
    phi: float
        The current angle that the shooter is at with respect to the y-axis
    v : float
        The current launch speed of the shooter.
    theta : float
        The angle relative to the horizontal that the shooter is inclined at. 
    angleWeight : float
        The weight in the loss function of a change in angle
    speedWeight : float
        The weight in the loss function of a change in shooter launching speed.
    headingWeight : float
        The weight in the loss function of a change in shooter heading.


    Methods 
    -------
    getNewShooterState()
        Returns a dict of a new state the motor should be in. (Launch speed and angle of inclination relative to floor).
    """
    def __init__(self, x:float, y:float, vx:float, vy:float, phi:float, v : float, theta:float, angleWeight:float, speedWeight:float, headingWeight:float):
        """
        Parameters
        ----------
        x : float
            The x-coordinate of the robot at the time of launch.
        y: float
            The y-coordinate of the robot at the time of launch.
        vx: float
            The change in the x-coordinate with respect to time at launch time.
        vy : float
            The change in the y-coordinate with respect to time at launch time.
        phi: float
            The current angle that the shooter is at with respect to the y-axis. Towards the left side is negative and towards the right is positive (viewing the opening of the speaker)
        launchSpeed : float
            The current launch speed of the shooter.
        currentAngleState : float
            The angle relative to the horizontal that the shooter is inclined at. 
        angleWeight : float
            The weight in the loss function of a change in angle.
        speedWeight : float
            The weight in the loss function of a change in shooter launching speed.
        headingWeight : float
            The weight in the loss function of a change in shooter heading.

        """

        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.phi = phi
        self.v = v
        self.theta = theta
        self.angleWeight = angleWeight
        self.speedWeight = speedWeight
        self.headingWeight = headingWeight

        self.MIN_HEIGHT = 1.98 #m
        self.MAX_HEIGHT = 2.11 #m
        self.GRAVITATIONAL_ACCELERATION = 9.805 #m/s/s
        self.OPENING_GAP = 0.46 #m
        self.SPEAKER_WIDTH = 1.05 #m
        self.NOTE_OUTER_RADIUS = 0.36 #m
        self.NOTE_THICKNESS = 0.06 #m
        self.MAX_LAUNCH_SPEED = 40 #m/s

    def getNewShooterState(self) -> dict:
        """
        Returns a dict of a new rotational speed, angle, and heading for the motor to be.
        """
  

        #Variable initialization
        opti = casadi.Opti()
        deltaV = opti.variable()
        deltaTheta = opti.variable()
        deltaPhi = opti.variable()

        if self.vx == 0:
            self.vx = 0.0001
        if self.vy == 0:
            self.vy = 0.0001
       
        opti.minimize( 
           ( self.angleWeight * deltaTheta ** 2) + (self.speedWeight * deltaV ** 2) + (self.headingWeight * deltaPhi ** 2)
        )

        opti.subject_to(
            0 < self.v + deltaV
        )

        opti.subject_to(
            self.v + deltaV <= self.MAX_LAUNCH_SPEED
        )
        opti.subject_to(
            0 < self.theta + deltaTheta 
        )
        
        opti.subject_to(
            self.theta + deltaTheta < math.pi / 2
        )

        opti.subject_to(
            -1 * math.pi / 2 < self.phi + deltaPhi 
        )

        opti.subject_to(
            self.phi + deltaPhi < math.pi / 2
        )

        opti.subject_to(
            self.MAX_HEIGHT - (self.NOTE_THICKNESS / 2) >
           ( (self.v + deltaV) * casadi.sin(deltaTheta + self.theta) * (self.y - self.OPENING_GAP)/ ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy ) )
           - 0.5 * self.GRAVITATIONAL_ACCELERATION * ((self.y - self.OPENING_GAP)/ ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy )) ** 2
        )

        opti.subject_to(
            self.MIN_HEIGHT + (self.NOTE_THICKNESS / 2) >
           ( (self.v + deltaV) * casadi.sin(deltaTheta + self.theta) * (self.y)/ ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy ) )
           - 0.5 * self.GRAVITATIONAL_ACCELERATION * ((self.y)/ ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy )) ** 2
        )

        opti.subject_to(
            0 <=
           ( (self.v + deltaV) * casadi.sin(deltaTheta + self.theta) * (1)/ ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy ) )
           - self.GRAVITATIONAL_ACCELERATION * (self.y)/ ( ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy )) ** 2
        )

        opti.subject_to(
            (-1 * self.SPEAKER_WIDTH + self.NOTE_OUTER_RADIUS) / 2 < self.x + (( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.sin(self.phi + deltaPhi)) + self.vx) * (self.y)/ ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy )
        )

        opti.subject_to(
            (self.SPEAKER_WIDTH - self.NOTE_OUTER_RADIUS) / 2 > self.x + (( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.sin(self.phi + deltaPhi)) + self.vx) * (self.y)/ ( ( (self.v + deltaV) * casadi.cos(self.theta + deltaTheta) * casadi.cos(self.phi + deltaPhi) ) + self.vy )
        )


        
        opti.solver('ipopt')
        try:
            solutions = opti.solve()
        except:
            return ValueError("No calculations for shooting available at this position.")


        return {
            'speed' : round(solutions.value(deltaV) + self.v, 3), 
            'angle' : round(solutions.value(deltaTheta) + self.theta, 3),
            'heading' : round(solutions.value(deltaPhi) + self.phi , 3)
        }

    def canShootWhileStationary(self) -> bool:
        if (self.y < (self.OPENING_GAP / self.SPEAKER_WIDTH ) * (self.x) - (self.OPENING_GAP / 2)) and (self.y < -1 * (self.OPENING_GAP / self.SPEAKER_WIDTH ) * (self.x) - (self.OPENING_GAP / 2)):
            return True
        else:
            return False

    





