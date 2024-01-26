import casadi
import math

class trajectory:
    """
    Represents a trajectory object. 
    Returns an optimal setting of angle relative to horizontal
    and shooter launch speed to send the note into the speaker.

    ...

    Attributes
    ----------
    distanceToTarget : str
        The distance from the robot to the center of the start of the speaker
    currentSpeedState : float
        The current speed at which the shooter will shoot the note (in m/s)
    currentAngleState : float
        The angle relative to the horizontal that the shooter is inclined at. 
    currentHeadingState : float
        The angle formed by the edge of the field containing the target speaker and the center of the edge of the speaker to the robot. 
    angleWeight : float
        The weight in the loss function of a change in angle
    speedWeight : float
        The weight in the loss function of a change in shooter launching speed.

    Methods 
    -------
    getNewShooterState()
        Returns a dict of a new rotational speed and angle for the motor to be.
    """
    def __init__(self, distanceToTarget:float, currentSpeedState:float, currentAngleState:float, currentHeadingState:float, angleWeight:float, speedWeight:float):
        """
        Parameters
        ----------
        distanceToTarget : str
            The distance from the robot to the center of the start of the speaker
        currentSpeedState : float
            The current launch speed of the shooter
        currentAngleState : float
            The angle relative to the horizontal that the shooter is inclined at. 
        currentHeadingState : float
            The angle formed by the edge of the field containing the target speaker and the center of the edge of the speaker to the robot. 
        angleWeight : float
            The weight in the loss function of a change in angle
        speedWeight : float
            The weight in the loss function of a change in shooter launching speed.
        """
    

        self.distanceToTarget = distanceToTarget
        self.currentSpeedState = currentSpeedState
        self.currentAngleState = currentAngleState
        self.currentHeadingState = currentHeadingState
        self.angleWeight = angleWeight
        self.speedWeight = speedWeight
        self.MIN_HEIGHT = 1.98 #m
        self.MAX_HEIGHT = 2.11 #m
        self.GRAVITATIONAL_ACCELERATION = 9.805 #m/s/s
        self.OPENING_GAP = 0.46 #m



    
    def getNewShooterState(self):
        """
        Returns a dict of a new rotational speed and angle for the motor to be.
        """
        
        #Variable initialization
        opti = casadi.Opti()
        deltaV = opti.variable()
        deltaTheta = opti.variable()

        opti.minimize( 
           ( self.angleWeight * deltaTheta ** 2) + (self.speedWeight * deltaV ** 2)
        )

        opti.subject_to(
            self.MIN_HEIGHT < ((self.distanceToTarget + self.OPENING_GAP) * casadi.tan(self.currentAngleState + deltaTheta))
            -  ( (self.GRAVITATIONAL_ACCELERATION * (self.distanceToTarget + self.OPENING_GAP) ** 2) / 
              (2 * ( (self.currentSpeedState + deltaV) ** 2) * (casadi.cos(self.currentAngleState + deltaTheta) ** 2) )
            )
        )
        opti.subject_to(
            self.MAX_HEIGHT > (self.distanceToTarget * casadi.tan(self.currentAngleState + deltaTheta))
            -  ( (self.GRAVITATIONAL_ACCELERATION * self.distanceToTarget ** 2) / 
              (2 * ( (self.currentSpeedState + deltaV) ** 2) * (casadi.cos(self.currentAngleState + deltaTheta) ** 2) )
            )
        )

        opti.subject_to(
            0 < (casadi.tan(self.currentAngleState + deltaTheta))
            -  ( (self.GRAVITATIONAL_ACCELERATION * self.distanceToTarget) / 
              (( (self.currentSpeedState + deltaV) ** 2) * (casadi.cos(self.currentAngleState + deltaTheta) ** 2) )
            )
        )

        opti.subject_to(self.currentSpeedState + deltaV < 30)
        

        opti.solver('ipopt')
        solutions = opti.solve()

        return {
            'speed' : round(solutions.value(deltaV) + self.currentSpeedState, 3), 
            'angle' : round(solutions.value(deltaTheta) + self.currentAngleState, 3),
        }



    





