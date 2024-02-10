import numpy as np

class RobotSettingsGivenState:
    """
    Represents the current robot state at the time of launch.

    ...

    Attributes
    ----------
    x : float
        The x-coordinate of the robot at the time of launch.
    y : str
        The y-coordinate of the robot at the time of launch.
    vx : float
        The velocity in the x-direction of the robot at the time of launch.
    vy : float
        The velocity in the y-direction of the robot at the time of launch.

    Methods
    -------
    getV()
        Returns the needed luanch speed at the position (x,y)
    getTheta()
        Returns the needed launch angle at the position (x,y)
    getAlpha()
        Returns the needed heading at the position (x,y)
    getVelocityVector()
        Returns the new state of the robot that the robot should be in at the time of launch. 
    
    """
    
    av = 0.001062497903615323
    bv = 0.7471887693867607
    cv = 5.478684772893152

    at = 7.752605287137327e-05
    bt = -0.007477191156019437
    ct = 1.0663329616679225

    def __init__(self, posX, posY, vx, vy):
        self.posX = posX
        self.posY = posY
        self.vx = vx
        self.vy = vy

    def getV(self):
        return (self.av * self.posX) + (self.bv * self.posY) + self.cv

    def getTheta(self):
        return ( (self.at * self.posX ** 2) + (self.bt * self.posY ** 2) + self.ct ) 

    def getAlpha(self):
        if self.posX == 0:
            return np.pi / 2
        return np.arctan(self.posY / self.posX)
    
    def getVelocityVector(self):
        v = self.getV()
        theta = self.getTheta()
        alpha = self.getAlpha()
        deltaVx = v * np.cos(alpha) - self.vx
        deltaVy = v * np.sin(alpha) - self.vy 
        if deltaVx == 0:
            returnAlpha = np.pi / 2
        else:
            returnAlpha = np.arctan(deltaVy / deltaVx)
        returnvx = np.cos(returnAlpha) * deltaVx
        returnvy = np.cos(returnAlpha) * deltaVy
        returnV = np.sqrt(((returnvx ** 2) + (returnvy ** 2)))
        return v + returnV, theta, returnAlpha
    

        
t1 = RobotSettingsGivenState(0, 3, 0, 0)
print(t1.getVelocityVector())
        

    


