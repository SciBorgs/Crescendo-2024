package org.sciborgs1155.robot.led;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Led extends SubsystemBase implements Logged{
    private final LedIO led = Robot.isReal() ? new RealLed() : new SimLed();
    
    public static enum LEDTheme{
        RAINBOW,
        BXSCI,
        IN_INTAKE,
        IN_PASSING,
        IN_SHOOTER,
        AUTO,
        EXPLODE //means error now, unless you want to implement the ability for the robot to explode (potentially good strategy)
    }

    //work on commands and stuff
}
