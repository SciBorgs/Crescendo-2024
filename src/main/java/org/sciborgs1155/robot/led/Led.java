package org.sciborgs1155.robot.led;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Led extends SubsystemBase implements Logged{
    private final LedIO led = Robot.isReal() ? new RealLed() : new SimLed();
    
}
