package org.sciborgs1155.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;

/** ClimberIO */
public interface ClimberIO {

  public void climberInit();

  public void setMotorSpeed(double speed);

  public Command stopExtending();

  public Command retract();

  public Command fullyExtend();

  public Command moveToGoal(double goal);
}
