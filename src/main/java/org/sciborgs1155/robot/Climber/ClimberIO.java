package org.sciborgs1155.robot.climber;

/** ClimberIO */
public interface ClimberIO {

  // public void climberInit();

  // public void setMotorSpeed(double speed);

  // public Command stopExtending();

  // public Command retract();

  // public Command fullyExtend();

  // public Command moveToGoal(double goal);

  public double getVelocity();

  public double getPosition();

  // public void setMotortype(int portID, MotorType motortype);

  public void set(double speed);

  public void setPosition(double position);

  public void initialize();
}
