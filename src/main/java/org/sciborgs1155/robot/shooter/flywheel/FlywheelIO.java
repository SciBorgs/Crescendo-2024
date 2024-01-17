package org.sciborgs1155.robot.shooter.flywheel;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public interface FlywheelIO extends AutoCloseable {
  public void setSetpoint(State setpoint);

  public void setVoltage(double voltage);

  public double getVelocity();
}
