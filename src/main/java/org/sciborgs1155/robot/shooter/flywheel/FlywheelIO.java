package org.sciborgs1155.robot.shooter.flywheel;

public interface FlywheelIO extends AutoCloseable {
  public void setVoltage(double voltage);

  public double getVelocity();
}
