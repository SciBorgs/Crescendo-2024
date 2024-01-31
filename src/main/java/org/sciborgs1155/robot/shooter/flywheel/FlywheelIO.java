package org.sciborgs1155.robot.shooter.flywheel;

public interface FlywheelIO extends AutoCloseable {
  public void setVoltage(double voltage);

  /**
   * @return flywheel velocity in radians per second
   */
  public double getVelocity();
}
