package org.sciborgs1155.robot.shooter;

public interface ShooterIO extends AutoCloseable {
  public void setVoltage(double voltage);

  /**
   * @return Shooter velocity in radians per second.
   */
  public double getVelocity();
}
