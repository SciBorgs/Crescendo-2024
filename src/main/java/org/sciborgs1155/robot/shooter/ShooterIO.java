package org.sciborgs1155.robot.shooter;

import monologue.Logged;

public interface ShooterIO extends AutoCloseable, Logged {
  public void setVoltage(double voltage);

  double getCurrent();

  /**
   * @return Shooter velocity in radians per second.
   */
  public double getVelocity();
}
