package org.sciborgs1155.robot.shooter;

import monologue.Logged;

public interface ShooterIO extends AutoCloseable, Logged {
  public void voltage(double voltage);

  double current();

  /**
   * @return Shooter velocity in radians per second.
   */
  public double velocity();
}
