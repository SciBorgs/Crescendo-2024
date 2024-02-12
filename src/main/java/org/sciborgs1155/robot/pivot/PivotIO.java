package org.sciborgs1155.robot.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Logged;

public interface PivotIO extends AutoCloseable, Logged {
  public void setVoltage(double voltage);

  public Rotation2d getPosition();

  /**
   * @return Velocity of pivot in radians per second
   */
  public double getVelocity();
}
