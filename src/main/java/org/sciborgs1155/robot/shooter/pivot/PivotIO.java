package org.sciborgs1155.robot.shooter.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Logged;

public interface PivotIO extends AutoCloseable, Logged {
  public void setVoltage(double voltage);

  public Rotation2d getPosition();

  // change to a unit of rotational velocity later. for now im just using a double cause is easier
  public double getVelocity();
}
