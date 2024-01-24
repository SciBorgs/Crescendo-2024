package org.sciborgs1155.robot.shooter.pivot;

import monologue.Logged;

public interface PivotIO extends AutoCloseable, Logged {
  public void setVoltage(double voltage);

  public double getPosition();
}
