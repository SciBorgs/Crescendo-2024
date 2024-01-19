package org.sciborgs1155.robot.shooter.pivot;

public interface PivotIO extends AutoCloseable {
  public void setVoltage(double voltage);

  public double getPosition();
}
