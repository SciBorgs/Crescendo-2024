package org.sciborgs1155.robot.pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public class NoPivot implements PivotIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public Rotation2d getPosition() {
    return new Rotation2d();
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
