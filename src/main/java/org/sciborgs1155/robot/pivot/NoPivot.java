package org.sciborgs1155.robot.pivot;

public class NoPivot implements PivotIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
