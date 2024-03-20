package org.sciborgs1155.robot.shooter;

public class NoWheel implements WheelIO {

  @Override
  public void close() throws Exception {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double velocity() {
    return 0.0;
  }
}
