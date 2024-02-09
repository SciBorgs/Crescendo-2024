package org.sciborgs1155.robot.feeder;

public class NoFeeder implements FeederIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public boolean getFrontBeambreakValue() {
    return false;
  }

  @Override
  public boolean getEndBeambreakValue() {
    return false;
  }

  @Override
  public void close() throws Exception {}
}
