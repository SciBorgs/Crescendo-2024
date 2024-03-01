package org.sciborgs1155.robot.feeder;

public class NoFeeder implements FeederIO {
  @Override
  public void setPower(double power) {}

  @Override
  public boolean beambreak() {
    return true;
  }

  @Override
  public double current() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
