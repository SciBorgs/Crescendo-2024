package org.sciborgs1155.robot.feeder;

public class NoFeeder implements FeederIO {
  @Override
  public void set(double power) {}

  @Override
  public double getVelocity() {
    return 0;
  }

  // @Override
  // public boolean frontBeamBreak() {
  //   return false;
  // }

  // @Override
  // public boolean backBeamBreak() {
  //   return false;
  // }

  @Override
  public void close() throws Exception {}
}
