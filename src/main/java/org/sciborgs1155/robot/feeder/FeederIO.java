package org.sciborgs1155.robot.feeder;

public interface FeederIO extends AutoCloseable {
  public void set(double power);

  public double getVelocity();

  // public boolean frontBeamBreak();

  // public boolean backBeamBreak();
}
