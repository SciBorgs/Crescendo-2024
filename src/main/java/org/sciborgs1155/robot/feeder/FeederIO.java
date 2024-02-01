package org.sciborgs1155.robot.feeder;

public interface FeederIO extends AutoCloseable {
  public void set(double voltage);
}
