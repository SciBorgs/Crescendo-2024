package org.sciborgs1155.robot.shooter.feeder;

public interface FeederIO extends AutoCloseable {
  public void set(double voltage);
}
