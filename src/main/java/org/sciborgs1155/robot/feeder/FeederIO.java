package org.sciborgs1155.robot.feeder;

public interface FeederIO extends AutoCloseable {
  public void setVoltage(double voltage);

  public double getVelocity();

  public boolean getFrontBeambreakValue();

  public boolean getEndBeambreakValue();
}
