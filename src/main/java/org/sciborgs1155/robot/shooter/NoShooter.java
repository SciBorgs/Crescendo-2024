package org.sciborgs1155.robot.shooter;

public class NoShooter implements ShooterIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public double getCurrent() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
