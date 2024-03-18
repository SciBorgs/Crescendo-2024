package org.sciborgs1155.robot.shooter;

public class NoShooter implements ShooterIO {

  @Override
  public void close() throws Exception {}

  @Override
  public void setSetpoint(double velocity) {}

  @Override
  public double topVelocity() {
    return 0;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double bottomVelocity() {
    return 0;
  }

  @Override
  public boolean atSetpoint() {
    return false;
  }
}
