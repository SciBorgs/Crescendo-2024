package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class NoModule implements ModuleIO {

  @Override
  public void setDriveVoltage(double voltage) {}

  @Override
  public void setTurnVoltage(double voltage) {}

  @Override
  public double drivePosition() {
    return 0;
  }

  @Override
  public double driveVelocity() {
    return 0;
  }

  @Override
  public Rotation2d rotation() {
    return new Rotation2d();
  }

  @Override
  public void resetEncoders() {}

  @Override
  public void close() {}
}
