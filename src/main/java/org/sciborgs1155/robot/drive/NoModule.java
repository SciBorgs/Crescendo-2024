package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public class NoModule implements ModuleIO {

  @Override
  public void setDriveVoltage(double voltage) {}

  @Override
  public void setTurnVoltage(double voltage) {}

  @Override
  public double getDrivePosition() {
    return 0;
  }

  @Override
  public double getDriveVelocity() {
    return 0;
  }

  @Override
  public Rotation2d getRotation() {
    return new Rotation2d();
  }

  @Override
  public void resetEncoders() {}

  @Override
  public void close() {}
}
