package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import monologue.Logged;

/** Generalized hardware internals for a swerve module */
public interface ModuleIO extends AutoCloseable, Logged {
  /**
   * Sets the drive voltage of the module.
   *
   * @param voltage The voltage to inputted into the drive motor.
   */
  public void setDriveVoltage(double voltage);

  /**
   * Sets the turn voltage of the module.
   *
   * @param voltage The voltage to inputted into the turn motor.
   */
  public void setTurnVoltage(double voltage);

  /**
   * Returns the distance the wheel traveled.
   *
   * @return The drive encoder position value, in radians.
   */
  public double getDrivePosition();

  /**
   * Returns the current velocity of the wheel.
   *
   * @return The drive encoder velocity value, in radians / seconds.
   */
  public double getDriveVelocity();

  /**
   * Returns the angular position of the module.
   *
   * @return The adjusted turn encoder position value, in radians.
   */
  public Rotation2d getRotation();

  /** Resets all encoders. */
  public void resetEncoders();

  @Override
  public void close();
}
