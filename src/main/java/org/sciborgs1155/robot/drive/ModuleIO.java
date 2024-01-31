package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import monologue.Logged;

/** Generalized hardware internals for a swerve module */
public interface ModuleIO extends AutoCloseable, Logged {
  /**
   * Sets the drive voltage of the module.
   *
   * @param voltage The voltage to inputted into the drive motor.
   */
  public void setDriveVoltage(Measure<Voltage> voltage);

  /**
   * Sets the turn voltage of the module.
   *
   * @param voltage The voltage to inputted into the turn motor.
   */
  public void setTurnVoltage(Measure<Voltage> voltage);

  /**
   * Returns the distance the wheel traveled.
   *
   * @return The drive encoder position value, in radians.
   */
  public Measure<Distance> getDrivePosition();

  /**
   * Returns the current velocity of the wheel.
   *
   * @return The drive encoder velocity value, in radians / seconds.
   */
  public Measure<Velocity<Distance>> getDriveVelocity();

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
