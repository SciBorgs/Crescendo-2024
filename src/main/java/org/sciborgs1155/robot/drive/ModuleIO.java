package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Logged;

/** Generalized hardware internals for a swerve module */
public interface ModuleIO extends Logged, AutoCloseable {
  /** The method to use when controlling the drive motor. */
  public static enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  /**
   * Returns the name of the swerve module (e.g. "FR" indicating the front right swerve module.)
   *
   * @return The name of the swerve module.
   */
  String name();

  /**
   * Sets the drive voltage of the module.
   *
   * @param voltage The voltage to inputted into the drive motor.
   */
  void setDriveVoltage(double voltage);

  /**
   * Sets the turn voltage of the module.
   *
   * @param voltage The voltage to inputted into the turn motor.
   */
  void setTurnVoltage(double voltage);

  /**
   * Returns the distance the wheel traveled.
   *
   * @return The drive encoder position value, in radians.
   */
  double drivePosition();

  /**
   * Returns the current velocity of the wheel.
   *
   * @return The drive encoder velocity value, in radians / seconds.
   */
  double driveVelocity();

  /**
   * Returns the angular position of the module.
   *
   * @return The adjusted turn encoder position value, in radians.
   */
  Rotation2d rotation();

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  SwerveModuleState state();

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  SwerveModulePosition position();

  /**
   * Returns the desired position of the module.
   *
   * @return The desired position of the module.
   */
  SwerveModuleState desiredState();

  /** Resets all encoders. */
  void resetEncoders();

  /**
   * Sets the setpoint value for the onboard drive motor's PID.
   *
   * @param velocity The velocity setpoint.
   */
  void setDriveSetpoint(double velocity);

  /**
   * Sets the setpoint value for the onboard turn motor's PID.
   *
   * @param angle The angle setpoint.
   */
  void setTurnSetpoint(double angle);

  /**
   * Updates controllers based on an optimized desired state and actuates the module accordingly.
   *
   * <p>This method should be called periodically.
   *
   * @param setpoint The desired state of the module.
   * @param mode The control mode to use when calculating drive voltage.
   */
  void updateSetpoint(SwerveModuleState setpoint, ControlMode mode);

  /**
   * Updates the drive voltage and turn angle.
   *
   * <p>This is useful for SysId characterization, but should never be run otherwise.
   *
   * @param angle The desired angle of the module.
   * @param voltage The voltage to supply to the drive motor.
   */
  void updateInputs(Rotation2d angle, double voltage);

  @Override
  void close();
}
