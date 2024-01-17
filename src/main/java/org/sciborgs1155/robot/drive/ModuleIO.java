package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import org.sciborgs1155.lib.Fallible;

/** Generalized SwerveModule with closed loop control */
public interface ModuleIO extends Fallible, Sendable, AutoCloseable {
  /** Returns the current state of the module. */
  public SwerveModuleState getState();

  /** Returns the current position of the module. */
  public SwerveModulePosition getPosition();

  /** Sets the desired state for the module. */
  public void setDesiredState(SwerveModuleState desiredState);

  /** Returns the desired state for the module. */
  public SwerveModuleState getDesiredState();

  /** Zeroes all the drive encoders. */
  public void resetEncoders();

  /** Sets the turn PID constants for the module. */
  public void setTurnPID(double kP, double kI, double kD);

  /** Sets the drive PID constants for the module. */
  public void setDrivePID(double kP, double kI, double kD);

  @Override
  default void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("current velocity", () -> getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("current angle", () -> getPosition().angle.getRadians(), null);
    builder.addDoubleProperty("current position", () -> getPosition().distanceMeters, null);
    builder.addDoubleProperty(
        "target velocity", () -> getDesiredState().speedMetersPerSecond, null);
    builder.addDoubleProperty("target angle", () -> getDesiredState().angle.getRadians(), null);
  }
}
