package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import monologue.Annotations.Log;
import monologue.Logged;

/** Class to encapsulate a REV Max Swerve module */
public class SwerveModule implements Logged, AutoCloseable {

  /** The method to use when controlling the drive motor. */
  public static enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  private final ModuleIO hardware;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public final String name;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve using vortexes (flex) or krakens (talon).
   *
   * @param hardware talon OR flex swerve module
   * @param angularOffset offset from drivetrain
   */
  public SwerveModule(ModuleIO hardware, Rotation2d angularOffset, String name) {
    this.hardware = hardware;
    this.name = name;
    
    setpoint = new SwerveModuleState();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Log.NT
  public SwerveModuleState state() {
    return new SwerveModuleState(hardware.driveVelocity(), hardware.rotation());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  @Log.NT
  public SwerveModulePosition position() {
    return new SwerveModulePosition(hardware.drivePosition(), hardware.rotation());
  }

  /**
   * Updates controllers based on an optimized desired state and actuates the module accordingly.
   *
   * <p>This method should be called periodically.
   *
   * @param setpoint The desired state of the module.
   * @param mode The control mode to use when calculating drive voltage.
   * @param movementRatio The ratio of translational velocity to the sum of rotational and
   *     translational velocity being requested of the entire swerve drive. 1 for only translation,
   */
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode, double movementRatio) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint = SwerveModuleState.optimize(setpoint, hardware.rotation());
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(hardware.rotation()).getCos();

    hardware.setDriveSetpoint(setpoint.speedMetersPerSecond, mode == ControlMode.OPEN_LOOP_VELOCITY);
    hardware.setTurnSetpoint(setpoint.angle.getRadians());

    this.setpoint = setpoint;
  }

  @Log.NT
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  public void resetEncoders() {
    hardware.resetEncoders();
  }

  @Override
  public void close() {
    hardware.close();
  }
}
