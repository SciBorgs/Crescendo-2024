package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import monologue.Annotations.Log;
import monologue.Logged;

/** Class to encapsulate a REV Max Swerve module */
public class SwerveModule implements Logged, AutoCloseable {
  private final ModuleIO module;

  @Log.NT private final PIDController driveFeedback;
  @Log.NT private final PIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public final String name;

  /**
   * Constructs a SwerveModule for rev's MAX Swerve using vortexes (flex) or krakens (talon).
   *
   * @param module talon OR flex swerve module
   * @param angularOffset offset from drivetrain
   */
  public SwerveModule(ModuleIO module, Rotation2d angularOffset, String name) {
    this.module = module;
    this.name = name;
    driveFeedback = new PIDController(Driving.PID.P, Driving.PID.I, Driving.PID.D);
    turnFeedback = new PIDController(Turning.PID.P, Turning.PID.I, Turning.PID.D);
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

    driveFeedforward = new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.A);

    setpoint = new SwerveModuleState();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Log.NT
  public SwerveModuleState state() {
    return new SwerveModuleState(module.getDriveVelocity(), module.getRotation());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  @Log.NT
  public SwerveModulePosition position() {
    return new SwerveModulePosition(module.getDrivePosition(), module.getRotation());
  }

  /**
   * Updates controllers based on an optimized desired state and actuates the module accordingly.
   *
   * <p>This method should be called periodically.
   *
   * @param desiredState The desired state of the module.
   */
  public void updateDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint = SwerveModuleState.optimize(desiredState, module.getRotation());
    updateDriveSpeed(MetersPerSecond.of(setpoint.speedMetersPerSecond));
    updateTurnRotation(setpoint.angle);
  }

  /**
   * Updates drive controller based on setpoint.
   *
   * <p>This is only used for Sysid.
   *
   * @param speed The desired speed of the module.
   */
  void updateDriveSpeed(Measure<Velocity<Distance>> speed) {
    module.setDriveVoltage(
        Volts.of(
            driveFeedforward.calculate(speed.in(MetersPerSecond))
                + driveFeedback.calculate(
                    module.getDriveVelocity().in(MetersPerSecond), speed.in(MetersPerSecond))));
  }

  /**
   * Updates turn controller based on setpoint.
   *
   * <p>This is only used for Sysid.
   *
   * @param rotation The desired rotation of the module.
   */
  void updateTurnRotation(Rotation2d rotation) {
    module.setTurnVoltage(
        Volts.of(turnFeedback.calculate(module.getRotation().getRadians(), rotation.getRadians())));
  }

  @Log.NT
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  public void setDriveVoltage(Measure<Voltage> voltage) {
    module.setDriveVoltage(voltage);
  }

  public void setTurnVoltage(Measure<Voltage> voltage) {
    module.setTurnVoltage(voltage);
  }

  public void resetEncoders() {
    module.resetEncoders();
  }

  @Override
  public void close() {
    module.close();
  }
}
