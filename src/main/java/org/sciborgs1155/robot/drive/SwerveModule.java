package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import java.util.List;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

/** Class to encapsulate a REV Max Swerve module */
public class SwerveModule implements Logged, AutoCloseable {
  private final ModuleIO module;

  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  private final SimpleMotorFeedforward driveFeedforward;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public final String name;

  private final DoubleEntry drivingD = Tuning.entry("/Robot/drive/driving/D", Driving.PID.D);
  private final DoubleEntry drivingI = Tuning.entry("/Robot/drive/driving/I", Driving.PID.I);
  private final DoubleEntry drivingP = Tuning.entry("/Robot/drive/driving/P", Driving.PID.P);

  private final DoubleEntry turningD = Tuning.entry("/Robot/drive/turning/D", Turning.PID.D);
  private final DoubleEntry turningI = Tuning.entry("/Robot/drive/turning/I", Turning.PID.I);
  private final DoubleEntry turningP = Tuning.entry("/Robot/drive/turning/P", Turning.PID.P);

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
    return new SwerveModuleState(module.driveVelocity(), module.rotation());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  @Log.NT
  public SwerveModulePosition position() {
    return new SwerveModulePosition(module.drivePosition(), module.rotation());
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
    setpoint = SwerveModuleState.optimize(desiredState, module.rotation());
    // Calculate cosine of turning error
    double cosScalar = (setpoint.angle.minus(module.rotation())).getCos();
    updateDriveSpeed(setpoint.speedMetersPerSecond * Math.abs(cosScalar));
    updateTurnRotation(setpoint.angle);
  }

  /**
   * Updates drive controller based on setpoint.
   *
   * <p>This is only used for Sysid.
   *
   * @param speed The desired speed of the module.
   */
  void updateDriveSpeed(double speed) {
    double driveFF = driveFeedforward.calculate(speed);
    double driveVoltage = driveFF + driveFeedback.calculate(module.driveVelocity(), speed);
    module.setDriveVoltage(driveVoltage);
  }

  /**
   * Updates turn controller based on setpoint.
   *
   * <p>This is only used for Sysid.
   *
   * @param rotation The desired rotation of the module.
   */
  void updateTurnRotation(Rotation2d rotation) {
    double turnVoltage =
        turnFeedback.calculate(module.rotation().getRadians(), rotation.getRadians());
    module.setTurnVoltage(turnVoltage);
  }

  @Log.NT
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  public void setDriveVoltage(double voltage) {
    module.setDriveVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    module.setTurnVoltage(voltage);
  }

  public void resetEncoders() {
    module.resetEncoders();
  }

  public void updatePID() {
    driveFeedback.setPID(drivingP.get(), drivingI.get(), drivingD.get());
    turnFeedback.setPID(turningP.get(), turningI.get(), turningD.get());

    Tuning.updateDoubles(List.of(drivingP, drivingI, drivingD, turningP, turningI, turningD));
  }

  @Override
  public void close() {
    module.close();
  }
}
