package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.lib.Tuning;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

/** Class to encapsulate a REV Max Swerve module */
public class SwerveModule implements Logged, AutoCloseable {

  /** The method to use when controlling the drive motor. */
  public static enum ControlMode {
    CLOSED_LOOP_VELOCITY,
    OPEN_LOOP_VELOCITY;
  }

  private final ModuleIO hardware;

  private final PIDController driveFeedback;
  private final PIDController turnFeedback;

  private final SimpleMotorFeedforward driveTranslationFeedforward;
  private final SimpleMotorFeedforward driveRotationFeedforward;

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
   * @param hardware talon OR flex swerve module
   * @param angularOffset offset from drivetrain
   */
  public SwerveModule(ModuleIO hardware, Rotation2d angularOffset, String name) {
    this.hardware = hardware;
    this.name = name;
    driveFeedback = new PIDController(Driving.PID.P, Driving.PID.I, Driving.PID.D);
    turnFeedback = new PIDController(Turning.PID.P, Turning.PID.I, Turning.PID.D);
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

    driveTranslationFeedforward =
        new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.kA_linear);
    driveRotationFeedforward =
        new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.kA_angular);

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

    // Calculate two feedforward values for using different kA depending on if the robot is rotating
    // or translating.
    double driveTVolts =
        switch (mode) {
          case CLOSED_LOOP_VELOCITY ->
              driveTranslationFeedforward.calculate(
                  this.setpoint.speedMetersPerSecond,
                  setpoint.speedMetersPerSecond,
                  PERIOD.in(Seconds));
          case OPEN_LOOP_VELOCITY ->
              driveTranslationFeedforward.calculate(setpoint.speedMetersPerSecond);
        };

    double driveRVolts =
        switch (mode) {
          case CLOSED_LOOP_VELOCITY ->
              driveRotationFeedforward.calculate(
                  this.setpoint.speedMetersPerSecond,
                  setpoint.speedMetersPerSecond,
                  PERIOD.in(Seconds));
          case OPEN_LOOP_VELOCITY ->
              driveRotationFeedforward.calculate(setpoint.speedMetersPerSecond);
        };

    double driveVolts = driveTVolts * movementRatio + driveRVolts * (1 - movementRatio);

    if (mode == ControlMode.CLOSED_LOOP_VELOCITY) {
      driveVolts +=
          driveFeedback.calculate(hardware.driveVelocity(), setpoint.speedMetersPerSecond);
    }

    double turnVolts =
        turnFeedback.calculate(hardware.rotation().getRadians(), setpoint.angle.getRadians());

    hardware.setDriveVoltage(driveVolts);
    hardware.setTurnVoltage(turnVolts);

    this.setpoint = setpoint;
  }

  /**
   * Updates the drive voltage and turn angle.
   *
   * <p>This is useful for SysId characterization, but should never be run otherwise.
   *
   * @param angle The desired angle of the module.
   * @param voltage The voltage to supply to the drive motor.
   */
  public void updateDriveVoltage(Rotation2d angle, double voltage) {
    setpoint.angle = angle;

    double turnVolts =
        turnFeedback.calculate(hardware.rotation().getRadians(), setpoint.angle.getRadians());

    hardware.setDriveVoltage(voltage);
    hardware.setTurnVoltage(turnVolts);
  }

  @Log.NT
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  public void resetEncoders() {
    hardware.resetEncoders();
  }

  public void updatePID() {
    driveFeedback.setPID(drivingP.get(), drivingI.get(), drivingD.get());
    turnFeedback.setPID(turningP.get(), turningI.get(), turningD.get());
  }

  @Override
  public void close() {
    hardware.close();
  }
}
