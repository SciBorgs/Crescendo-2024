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
   * @param mode The control mode to use when calculating drive voltage.
   */
  public void updateDesiredState(SwerveModuleState desiredState, ControlMode mode) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint = SwerveModuleState.optimize(desiredState, module.rotation());
    // Scale setpoint by cos of turning error to improve tread wear
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(module.rotation()).getCos();

    double driveVolts =
        switch (mode) {
          case OPEN_LOOP_VELOCITY -> driveFeedforward.calculate(setpoint.speedMetersPerSecond);
          case CLOSED_LOOP_VELOCITY ->
              driveFeedforward.calculate(
                      driveFeedback.getSetpoint(),
                      setpoint.speedMetersPerSecond,
                      PERIOD.in(Seconds))
                  + driveFeedback.calculate(module.driveVelocity(), setpoint.speedMetersPerSecond);
        };

    double turnVolts =
        turnFeedback.calculate(module.rotation().getRadians(), setpoint.angle.getRadians());

    module.setDriveVoltage(driveVolts);
    module.setTurnVoltage(turnVolts);
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
        turnFeedback.calculate(module.rotation().getRadians(), setpoint.angle.getRadians());

    module.setDriveVoltage(voltage);
    module.setTurnVoltage(turnVolts);
  }

  @Log.NT
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  public void resetEncoders() {
    module.resetEncoders();
  }

  public void updatePID() {
    driveFeedback.setPID(drivingP.get(), drivingI.get(), drivingD.get());
    turnFeedback.setPID(turningP.get(), turningI.get(), turningD.get());
  }

  @Override
  public void close() {
    module.close();
  }
}
