package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SimModule implements ModuleIO {

  private final DCMotorSim drive =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Driving.FF.SPARK.V, Driving.FF.SPARK.kA_linear),
          DCMotor.getNeoVortex(1),
          1 / Driving.GEARING);
  private final DCMotorSim turn =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(Turning.FF.V, Turning.FF.A),
          DCMotor.getNeo550(1),
          1 / Turning.MOTOR_GEARING);

  private final PIDController driveFeedback =
      new PIDController(Driving.PID.SIM.P, Driving.PID.SIM.I, Driving.PID.SIM.D);
  private final PIDController turnFeedback =
      new PIDController(Turning.PID.SIM.P, Turning.PID.SIM.I, Turning.PID.SIM.D);

  private final SimpleMotorFeedforward driveTranslationFeedforward =
      new SimpleMotorFeedforward(
          Driving.FF.SPARK.S, Driving.FF.SPARK.V, Driving.FF.SPARK.kA_linear);
  private final SimpleMotorFeedforward driveRotationFeedforward =
      new SimpleMotorFeedforward(
          Driving.FF.SPARK.S, Driving.FF.SPARK.V, Driving.FF.SPARK.kA_angular);

  private SwerveModuleState setpoint = new SwerveModuleState();

  private final String name;

  public SimModule(String name) {
    this.name = name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    drive.setInputVoltage(voltage);
    drive.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turn.setInputVoltage(voltage);
    turn.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double drivePosition() {
    return drive.getAngularPositionRad();
  }

  @Override
  public double driveVelocity() {
    return drive.getAngularVelocityRadPerSec();
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.fromRadians(turn.getAngularPositionRad());
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public SwerveModuleState state() {
    return new SwerveModuleState(driveVelocity(), rotation());
  }

  @Override
  public SwerveModulePosition position() {
    return new SwerveModulePosition(drivePosition(), rotation());
  }

  @Override
  public SwerveModuleState desiredState() {
    return setpoint;
  }

  @Override
  public void resetEncoders() {
    drive.setState(VecBuilder.fill(0, 0));
    turn.setState(VecBuilder.fill(0, 0));
  }

  @Override
  public void setDriveSetpoint(double velocity) {}

  @Override
  public void setTurnSetpoint(double angle) {}

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    setpoint = SwerveModuleState.optimize(setpoint, rotation());
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(rotation()).getCos();

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

    double driveVolts =
        driveTVolts + driveRVolts; // original: double driveVolts = driveTVolts * movementRatio +
    // driveRVolts * (1 - movementRatio);

    if (mode == ControlMode.CLOSED_LOOP_VELOCITY) {
      driveVolts += driveFeedback.calculate(driveVelocity(), setpoint.speedMetersPerSecond);
    }

    double turnVolts = turnFeedback.calculate(rotation().getRadians(), setpoint.angle.getRadians());

    setDriveVoltage(driveVolts);
    setTurnVoltage(turnVolts);

    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;

    double turnVolts = turnFeedback.calculate(rotation().getRadians(), setpoint.angle.getRadians());

    setDriveVoltage(voltage);
    setTurnVoltage(turnVolts);
  }

  @Override
  public void close() {}
}
