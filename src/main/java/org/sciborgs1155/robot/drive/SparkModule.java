package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Set;
import monologue.Annotations.Log;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

public class SparkModule implements ModuleIO {
  private final CANSparkFlex driveMotor; // Neo Vortex
  private final CANSparkMax turnMotor; // Neo 550

  private final RelativeEncoder driveEncoder;
  private final SparkAbsoluteEncoder turningEncoder;

  private final SparkPIDController drivePID;
  private final SparkPIDController turnPID;

  private final SimpleMotorFeedforward driveFF;
  private final Rotation2d angularOffset;

  private double lastPosition;
  private double lastVelocity;

  @Log.NT private SwerveModuleState setpoint = new SwerveModuleState();

  private final String name;

  public SparkModule(int drivePort, int turnPort, Rotation2d angularOffset, String name) {
    driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    drivePID = driveMotor.getPIDController();
    driveFF =
        new SimpleMotorFeedforward(
            Driving.FF.SPARK.S, Driving.FF.SPARK.V, Driving.FF.SPARK.kA_linear); // TODO: Re-tune

    check(driveMotor, driveMotor.restoreFactoryDefaults());

    // TODO: Re-tune
    check(driveMotor, drivePID.setP(Driving.PID.SPARK.P));
    check(driveMotor, drivePID.setI(Driving.PID.SPARK.I));
    check(driveMotor, drivePID.setD(Driving.PID.SPARK.D));
    check(driveMotor, drivePID.setFeedbackDevice(driveEncoder));

    check(driveMotor, driveMotor.setIdleMode(IdleMode.kBrake));
    check(driveMotor, driveMotor.setSmartCurrentLimit((int) Driving.CURRENT_LIMIT.in(Amps)));
    check(driveMotor, driveEncoder.setPositionConversionFactor(Driving.POSITION_FACTOR.in(Meters)));
    check(
        driveMotor,
        driveEncoder.setVelocityConversionFactor(Driving.VELOCITY_FACTOR.in(MetersPerSecond)));
    check(driveMotor, driveEncoder.setAverageDepth(16));
    check(driveMotor, driveEncoder.setMeasurementPeriod(32));
    check(
        driveMotor,
        SparkUtils.configureFrameStrategy(
            driveMotor,
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.INTEGRATED),
            false));
    check(driveMotor, driveMotor.burnFlash());

    turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    turningEncoder = turnMotor.getAbsoluteEncoder();
    turnPID = turnMotor.getPIDController();

    check(turnMotor, turnMotor.restoreFactoryDefaults());

    // TODO: Re-tune
    check(turnMotor, turnPID.setP(Turning.PID.SPARK.P));
    check(turnMotor, turnPID.setI(Turning.PID.SPARK.I));
    check(turnMotor, turnPID.setD(Turning.PID.SPARK.D));
    check(turnMotor, turnPID.setPositionPIDWrappingEnabled(true));
    check(turnMotor, turnPID.setPositionPIDWrappingMinInput(-Math.PI));
    check(turnMotor, turnPID.setPositionPIDWrappingMaxInput(Math.PI));
    check(turnMotor, turnPID.setFeedbackDevice(turningEncoder));

    check(turnMotor, turnMotor.setIdleMode(IdleMode.kBrake));
    check(turnMotor, turnMotor.setSmartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)));
    turningEncoder.setInverted(Turning.ENCODER_INVERTED);
    check(turnMotor);
    check(
        turnMotor, turningEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians)));
    check(
        turnMotor,
        turningEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond)));
    check(turnMotor, turningEncoder.setAverageDepth(2));
    check(
        turnMotor,
        SparkUtils.configureFrameStrategy(
            turnMotor,
            Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
            Set.of(Sensor.ABSOLUTE),
            false));
    SparkUtils.addChecker(
        () ->
            check(
                turnMotor,
                SparkUtils.configureFrameStrategy(
                    turnMotor,
                    Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                    Set.of(Sensor.ABSOLUTE),
                    false)));
    check(turnMotor, turnMotor.burnFlash());

    register(driveMotor);
    register(turnMotor);

    resetEncoders();

    this.angularOffset = angularOffset;
    this.name = name;
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
    check(driveMotor);
    log("current", driveMotor.getOutputCurrent());
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
    check(turnMotor);
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public double drivePosition() {
    lastPosition = SparkUtils.wrapCall(driveMotor, driveEncoder.getPosition()).orElse(lastPosition);
    // account for rotation of turn motor on rotation of drive motor
    return lastPosition - turningEncoder.getPosition() * COUPLING_RATIO;
  }

  @Override
  public double driveVelocity() {
    lastVelocity = SparkUtils.wrapCall(driveMotor, driveEncoder.getVelocity()).orElse(lastVelocity);
    return lastVelocity;
  }

  @Override
  public Rotation2d rotation() {
    return Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset);
  }

  @Override
  public void resetEncoders() {
    driveEncoder.setPosition(0);
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
  public void setDriveSetpoint(double velocity) {
    drivePID.setReference(velocity, ControlType.kVelocity, 0, driveFF.calculate(velocity));
  }

  @Override
  public void setTurnSetpoint(double angle) {
    turnPID.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    setpoint = SwerveModuleState.optimize(setpoint, rotation());
    // Scale setpoint by cos of turning error to reduce tread wear
    setpoint.speedMetersPerSecond *= setpoint.angle.minus(rotation()).getCos();

    if (mode == ControlMode.OPEN_LOOP_VELOCITY) {
      setDriveVoltage(driveFF.calculate(setpoint.speedMetersPerSecond));
    } else {
      setDriveSetpoint(setpoint.speedMetersPerSecond);
    }

    setTurnSetpoint(setpoint.angle.getRadians());
    this.setpoint = setpoint;
  }

  @Override
  public void updateInputs(Rotation2d angle, double voltage) {
    setpoint.angle = angle;
    setDriveVoltage(voltage);
    setTurnSetpoint(angle.getRadians());
  }

  @Override
  public void close() {
    driveMotor.close();
    turnMotor.close();
  }
}
